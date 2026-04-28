/*************************************************************
 * SBUS Handler for Raspberry Pi
 * 
 * Implementation of the SbusHandler class
 * 
 * Author: Raymond Turrisi (modified for Pi by Claude)
 * Last Edit: 2025-03-29
 ************************************************************/

 #include "sbus_handler.h"
 #include <string.h>
 #include <stdio.h>
 #include <fcntl.h>
 #include <unistd.h>
 #include <termios.h>
 #include <errno.h>
 #include <sys/ioctl.h>
 #include <time.h>
 
 // Custom BOTHER flag for custom baud rate
 #define BOTHER 0010000
 
 /**
  * Constructor
  */
 SbusHandler::SbusHandler(const std::string& device)
     : frame_index_(0)
     , in_frame_(false)
     , last_byte_time_(0)
     , start_time_(0)
     , last_valid_frame_time_(0)
     , channel_17_(false)
     , channel_18_(false)
     , frame_lost_(false)
     , failsafe_(false)
     , frames_received_(0)
     , frame_errors_(0)
     , consecutive_frame_losses_(0)
     , controller_connected_(false)
     , frame_valid_(false)
     , consecutive_good_frames_(0)
     , fd_(-1)
     , initialized_(false)
     , device_(device)
 {
     // Initialize channels with default values
     for (int i = 0; i < SBUS_NUM_CHANNELS; i++) {
         channels_[i] = SBUS_MID_VALUE;
     }
     
     // Initialize frame buffer
     memset(frame_, 0, SBUS_FRAME_SIZE);
     
     // Record start time
     start_time_ = micros();
     last_valid_frame_time_ = start_time_;
 }
 
 /**
  * Destructor
  */
 SbusHandler::~SbusHandler() {
     close();
 }
 
 /**
  * Initialize the SBUS handler and open the UART
  */
 bool SbusHandler::initialize() {
     std::lock_guard<std::mutex> lock(mutex_);
     
     // Clean up if already initialized
     if (initialized_) {
         close();
     }
     
     // Open the UART device
     fd_ = ::open(device_.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK);
     if (fd_ < 0) {
         perror("Failed to open UART device");
         return false;
     }
     
     // Configure the UART
     if (!configureUartForSbus()) {
         ::close(fd_);
         fd_ = -1;
         return false;
     }
     
     initialized_ = true;
     return true;
 }
 
 /**
  * Close the SBUS handler
  */
 void SbusHandler::close() {
     std::lock_guard<std::mutex> lock(mutex_);
     
     if (initialized_ && fd_ >= 0) {
         ::close(fd_);
         fd_ = -1;
     }
     initialized_ = false;
 }
 
 /**
  * Get current time in microseconds
  */
 uint64_t SbusHandler::micros() const {
     struct timespec ts;
     clock_gettime(CLOCK_MONOTONIC, &ts);
     return (uint64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
 }
 
 /**
  * Configure UART for SBUS with custom baud rate
  */
 bool SbusHandler::configureUartForSbus() {
     // First set up the port with standard termios
     struct termios tio;
     
     // Get current settings
     if (tcgetattr(fd_, &tio) < 0) {
         perror("Failed to get UART attributes");
         return false;
     }
     
     // Set a standard baud rate first (this step is necessary)
     cfsetispeed(&tio, B38400);
     cfsetospeed(&tio, B38400);
     
     // Set frame format: 8E2 (8 data bits, even parity, 2 stop bits)
     tio.c_cflag &= ~CSIZE;
     tio.c_cflag |= CS8;         // 8 data bits
     tio.c_cflag |= PARENB;      // Even parity
     tio.c_cflag &= ~PARODD;     // (not odd parity)
     tio.c_cflag |= CSTOPB;      // 2 stop bits
     
     // No flow control
     tio.c_cflag &= ~CRTSCTS;    // No hardware flow control
     tio.c_iflag &= ~(IXON | IXOFF | IXANY); // No software flow control
     
     // Raw input mode
     tio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
     tio.c_iflag &= ~(INLCR | ICRNL);
     tio.c_oflag &= ~OPOST;
     
     // Non-blocking with VMIN=0, VTIME=0
     tio.c_cc[VMIN] = 0;
     tio.c_cc[VTIME] = 0;
     
     // Apply standard settings first
     if (tcsetattr(fd_, TCSANOW, &tio) < 0) {
         perror("Failed to set UART attributes");
         return false;
     }
     
     // Now set custom speed using Linux-specific ioctl
     struct custom_termios2 {
         tcflag_t c_iflag;
         tcflag_t c_oflag;
         tcflag_t c_cflag;
         tcflag_t c_lflag;
         cc_t c_line;
         cc_t c_cc[19];
         speed_t c_ispeed;
         speed_t c_ospeed;
     };
     
     // TCGETS2 = 0x802C542A, TCSETS2 = 0x402C542B
     struct custom_termios2 tio2;
     if (ioctl(fd_, 0x802C542A, &tio2) < 0) {
         perror("TCGETS2 ioctl failed");
         return false;
     }
     
     // Set custom baudrate
     tio2.c_cflag &= ~0000017;  // Remove CBAUD
     tio2.c_cflag |= BOTHER;
     tio2.c_ispeed = SBUS_BAUDRATE;
     tio2.c_ospeed = SBUS_BAUDRATE;
     
     if (ioctl(fd_, 0x402C542B, &tio2) < 0) {
         perror("TCSETS2 ioctl failed");
         return false;
     }
     
     // Flush buffers
     tcflush(fd_, TCIOFLUSH);
     
     return true;
 }
 
 /**
  * Decode SBUS channels from a frame
  */
 bool SbusHandler::decodeFrame() {
     // Validate frame header
     if (frame_[SBUS_HEADER_OFFSET] != SBUS_START_BYTE) {
         return false;
     }

     // Validate frame footer.
     //  - 0x00 is the normal SBUS end byte.
     //  - SBUS2 receivers use 0x04, 0x14, 0x24, 0x34 to mark which
     //    inter-frame telemetry slot follows; the low nibble is 0x04
     //    in all four cases.
     // Any other value here means the frame is misaligned or
     // corrupted and should be rejected.
     {
         uint8_t footer = frame_[SBUS_FOOTER_OFFSET];
         if (footer != SBUS_END_BYTE && (footer & 0x0F) != 0x04) {
             return false;
         }
     }

     // Extract the 16 channels (11 bits each)
     // Optimized bit-shifting logic for RadioLink AT9S Pro
     uint16_t new_channels[SBUS_NUM_CHANNELS];
     
     new_channels[0]  = ((frame_[1] | (frame_[2] << 8)) & 0x07FF);
     new_channels[1]  = ((frame_[2] >> 3) | (frame_[3] << 5)) & 0x07FF;
     new_channels[2]  = ((frame_[3] >> 6) | (frame_[4] << 2) | (frame_[5] << 10)) & 0x07FF;
     new_channels[3]  = ((frame_[5] >> 1) | (frame_[6] << 7)) & 0x07FF;
     new_channels[4]  = ((frame_[6] >> 4) | (frame_[7] << 4)) & 0x07FF;
     new_channels[5]  = ((frame_[7] >> 7) | (frame_[8] << 1) | (frame_[9] << 9)) & 0x07FF;
     new_channels[6]  = ((frame_[9] >> 2) | (frame_[10] << 6)) & 0x07FF;
     new_channels[7]  = ((frame_[10] >> 5) | (frame_[11] << 3)) & 0x07FF;
     new_channels[8]  = ((frame_[12] | (frame_[13] << 8)) & 0x07FF);
     new_channels[9]  = ((frame_[13] >> 3) | (frame_[14] << 5)) & 0x07FF;
     new_channels[10] = ((frame_[14] >> 6) | (frame_[15] << 2) | (frame_[16] << 10)) & 0x07FF;
     new_channels[11] = ((frame_[16] >> 1) | (frame_[17] << 7)) & 0x07FF;
     new_channels[12] = ((frame_[17] >> 4) | (frame_[18] << 4)) & 0x07FF;
     new_channels[13] = ((frame_[18] >> 7) | (frame_[19] << 1) | (frame_[20] << 9)) & 0x07FF;
     new_channels[14] = ((frame_[20] >> 2) | (frame_[21] << 6)) & 0x07FF;
     new_channels[15] = ((frame_[21] >> 5) | (frame_[22] << 3)) & 0x07FF;
     
     // Extract the flags from byte 23
     uint8_t flags = frame_[SBUS_FLAGS_OFFSET];
     bool new_channel_17 = (flags & SBUS_FLAG_CHANNEL_17) != 0;
     bool new_channel_18 = (flags & SBUS_FLAG_CHANNEL_18) != 0;
     bool new_frame_lost = (flags & SBUS_FLAG_FRAME_LOST) != 0;
     bool new_failsafe = (flags & SBUS_FLAG_FAILSAFE) != 0;
     
     // Validate and update channel values.
     // Decoded channel values are already 11-bit (& 0x07FF), so the
     // raw range is guaranteed to be [0, 2047]. We additionally
     // require each channel to fall within the canonical Futaba SBUS
     // range [SBUS_MIN_VALUE, SBUS_MAX_VALUE] = [172, 1812], which
     // corresponds to ~880-2120 us PWM. Anything outside this band
     // is assumed to be the result of a misaligned/corrupted frame
     // (the previous looser check of [100, 2100] caught only values
     // 0-99, since the upper bound was unreachable after the mask).
     // If a future radio is configured for extended endpoints
     // (>125%), widen SBUS_MIN_VALUE / SBUS_MAX_VALUE in the header.
     bool valid_frame = true;
     for (int i = 0; i < SBUS_NUM_CHANNELS; i++) {
         if (new_channels[i] < SBUS_MIN_VALUE ||
             new_channels[i] > SBUS_MAX_VALUE) {
             valid_frame = false;
             break;
         }
     }
     
     // If the frame decoded cleanly, update all values atomically.
     if (valid_frame) {
         std::lock_guard<std::mutex> lock(mutex_);

         for (int i = 0; i < SBUS_NUM_CHANNELS; i++) {
             channels_[i] = new_channels[i];
         }

         channel_17_ = new_channel_17;
         channel_18_ = new_channel_18;
         frame_lost_ = new_frame_lost;
         failsafe_   = new_failsafe;

         // Reset wire-level error counter on a clean decode.
         consecutive_frame_losses_ = 0;
         last_valid_frame_time_    = micros();

         // === Per-frame validity + asymmetric hysteresis ===
         // A frame is "valid" only if the receiver also flagged it
         // as a fresh transmitter frame (no failsafe, no frame_lost).
         //   - frame_valid_ flips per frame, no debounce.
         //   - controller_connected_ flips false on a single bad
         //     frame, but requires SBUS_HYSTERESIS_GOOD_FRAMES
         //     consecutive valid frames to flip back to true.
         frame_valid_ = !new_frame_lost && !new_failsafe;
         if (frame_valid_) {
             consecutive_good_frames_++;
             if (consecutive_good_frames_ >= SBUS_HYSTERESIS_GOOD_FRAMES) {
                 controller_connected_ = true;
             }
         } else {
             consecutive_good_frames_ = 0;
             controller_connected_    = false;
         }
     }
     
     return valid_frame;
 }
 
 /**
  * Process available SBUS data
  * Returns true if a new frame was received
  */
 bool SbusHandler::update() {
     if (!initialized_ || fd_ < 0) {
         return false;
     }
     
     uint8_t byte;
     uint8_t buffer[64]; // Buffer for bulk reads
     ssize_t nbytes;
     bool new_frame = false;
     const uint32_t FRAME_GAP_THRESHOLD_US = 3000;  // Reduced threshold for RadioLink receivers
     
     // First, try to read multiple bytes at once to prevent buffer overrun
     nbytes = read(fd_, buffer, sizeof(buffer));
     
     if (nbytes > 0) {
         // Process each byte in the buffer
         for (int i = 0; i < nbytes; i++) {
             byte = buffer[i];
             uint64_t now = micros();
             uint64_t time_since_last_byte = now - last_byte_time_;
             last_byte_time_ = now;
             
             // Always check for start byte, even if we're in a frame
             // This improves synchronization for RadioLink receivers
             if (byte == SBUS_START_BYTE && (time_since_last_byte > FRAME_GAP_THRESHOLD_US || !in_frame_)) {
                 // Start of a new frame
                 // If we were already in a frame, it was incomplete - discard it
                 if (in_frame_ && frame_index_ > 0) {
                     frame_errors_++;
                     
                     // Increment consecutive frame losses when a frame is incomplete
                     consecutive_frame_losses_++;
                 }
                 
                 in_frame_ = true;
                 frame_index_ = 0;
                 frame_[frame_index_++] = byte;
                 continue;
             }
             
             // Store the byte if we're in a frame
             if (in_frame_) {
                 if (frame_index_ < SBUS_FRAME_SIZE) {
                     frame_[frame_index_++] = byte;
                     
                     // Check if we have a complete frame
                     if (frame_index_ == SBUS_FRAME_SIZE) {
                         // Process the frame
                         if (decodeFrame()) {
                             frames_received_++;
                             new_frame = true;
                         } else {
                             frame_errors_++;
                             consecutive_frame_losses_++;
                             // A failed decode means the latest frame
                             // is not trustworthy. Drop frame_valid_
                             // immediately and reset the hysteresis
                             // counter; updateConnectionStatus() will
                             // decide whether the accumulated losses
                             // warrant flipping controller_connected_.
                             {
                                 std::lock_guard<std::mutex> lock(mutex_);
                                 frame_valid_ = false;
                                 consecutive_good_frames_ = 0;
                             }
                         }
                         
                         // Reset for next frame
                         in_frame_ = false;
                         frame_index_ = 0;
                     }
                 } else {
                     // Frame too long - reset
                     in_frame_ = false;
                     frame_index_ = 0;
                     frame_errors_++;
                     consecutive_frame_losses_++;
                 }
             }
         }
     } else if (nbytes < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
         // Check for read errors (except EAGAIN which is normal for non-blocking reads)
         perror("Error reading from UART");
         return false;
     }
     
     // Check for frame timeout - helps recover from partial frames
     uint64_t now = micros();
     if (in_frame_ && (now - last_byte_time_ > 10000)) { // 10ms timeout
         in_frame_ = false;
         frame_index_ = 0;
         // Count as an error for consecutive loss tracking
         consecutive_frame_losses_++;
     }
     
     // Update controller connected status
     updateConnectionStatus();
     
     return new_frame;
 }
 
 /**
  * Update controller connection status based on frame reception and flags
  */
 void SbusHandler::updateConnectionStatus() {
     std::lock_guard<std::mutex> lock(mutex_);

     uint64_t now = micros();
     uint64_t time_since_last_frame = now - last_valid_frame_time_;

     // The per-frame state (frame_valid_, controller_connected_,
     // consecutive_good_frames_) is updated atomically inside
     // decodeFrame() each time a fresh frame arrives.
     //
     // This function runs every call to update() - including calls
     // where no fresh frame was decoded - and is only responsible
     // for the staleness backstop. If too much wall time has
     // elapsed, or too many consecutive wire-level errors have
     // piled up since the last good frame, the cached channel data
     // is no longer trustworthy and we invalidate everything
     // regardless of what flags the last decoded frame happened to
     // carry.
     bool stale =
         (time_since_last_frame > (SBUS_SIGNAL_LOSS_TIMEOUT_MS * 1000)) ||
         (consecutive_frame_losses_ > SBUS_MAX_CONSECUTIVE_LOSS);

     if (stale) {
         frame_valid_             = false;
         controller_connected_    = false;
         consecutive_good_frames_ = 0;
     }
 }
 
 /**
  * Get the raw value of a specific channel
  */
 uint16_t SbusHandler::getChannel(uint8_t channel) const {
     std::lock_guard<std::mutex> lock(mutex_);
     
     if (channel < SBUS_NUM_CHANNELS) {
         return channels_[channel];
     }
     return SBUS_MID_VALUE; // Return center value for invalid channels
 }
 
 /**
  * Map a channel value to a specific output range
  */
 int SbusHandler::getChannelScaled(uint8_t channel, int min_output, int max_output) const {
     std::lock_guard<std::mutex> lock(mutex_);
     
     if (channel >= SBUS_NUM_CHANNELS) {
         return 0;
     }
     
     uint16_t value = channels_[channel];
     
     // Constrain value to valid SBUS range
     if (value < SBUS_MIN_VALUE) value = SBUS_MIN_VALUE;
     if (value > SBUS_MAX_VALUE) value = SBUS_MAX_VALUE;
     
     // Scale to output range
     return min_output + ((value - SBUS_MIN_VALUE) * (max_output - min_output)) / 
            (SBUS_MAX_VALUE - SBUS_MIN_VALUE);
 }
 
 /**
  * Get the status of channel 17
  */
 bool SbusHandler::getChannel17() const {
     std::lock_guard<std::mutex> lock(mutex_);
     return channel_17_;
 }
 
 /**
  * Get the status of channel 18
  */
 bool SbusHandler::getChannel18() const {
     std::lock_guard<std::mutex> lock(mutex_);
     return channel_18_;
 }
 
 /**
  * Check if frame is lost
  */
 bool SbusHandler::isFrameLost() const {
     std::lock_guard<std::mutex> lock(mutex_);
     return frame_lost_;
 }
 
 /**
  * Check if failsafe is active
  */
 bool SbusHandler::isFailsafe() const {
     std::lock_guard<std::mutex> lock(mutex_);
     return failsafe_;
 }
 
 /**
  * Get the number of frames received
  */
 unsigned long SbusHandler::getFramesReceived() const {
     std::lock_guard<std::mutex> lock(mutex_);
     return frames_received_;
 }
 
 /**
  * Get the number of frame errors
  */
 unsigned long SbusHandler::getFrameErrors() const {
     std::lock_guard<std::mutex> lock(mutex_);
     return frame_errors_;
 }
 
 /**
  * Get the frame rate in Hz
  */
 float SbusHandler::getFrameRate() const {
     std::lock_guard<std::mutex> lock(mutex_);
     uint64_t now = micros();
     float runtime = (now - start_time_) / 1000000.0f;
     if (runtime > 0) {
         return frames_received_ / runtime;
     }
     return 0.0f;
 }
 
 /**
  * Check if the RC controller is still connected (debounced).
  * True only after SBUS_HYSTERESIS_GOOD_FRAMES consecutive valid
  * frames since the last bad event.
  */
 bool SbusHandler::isControllerConnected() const {
     std::lock_guard<std::mutex> lock(mutex_);
     return controller_connected_;
 }

 /**
  * Per-frame validity. True iff the most recent decoded frame had
  * no failsafe / no frame_lost AND the link is not stale, with no
  * debounce. Use this to gate per-cycle outputs (e.g. RC thrust).
  */
 bool SbusHandler::isFrameValid() const {
     std::lock_guard<std::mutex> lock(mutex_);
     return frame_valid_;
 }

 /**
  * Count of consecutive valid frames since the last bad frame.
  * Useful for diagnostics and the appcast.
  */
 unsigned long SbusHandler::getConsecutiveGoodFrames() const {
     std::lock_guard<std::mutex> lock(mutex_);
     return consecutive_good_frames_;
 }
 
 /**
  * Get time since last valid frame in microseconds
  */
 uint64_t SbusHandler::getTimeSinceLastFrame() const {
     std::lock_guard<std::mutex> lock(mutex_);
     uint64_t now = micros();
     return now - last_valid_frame_time_;
 }
 
 /**
  * Get count of consecutive lost frames
  */
 unsigned long SbusHandler::getConsecutiveFrameLosses() const {
     std::lock_guard<std::mutex> lock(mutex_);
     return consecutive_frame_losses_;
 }
 
 /**
  * TODO: Initialize for transmitting data back to RC receiver
  */
 bool SbusHandler::initializeTx(const std::string& device) {
     // TODO: Implement initialization for transmitting SBUS data
     // This would open the device for writing and configure it appropriately
     return false;
 }
 
 /**
  * TODO: Set a channel value for transmission
  */
 bool SbusHandler::setChannel(uint8_t channel, uint16_t value) {
     // TODO: Implement setting channel value for transmission buffer
     // This would store the value in a transmit buffer for later sending
     return false;
 }
 
 /**
  * TODO: Send a frame to the RC receiver
  */
 bool SbusHandler::sendFrame() {
     // TODO: Implement sending an SBUS frame to the RC receiver
     // This would encode the channel values into a properly formatted SBUS frame and send it
     return false;
 }