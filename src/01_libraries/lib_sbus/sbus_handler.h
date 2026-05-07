/*************************************************************
 * SBUS Handler for Raspberry Pi
 *
 * A lightweight class for parsing SBUS data from RC receivers
 * Optimized for Raspberry Pi hardware
 *
 * Author: Raymond Turrisi (modified for Pi by Claude)
 * Last Edit: 2025-03-29
 ************************************************************/

 #ifndef SBUS_HANDLER_H
 #define SBUS_HANDLER_H

 #include <stdint.h>
 #include <string>
 #include <mutex>

 // SBUS protocol defines
 #define SBUS_UART_DEV       "/dev/ttyS0"    // Default UART device
 #define SBUS_BAUDRATE       100000          // SBUS baud rate
 #define SBUS_START_BYTE     0x0F            // Start byte
 #define SBUS_END_BYTE       0x00            // End byte (sometimes 0x04)
 #define SBUS_FRAME_SIZE     25              // Full frame size
 #define SBUS_NUM_CHANNELS   16              // Number of channels
 #define SBUS_HEADER_OFFSET  0
 #define SBUS_FLAGS_OFFSET   23
 #define SBUS_FOOTER_OFFSET  24

 // Bit flags
 #define SBUS_FLAG_CHANNEL_17        0x01
 #define SBUS_FLAG_CHANNEL_18        0x02
 #define SBUS_FLAG_FRAME_LOST        0x04
 #define SBUS_FLAG_FAILSAFE          0x08

 // Channel value ranges
 #define SBUS_MIN_VALUE      172     // Corresponds to RC pulse width of ~880µs
 #define SBUS_MID_VALUE      992     // Corresponds to RC pulse width of ~1500µs
 #define SBUS_MAX_VALUE      1812    // Corresponds to RC pulse width of ~2120µs

 // Signal loss detection defines
 #define SBUS_SIGNAL_LOSS_TIMEOUT_MS 500     // Signal considered lost after 500ms without frames
 #define SBUS_MAX_CONSECUTIVE_LOSS   20      // Max consecutive frames lost before signal is considered lost

 // Number of consecutive valid frames required to flip
 // controller_connected_ from false->true. A single bad frame flips
 // it back to false (asymmetric hysteresis: fast disconnect, slow
 // reconnect). At ~70 Hz framing, 3 ~= 42 ms.
 #define SBUS_HYSTERESIS_GOOD_FRAMES  3

 class SbusHandler {
 public:
     // Constructor and destructor
     SbusHandler(const std::string& device = SBUS_UART_DEV);
     ~SbusHandler();

     // Initialize and close methods
     bool initialize();
     void close();

     // Update method - call this regularly to process new data
     bool update();

     // Channel getters
     uint16_t getChannel(uint8_t channel) const;
     int getChannelScaled(uint8_t channel, int min_output, int max_output) const;

     // Status flag getters
     bool getChannel17() const;
     bool getChannel18() const;
     bool isFrameLost() const;
     bool isFailsafe() const;

     // Statistics getters
     unsigned long getFramesReceived() const;
     unsigned long getFrameErrors() const;
     float getFrameRate() const;

     // Connection state - two distinct booleans for two distinct uses.
     //
     //   isFrameValid()         - per-frame, instantaneous. True iff
     //                            the most recent SBUS frame was free
     //                            of failsafe / frame_lost flags AND
     //                            the link is not stale. Use this to
     //                            gate per-cycle commands such as RC
     //                            thrust output.
     //
     //   isControllerConnected() - debounced. Becomes true only after
     //                            SBUS_HYSTERESIS_GOOD_FRAMES
     //                            consecutive valid frames since the
     //                            last bad one; becomes false on a
     //                            single bad event. Use this for
     //                            mode switching, UI, and event
     //                            logging.
     bool isFrameValid() const;
     bool isControllerConnected() const;
     uint64_t getTimeSinceLastFrame() const;
     unsigned long getConsecutiveFrameLosses() const;
     unsigned long getConsecutiveGoodFrames() const;

     // TODO: Methods for sending data back to RC receiver
     bool initializeTx(const std::string& device = SBUS_UART_DEV);
     bool setChannel(uint8_t channel, uint16_t value);
     bool sendFrame();

 private:
     // Frame buffer and state
     uint8_t frame_[SBUS_FRAME_SIZE];
     int frame_index_;
     bool in_frame_;
     uint64_t last_byte_time_;
     uint64_t start_time_;
     uint64_t last_valid_frame_time_; // NEW: Time of last valid frame

     // Channel data
     uint16_t channels_[SBUS_NUM_CHANNELS];
     bool channel_17_;
     bool channel_18_;
     bool frame_lost_;
     bool failsafe_;

     // Statistics
     unsigned long frames_received_;
     unsigned long frame_errors_;
     unsigned long consecutive_frame_losses_; // NEW: Track consecutive losses
     bool controller_connected_; // NEW: Controller connection status

     // Hysteresis state for the split-API connection model.
     bool frame_valid_;                       // per-frame snapshot
     unsigned long consecutive_good_frames_;  // resets to 0 on any bad frame

     // UART file descriptor
     int fd_;
     bool initialized_;
     std::string device_;

     // Mutex for thread safety
     mutable std::mutex mutex_;

     // Private methods
     bool decodeFrame();
     bool configureUartForSbus();
     uint64_t micros() const;
     void updateConnectionStatus(); // NEW: Update controller connection status
 };

 #endif /* SBUS_HANDLER_H */
