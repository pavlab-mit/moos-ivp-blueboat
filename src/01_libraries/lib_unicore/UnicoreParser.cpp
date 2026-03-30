#include "UnicoreParser.h"
#include <iostream>
#include <cstring>
#include <cerrno>
#include <algorithm>
#include <cstdint>

#ifdef __APPLE__
#include <sys/ioctl.h>
#include <IOKit/serial/ioss.h>
#endif

// GPS epoch: January 6, 1980 00:00:00 UTC
static const int64_t GPS_EPOCH_UNIX = 315964800;
static const int SECONDS_PER_WEEK = 604800;
static const int GPS_LEAP_SECONDS = 18; // As of 2024

UnicoreParser::UnicoreParser(const std::string& port, int baudrate)
    : port_(port),
      baudrate_(baudrate),
      serial_fd_(-1)
{
    // Initialize tracking maps
    newData_[UnicoreMessageType::BESTNAVA] = false;
    newData_[UnicoreMessageType::UNIHEADINGA] = false;
    newData_[UnicoreMessageType::BESTVELA] = false;
    newData_[UnicoreMessageType::PSRDOPA] = false;
    newData_[UnicoreMessageType::STADOPA] = false;
    newData_[UnicoreMessageType::STADOPHA] = false;
    newData_[UnicoreMessageType::RTKSTATUSA] = false;
    newData_[UnicoreMessageType::UNKNOWN] = false;

    messageCounts_[UnicoreMessageType::BESTNAVA] = 0;
    messageCounts_[UnicoreMessageType::UNIHEADINGA] = 0;
    messageCounts_[UnicoreMessageType::BESTVELA] = 0;
    messageCounts_[UnicoreMessageType::PSRDOPA] = 0;
    messageCounts_[UnicoreMessageType::STADOPA] = 0;
    messageCounts_[UnicoreMessageType::STADOPHA] = 0;
    messageCounts_[UnicoreMessageType::RTKSTATUSA] = 0;
    messageCounts_[UnicoreMessageType::UNKNOWN] = 0;

    lastUpdateTime_[UnicoreMessageType::BESTNAVA] = 0;
    lastUpdateTime_[UnicoreMessageType::UNIHEADINGA] = 0;
    lastUpdateTime_[UnicoreMessageType::BESTVELA] = 0;
    lastUpdateTime_[UnicoreMessageType::PSRDOPA] = 0;
    lastUpdateTime_[UnicoreMessageType::STADOPA] = 0;
    lastUpdateTime_[UnicoreMessageType::STADOPHA] = 0;
    lastUpdateTime_[UnicoreMessageType::RTKSTATUSA] = 0;
    lastUpdateTime_[UnicoreMessageType::UNKNOWN] = 0;

    // Initialize numeric fields (strings are already initialized by default constructors)
    bestnava_.cpuIdle = 0;
    bestnava_.gpsWeek = 0;
    bestnava_.gpsSeconds = 0;
    bestnava_.lat = 0;
    bestnava_.lon = 0;
    bestnava_.height = 0;
    bestnava_.undulation = 0;
    bestnava_.latSigma = 0;
    bestnava_.lonSigma = 0;
    bestnava_.hgtSigma = 0;
    bestnava_.diffAge = 0.0f;
    bestnava_.solAge = 0.0f;
    bestnava_.numTracked = 0;
    bestnava_.numUsed = 0;
    bestnava_.numL1 = 0;
    bestnava_.numMulti = 0;
    bestnava_.hAcc = 0;
    bestnava_.vAcc = 0;
    bestnava_.epochTime = 0;

    uniheadinga_.cpuIdle = 0;
    uniheadinga_.gpsWeek = 0;
    uniheadinga_.gpsSeconds = 0;
    uniheadinga_.baseline = 0;
    uniheadinga_.heading = 0;
    uniheadinga_.pitch = 0;
    uniheadinga_.headingSigma = 0;
    uniheadinga_.pitchSigma = 0;
    uniheadinga_.numTracked = 0;
    uniheadinga_.numUsed = 0;
    uniheadinga_.numAboveMask = 0;
    uniheadinga_.numL2AboveMask = 0;
    uniheadinga_.solSource = 0;
    uniheadinga_.headingValid = false;
    uniheadinga_.headingAcc = 0;
    uniheadinga_.epochTime = 0;

    bestvela_.gpsWeek = 0;
    bestvela_.gpsSeconds = 0;
    bestvela_.latency = 0;
    bestvela_.age = 0;
    bestvela_.horSpd = 0;
    bestvela_.trkGnd = 0;
    bestvela_.vertSpd = 0;
    bestvela_.velN = 0;
    bestvela_.velE = 0;
    bestvela_.epochTime = 0;

    psrdopa_.gpsWeek = 0;
    psrdopa_.gpsSeconds = 0;
    psrdopa_.gdop = 99.9f;
    psrdopa_.pdop = 99.9f;
    psrdopa_.hdop = 99.9f;
    psrdopa_.htdop = 99.9f;
    psrdopa_.tdop = 99.9f;
    psrdopa_.vdop = 99.9f;
    psrdopa_.epochTime = 0;

    stadopa_.gpsWeek = 0;
    stadopa_.gpsSeconds = 0;
    stadopa_.gdop = 99.9f;
    stadopa_.pdop = 99.9f;
    stadopa_.hdop = 99.9f;
    stadopa_.htdop = 99.9f;
    stadopa_.tdop = 99.9f;
    stadopa_.vdop = 99.9f;
    stadopa_.epochTime = 0;

    stadopha_.gpsWeek = 0;
    stadopha_.gpsSeconds = 0;
    stadopha_.gdop = 99.9f;
    stadopha_.pdop = 99.9f;
    stadopha_.hdop = 99.9f;
    stadopha_.htdop = 99.9f;
    stadopha_.tdop = 99.9f;
    stadopha_.vdop = 99.9f;
    stadopha_.epochTime = 0;

    rtkstatusa_.gpsWeek = 0;
    rtkstatusa_.gpsSeconds = 0;
    rtkstatusa_.calcStatus = 0;
    rtkstatusa_.ionDetected = 0;
    rtkstatusa_.dualRtkFlag = 0xFF;
    rtkstatusa_.adrNumber = 0;
    rtkstatusa_.corrHealthy = false;
    rtkstatusa_.epochTime = 0;
}

UnicoreParser::~UnicoreParser() {
    disconnect();
}

bool UnicoreParser::connect() {
    serial_fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd_ == -1) {
        std::cerr << "Error opening serial port: " << port_ << std::endl;
        return false;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    if (tcgetattr(serial_fd_, &tty) != 0) {
        std::cerr << "Error from tcgetattr" << std::endl;
        close(serial_fd_);
        serial_fd_ = -1;
        return false;
    }

    // Set baud rate
#ifdef __APPLE__
    // macOS: use B230400 as placeholder, then set actual rate via iossiospeed
    cfsetispeed(&tty, B230400);
    cfsetospeed(&tty, B230400);
#else
    speed_t baud;
    switch (baudrate_) {
        case 9600: baud = B9600; break;
        case 19200: baud = B19200; break;
        case 38400: baud = B38400; break;
        case 57600: baud = B57600; break;
        case 115200: baud = B115200; break;
        case 230400: baud = B230400; break;
        case 460800: baud = B460800; break;
        case 921600: baud = B921600; break;
        default:
            std::cerr << "Unsupported baud rate: " << baudrate_ << std::endl;
            close(serial_fd_);
            serial_fd_ = -1;
            return false;
    }
    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);
#endif

    // 8N1, no flow control
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;

    // Raw input
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    // Raw output
    tty.c_oflag &= ~OPOST;

    // No software flow control
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    // Non-blocking read
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
        std::cerr << "Error from tcsetattr" << std::endl;
        close(serial_fd_);
        serial_fd_ = -1;
        return false;
    }

    tcflush(serial_fd_, TCIOFLUSH);

#ifdef __APPLE__
    // macOS: set the actual baud rate via iossiospeed ioctl
    speed_t speed = static_cast<speed_t>(baudrate_);
    if (ioctl(serial_fd_, IOSSIOSPEED, &speed) == -1) {
        std::cerr << "Error setting baud rate " << baudrate_ << " via iossiospeed" << std::endl;
        close(serial_fd_);
        serial_fd_ = -1;
        return false;
    }
#endif

    std::cout << "Connected to Unicore on " << port_ << " at " << baudrate_ << " baud" << std::endl;
    return true;
}

void UnicoreParser::disconnect() {
    if (serial_fd_ >= 0) {
        close(serial_fd_);
        serial_fd_ = -1;
        std::cout << "Disconnected from Unicore" << std::endl;
    }
}

std::map<UnicoreMessageType, bool> UnicoreParser::update() {
    // Reset new data flags
    for (auto& pair : newData_) {
        pair.second = false;
    }

    if (serial_fd_ < 0) {
        return newData_;
    }

    // Read available data
    char buffer[1024];
    ssize_t bytes_read = read(serial_fd_, buffer, sizeof(buffer) - 1);

    if (bytes_read > 0) {
        // Call raw data callback if set (for binary/ASCII logging)
        if (rawDataCallback_) {
            rawDataCallback_(reinterpret_cast<const uint8_t*>(buffer), static_cast<size_t>(bytes_read));
        }

        buffer[bytes_read] = '\0';

        // Append to line buffer and process complete lines
        lineBuffer_ += buffer;

        size_t pos;
        while ((pos = lineBuffer_.find("\r\n")) != std::string::npos) {
            std::string line = lineBuffer_.substr(0, pos);
            lineBuffer_.erase(0, pos + 2);

            if (!line.empty()) {
                processLine(line);
            }
        }

        // Prevent buffer from growing too large
        if (lineBuffer_.size() > 4096) {
            lineBuffer_.clear();
        }
    }

    return newData_;
}

bool UnicoreParser::writeRawData(const uint8_t* data, size_t len) {
    if (serial_fd_ < 0 || data == nullptr || len == 0) {
        return false;
    }

    size_t total_written = 0;
    while (total_written < len) {
        ssize_t n = write(serial_fd_, data + total_written, len - total_written);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // Non-blocking write would block, brief pause and retry
                usleep(1000);
                continue;
            }
            std::cerr << "Error writing to serial port: " << strerror(errno) << std::endl;
            return false;
        }
        total_written += static_cast<size_t>(n);
    }
    return true;
}

bool UnicoreParser::processLine(const std::string& line) {
    if (line.empty() || line[0] != '#') {
        return false;
    }

    // Verify checksum before parsing
    if (!verifyChecksum(line)) {
        return false;
    }

    if (line.compare(0, 9, "#BESTNAVA") == 0) {
        return parseBestNavA(line);
    } else if (line.compare(0, 12, "#UNIHEADINGA") == 0) {
        return parseUniHeadingA(line);
    } else if (line.compare(0, 9, "#BESTVELA") == 0) {
        return parseBestVelA(line);
    } else if (line.compare(0, 9, "#STADOPHA") == 0) {
        return parseStaDophA(line);
    } else if (line.compare(0, 8, "#STADOPA") == 0) {
        return parseStaDopA(line);
    } else if (line.compare(0, 8, "#PSRDOPA") == 0) {
        return parsePsrDopA(line);
    } else if (line.compare(0, 11, "#RTKSTATUSA") == 0) {
        return parseRtkStatusA(line);
    }

    return false;
}

bool UnicoreParser::parseBestNavA(const std::string& line) {
    // Format: #BESTNAVA,port,0,cpuIdle,timeStatus,gpsWeek,gpsSeconds,...;body*checksum
    size_t semicolon = line.find(';');
    if (semicolon == std::string::npos) {
        return false;
    }

    std::string header = line.substr(1, semicolon - 1); // Skip '#'
    std::string body = line.substr(semicolon + 1);

    // Remove checksum from body
    size_t asterisk = body.find('*');
    if (asterisk != std::string::npos) {
        body = body.substr(0, asterisk);
    }

    // Parse header
    std::vector<std::string> headerFields = split(header, ',');
    if (headerFields.size() < 7) {
        return false;
    }

    try {
        bestnava_.port = headerFields[1];
        // Field [2] is message sequence/type (e.g., "GPS")
        // Field [3] is time status (e.g., "FINE")
        bestnava_.timeStatus = headerFields[3];
        bestnava_.gpsWeek = static_cast<uint16_t>(std::stoi(headerFields[4]));
        bestnava_.gpsSeconds = std::stod(headerFields[5]) / 1000.0; // Convert ms to seconds
        bestnava_.cpuIdle = 0; // Not provided in this format

        // Parse body
        std::vector<std::string> bodyFields = split(body, ',');
        if (bodyFields.size() < 15) {
            return false;
        }

        bestnava_.solStatus = bodyFields[0];
        bestnava_.posType = bodyFields[1];
        bestnava_.lat = std::stod(bodyFields[2]);
        bestnava_.lon = std::stod(bodyFields[3]);
        bestnava_.height = std::stod(bodyFields[4]);
        bestnava_.undulation = std::stod(bodyFields[5]);
        bestnava_.datumId = bodyFields[6];
        bestnava_.latSigma = std::stod(bodyFields[7]);
        bestnava_.lonSigma = std::stod(bodyFields[8]);
        bestnava_.hgtSigma = std::stod(bodyFields[9]);
        // Strip surrounding quotes from base station ID (raw data may have "")
        bestnava_.baseId = bodyFields[10];
        if (bestnava_.baseId.size() >= 2 &&
            bestnava_.baseId.front() == '"' && bestnava_.baseId.back() == '"') {
            bestnava_.baseId = bestnava_.baseId.substr(1, bestnava_.baseId.size() - 2);
        }
        bestnava_.diffAge = std::stof(bodyFields[11]);
        bestnava_.solAge = std::stof(bodyFields[12]);
        bestnava_.numTracked = static_cast<uint8_t>(std::stoi(bodyFields[13]));
        bestnava_.numUsed = static_cast<uint8_t>(std::stoi(bodyFields[14]));

        if (bodyFields.size() > 15) {
            bestnava_.numL1 = static_cast<uint8_t>(std::stoi(bodyFields[15]));
        }
        if (bodyFields.size() > 16) {
            bestnava_.numMulti = static_cast<uint8_t>(std::stoi(bodyFields[16]));
        }

        // Compute derived values
        bestnava_.hAcc = std::sqrt(bestnava_.latSigma * bestnava_.latSigma +
                                   bestnava_.lonSigma * bestnava_.lonSigma);
        bestnava_.vAcc = bestnava_.hgtSigma;
        bestnava_.fixType = mapPosTypeToFix(bestnava_.posType);
        bestnava_.epochTime = gpsToEpoch(bestnava_.gpsWeek, bestnava_.gpsSeconds);

        newData_[UnicoreMessageType::BESTNAVA] = true;
        messageCounts_[UnicoreMessageType::BESTNAVA]++;
        lastUpdateTime_[UnicoreMessageType::BESTNAVA] = getCurrentTime();

        return true;
    } catch (const std::exception& e) {
        std::cerr << "Error parsing BESTNAVA: " << e.what() << std::endl;
        std::cerr << "  Raw line: " << line << std::endl;
        std::cerr << "  Header fields: " << headerFields.size() << std::endl;
        for (size_t i = 0; i < std::min(headerFields.size(), size_t(10)); i++) {
            std::cerr << "    [" << i << "] = '" << headerFields[i] << "'" << std::endl;
        }
        return false;
    }
}

bool UnicoreParser::parseUniHeadingA(const std::string& line) {
    // Format: #UNIHEADINGA,port,0,cpuIdle,timeStatus,gpsWeek,gpsSeconds,...;body*checksum
    size_t semicolon = line.find(';');
    if (semicolon == std::string::npos) {
        return false;
    }

    std::string header = line.substr(1, semicolon - 1);
    std::string body = line.substr(semicolon + 1);

    size_t asterisk = body.find('*');
    if (asterisk != std::string::npos) {
        body = body.substr(0, asterisk);
    }

    std::vector<std::string> headerFields = split(header, ',');
    if (headerFields.size() < 7) {
        return false;
    }

    try {
        uniheadinga_.port = headerFields[1];
        // Field [2] is message sequence/type (e.g., "GPS")
        // Field [3] is time status (e.g., "FINE")
        uniheadinga_.timeStatus = headerFields[3];
        uniheadinga_.gpsWeek = static_cast<uint16_t>(std::stoi(headerFields[4]));
        uniheadinga_.gpsSeconds = std::stod(headerFields[5]) / 1000.0; // Convert ms to seconds
        uniheadinga_.cpuIdle = 0; // Not provided in this format

        std::vector<std::string> bodyFields = split(body, ',');
        if (bodyFields.size() < 8) {
            return false;
        }

        uniheadinga_.solStatus = bodyFields[0];
        uniheadinga_.posType = bodyFields[1];
        uniheadinga_.baseline = std::stod(bodyFields[2]);
        uniheadinga_.heading = std::stod(bodyFields[3]);
        uniheadinga_.pitch = std::stod(bodyFields[4]);
        uniheadinga_.reserved = bodyFields[5];
        uniheadinga_.headingSigma = std::stod(bodyFields[6]);
        uniheadinga_.pitchSigma = std::stod(bodyFields[7]);

        if (bodyFields.size() > 8) {
            uniheadinga_.stationId = bodyFields[8];
        }
        if (bodyFields.size() > 9) {
            uniheadinga_.numTracked = static_cast<uint8_t>(std::stoi(bodyFields[9]));
        }
        if (bodyFields.size() > 10) {
            uniheadinga_.numUsed = static_cast<uint8_t>(std::stoi(bodyFields[10]));
        }
        if (bodyFields.size() > 11) {
            uniheadinga_.numAboveMask = static_cast<uint8_t>(std::stoi(bodyFields[11]));
        }
        if (bodyFields.size() > 12) {
            uniheadinga_.numL2AboveMask = static_cast<uint8_t>(std::stoi(bodyFields[12]));
        }
        if (bodyFields.size() > 13) {
            uniheadinga_.solSource = static_cast<uint8_t>(std::stoi(bodyFields[13]));
        }

        // Compute derived values
        uniheadinga_.headingValid = (uniheadinga_.solStatus == "SOL_COMPUTED") &&
                                    (uniheadinga_.posType == "NARROW_INT" ||
                                     uniheadinga_.posType == "WIDE_INT" ||
                                     uniheadinga_.posType == "NARROW_FLOAT" ||
                                     uniheadinga_.posType == "WIDE_FLOAT");
        uniheadinga_.headingAcc = uniheadinga_.headingSigma;
        uniheadinga_.carrSoln = uniheadinga_.posType;
        uniheadinga_.epochTime = gpsToEpoch(uniheadinga_.gpsWeek, uniheadinga_.gpsSeconds);

        newData_[UnicoreMessageType::UNIHEADINGA] = true;
        messageCounts_[UnicoreMessageType::UNIHEADINGA]++;
        lastUpdateTime_[UnicoreMessageType::UNIHEADINGA] = getCurrentTime();

        return true;
    } catch (const std::exception& e) {
        std::cerr << "Error parsing UNIHEADINGA: " << e.what() << std::endl;
        std::cerr << "  Raw line: " << line << std::endl;
        std::cerr << "  Header fields: " << headerFields.size() << std::endl;
        for (size_t i = 0; i < std::min(headerFields.size(), size_t(10)); i++) {
            std::cerr << "    [" << i << "] = '" << headerFields[i] << "'" << std::endl;
        }
        return false;
    }
}

bool UnicoreParser::parseBestVelA(const std::string& line) {
    // Format: #BESTVELA,port,seq,cpuIdle,timeStatus,gpsWeek,gpsSeconds,...;sol_status,vel_type,latency,age,hor_spd,trk_gnd,vert_spd,reserved*checksum
    // Example: #BESTVELA,COM1,11826,79.0,FINE,2398,252632.900,147888,6,18;SOL_COMPUTED,DOPPLER_VELOCITY,0.000,0.000,0.0012,278.598264,-0.0034,0000200003*2c9614a8
    size_t semicolon = line.find(';');
    if (semicolon == std::string::npos) {
        return false;
    }

    std::string header = line.substr(1, semicolon - 1); // Skip '#'
    std::string body = line.substr(semicolon + 1);

    // Remove checksum from body
    size_t asterisk = body.find('*');
    if (asterisk != std::string::npos) {
        body = body.substr(0, asterisk);
    }

    // Parse header
    std::vector<std::string> headerFields = split(header, ',');
    if (headerFields.size() < 7) {
        return false;
    }

    try {
        bestvela_.port = headerFields[1];
        // headerFields[2] is sequence number
        // headerFields[3] is cpuIdle
        bestvela_.timeStatus = headerFields[4];
        bestvela_.gpsWeek = static_cast<uint16_t>(std::stoi(headerFields[5]));
        bestvela_.gpsSeconds = std::stod(headerFields[6]); // Already in seconds

        // Parse body: sol_status,vel_type,latency,age,hor_spd,trk_gnd,vert_spd,reserved
        std::vector<std::string> bodyFields = split(body, ',');
        if (bodyFields.size() < 7) {
            return false;
        }

        bestvela_.solStatus = bodyFields[0];
        bestvela_.velType = bodyFields[1];
        bestvela_.latency = std::stof(bodyFields[2]);
        bestvela_.age = std::stof(bodyFields[3]);
        bestvela_.horSpd = std::stod(bodyFields[4]);
        bestvela_.trkGnd = std::stod(bodyFields[5]);
        bestvela_.vertSpd = std::stod(bodyFields[6]);

        // Compute NED velocity components from horizontal speed and track over ground
        // trkGnd is degrees from True North, clockwise
        double trkRad = bestvela_.trkGnd * M_PI / 180.0;
        bestvela_.velN = bestvela_.horSpd * std::cos(trkRad);
        bestvela_.velE = bestvela_.horSpd * std::sin(trkRad);

        bestvela_.epochTime = gpsToEpoch(bestvela_.gpsWeek, bestvela_.gpsSeconds);

        newData_[UnicoreMessageType::BESTVELA] = true;
        messageCounts_[UnicoreMessageType::BESTVELA]++;
        lastUpdateTime_[UnicoreMessageType::BESTVELA] = getCurrentTime();

        return true;
    } catch (const std::exception& e) {
        std::cerr << "Error parsing BESTVELA: " << e.what() << std::endl;
        std::cerr << "  Raw line: " << line << std::endl;
        return false;
    }
}

bool UnicoreParser::parsePsrDopA(const std::string& line) {
    // Format: #PSRDOPA,port,seq,cpuIdle,timeStatus,gpsWeek,gpsSeconds,...;gdop,pdop,hdop,htdop,tdop,cutoff,#prns,prn1,prn2,...*checksum
    // Example: #PSRDOPA,COM1,11826,79.0,FINE,2398,252632.900,...;1.2,1.0,0.7,1.1,0.5,10.0,15,2,5,...*checksum
    size_t semicolon = line.find(';');
    if (semicolon == std::string::npos) {
        return false;
    }

    std::string header = line.substr(1, semicolon - 1); // Skip '#'
    std::string body = line.substr(semicolon + 1);

    // Remove checksum from body
    size_t asterisk = body.find('*');
    if (asterisk != std::string::npos) {
        body = body.substr(0, asterisk);
    }

    // Parse header
    std::vector<std::string> headerFields = split(header, ',');
    if (headerFields.size() < 7) {
        return false;
    }

    try {
        psrdopa_.port = headerFields[1];
        // headerFields[2] is sequence number
        // headerFields[3] is cpuIdle
        psrdopa_.timeStatus = headerFields[4];
        psrdopa_.gpsWeek = static_cast<uint16_t>(std::stoi(headerFields[5]));
        psrdopa_.gpsSeconds = std::stod(headerFields[6]); // Already in seconds

        // Parse body: gdop,pdop,hdop,htdop,tdop,cutoff,#prns,...
        std::vector<std::string> bodyFields = split(body, ',');
        if (bodyFields.size() < 5) {
            return false;
        }

        psrdopa_.gdop = std::stof(bodyFields[0]);
        psrdopa_.pdop = std::stof(bodyFields[1]);
        psrdopa_.hdop = std::stof(bodyFields[2]);
        psrdopa_.htdop = std::stof(bodyFields[3]);
        psrdopa_.tdop = std::stof(bodyFields[4]);

        // Derive VDOP from PDOP and HDOP: vdop = sqrt(pdop^2 - hdop^2)
        float pdop2 = psrdopa_.pdop * psrdopa_.pdop;
        float hdop2 = psrdopa_.hdop * psrdopa_.hdop;
        if (pdop2 >= hdop2) {
            psrdopa_.vdop = std::sqrt(pdop2 - hdop2);
        } else {
            psrdopa_.vdop = 0.0f; // Edge case: shouldn't happen with valid data
        }

        psrdopa_.epochTime = gpsToEpoch(psrdopa_.gpsWeek, psrdopa_.gpsSeconds);

        newData_[UnicoreMessageType::PSRDOPA] = true;
        messageCounts_[UnicoreMessageType::PSRDOPA]++;
        lastUpdateTime_[UnicoreMessageType::PSRDOPA] = getCurrentTime();

        return true;
    } catch (const std::exception& e) {
        std::cerr << "Error parsing PSRDOPA: " << e.what() << std::endl;
        std::cerr << "  Raw line: " << line << std::endl;
        return false;
    }
}

bool UnicoreParser::parseStaDopA(const std::string& line) {
    // STADOPA has the same body format as PSRDOPA: gdop,pdop,hdop,htdop,tdop,...
    // but reports DOP for the actual BESTNAV solution type (not just pseudorange)
    size_t semicolon = line.find(';');
    if (semicolon == std::string::npos) {
        return false;
    }

    std::string header = line.substr(1, semicolon - 1);
    std::string body = line.substr(semicolon + 1);

    size_t asterisk = body.find('*');
    if (asterisk != std::string::npos) {
        body = body.substr(0, asterisk);
    }

    std::vector<std::string> headerFields = split(header, ',');
    if (headerFields.size() < 7) {
        return false;
    }

    try {
        stadopa_.port = headerFields[1];
        stadopa_.timeStatus = headerFields[4];
        stadopa_.gpsWeek = static_cast<uint16_t>(std::stoi(headerFields[5]));
        stadopa_.gpsSeconds = std::stod(headerFields[6]);

        // STADOPA body format (differs from PSRDOPA — field order is reshuffled
        // and VDOP is explicit). Verified from raw output:
        // body: gps_time,gdop,pdop,tdop,hdop,vdop,htdop,???,cutoff,...
        std::vector<std::string> bodyFields = split(body, ',');
        if (bodyFields.size() < 7) {
            return false;
        }

        stadopa_.gdop = std::stof(bodyFields[1]);
        stadopa_.pdop = std::stof(bodyFields[2]);
        stadopa_.tdop = std::stof(bodyFields[3]);
        stadopa_.hdop = std::stof(bodyFields[4]);
        stadopa_.vdop = std::stof(bodyFields[5]);  // Explicit, not derived
        stadopa_.htdop = std::stof(bodyFields[6]);

        stadopa_.epochTime = gpsToEpoch(stadopa_.gpsWeek, stadopa_.gpsSeconds);

        newData_[UnicoreMessageType::STADOPA] = true;
        messageCounts_[UnicoreMessageType::STADOPA]++;
        lastUpdateTime_[UnicoreMessageType::STADOPA] = getCurrentTime();

        return true;
    } catch (const std::exception& e) {
        std::cerr << "Error parsing STADOPA: " << e.what() << std::endl;
        std::cerr << "  Raw line: " << line << std::endl;
        return false;
    }
}

bool UnicoreParser::parseStaDophA(const std::string& line) {
    // STADOPHA has the same body format as STADOPA/PSRDOPA: gdop,pdop,hdop,htdop,tdop,...
    // but reports DOP for the heading solution (not position)
    size_t semicolon = line.find(';');
    if (semicolon == std::string::npos) {
        return false;
    }

    std::string header = line.substr(1, semicolon - 1);
    std::string body = line.substr(semicolon + 1);

    size_t asterisk = body.find('*');
    if (asterisk != std::string::npos) {
        body = body.substr(0, asterisk);
    }

    std::vector<std::string> headerFields = split(header, ',');
    if (headerFields.size() < 7) {
        return false;
    }

    try {
        stadopha_.port = headerFields[1];
        stadopha_.timeStatus = headerFields[4];
        stadopha_.gpsWeek = static_cast<uint16_t>(std::stoi(headerFields[5]));
        stadopha_.gpsSeconds = std::stod(headerFields[6]);

        // STADOPHA body format (same layout as STADOPA — verified from raw output):
        // body: gps_time,gdop,pdop,tdop,hdop,vdop,htdop,???,cutoff,...
        std::vector<std::string> bodyFields = split(body, ',');
        if (bodyFields.size() < 7) {
            return false;
        }

        stadopha_.gdop = std::stof(bodyFields[1]);
        stadopha_.pdop = std::stof(bodyFields[2]);
        stadopha_.tdop = std::stof(bodyFields[3]);
        stadopha_.hdop = std::stof(bodyFields[4]);
        stadopha_.vdop = std::stof(bodyFields[5]);  // Explicit, not derived
        stadopha_.htdop = std::stof(bodyFields[6]);

        stadopha_.epochTime = gpsToEpoch(stadopha_.gpsWeek, stadopha_.gpsSeconds);

        newData_[UnicoreMessageType::STADOPHA] = true;
        messageCounts_[UnicoreMessageType::STADOPHA]++;
        lastUpdateTime_[UnicoreMessageType::STADOPHA] = getCurrentTime();

        return true;
    } catch (const std::exception& e) {
        std::cerr << "Error parsing STADOPHA: " << e.what() << std::endl;
        std::cerr << "  Raw line: " << line << std::endl;
        return false;
    }
}

bool UnicoreParser::parseRtkStatusA(const std::string& line) {
    // RTKSTATUSA: RTK Solution Status (Unicore Ref Manual section 7.5.84, Table 7-153)
    // Body fields (0-indexed after semicolon):
    //   0: gpsSource(hex), 1: reserved, 2: bdsSource1(hex), 3: bdsSource2(hex),
    //   4: reserved, 5: gloSource(hex), 6: reserved, 7: galSource1(hex),
    //   8: galSource2(hex), 9: QzssSource(hex), 10: reserved,
    //   11: pos_type(enum), 12: calc_status(0-5), 13: ion_detected,
    //   14: dual_rtk_flag, 15: adr_number, 16: reserved
    size_t semicolon = line.find(';');
    if (semicolon == std::string::npos) {
        return false;
    }

    std::string header = line.substr(1, semicolon - 1);
    std::string body = line.substr(semicolon + 1);

    size_t asterisk = body.find('*');
    if (asterisk != std::string::npos) {
        body = body.substr(0, asterisk);
    }

    std::vector<std::string> headerFields = split(header, ',');
    if (headerFields.size() < 7) {
        return false;
    }

    try {
        rtkstatusa_.port = headerFields[1];
        rtkstatusa_.timeStatus = headerFields[4];
        rtkstatusa_.gpsWeek = static_cast<uint16_t>(std::stoi(headerFields[5]));
        rtkstatusa_.gpsSeconds = std::stod(headerFields[6]);

        std::vector<std::string> bodyFields = split(body, ',');
        // Table 7-153: body has 18 fields (indices 0-17)
        // Fields 0-11 are hex bitmasks + reserved, field 12 is pos_type
        if (bodyFields.size() < 13) {
            return false;
        }

        // Field 12 (0-indexed after semicolon = field 13 in table): Position type
        // Note: table uses 1-based IDs where field 1 is header; body fields start at field 2.
        // In the ASCII body after semicolon: index 11 = pos_type, 12 = calc_status, etc.
        rtkstatusa_.posType = bodyFields[11];

        // Field 13 in table = index 12: Calculate status (0-5, 5=RTK solution available)
        rtkstatusa_.calcStatus = 0;
        if (bodyFields.size() > 12) {
            try { rtkstatusa_.calcStatus = static_cast<uint8_t>(std::stoi(bodyFields[12])); } catch (...) {}
        }

        // Field 14 in table = index 13: Ion detected
        rtkstatusa_.ionDetected = 0;
        if (bodyFields.size() > 13) {
            try { rtkstatusa_.ionDetected = static_cast<uint8_t>(std::stoi(bodyFields[13])); } catch (...) {}
        }

        // Field 15 in table = index 14: Dual RTK flag
        rtkstatusa_.dualRtkFlag = 0xFF;
        if (bodyFields.size() > 14) {
            try { rtkstatusa_.dualRtkFlag = static_cast<uint8_t>(std::stoi(bodyFields[14])); } catch (...) {}
        }

        // Field 16 in table = index 15: ADR Number (valid carrier-phase observations)
        rtkstatusa_.adrNumber = 0;
        if (bodyFields.size() > 15) {
            try { rtkstatusa_.adrNumber = static_cast<uint8_t>(std::stoi(bodyFields[15])); } catch (...) {}
        }

        // Derive correction health: healthy if we have an RTK position type
        // and calculate status indicates RTK solution is available (5)
        bool hasRtk = (rtkstatusa_.posType == "NARROW_INT" ||
                       rtkstatusa_.posType == "WIDE_INT" ||
                       rtkstatusa_.posType == "NARROW_FLOAT" ||
                       rtkstatusa_.posType == "WIDE_FLOAT");
        rtkstatusa_.corrHealthy = hasRtk && (rtkstatusa_.calcStatus == 5);

        rtkstatusa_.epochTime = gpsToEpoch(rtkstatusa_.gpsWeek, rtkstatusa_.gpsSeconds);

        newData_[UnicoreMessageType::RTKSTATUSA] = true;
        messageCounts_[UnicoreMessageType::RTKSTATUSA]++;
        lastUpdateTime_[UnicoreMessageType::RTKSTATUSA] = getCurrentTime();

        return true;
    } catch (const std::exception& e) {
        std::cerr << "Error parsing RTKSTATUSA: " << e.what() << std::endl;
        std::cerr << "  Raw line: " << line << std::endl;
        return false;
    }
}

// Unicore/NovAtel CRC-32 lookup table (polynomial 0xEDB88320, reflected)
// init = 0, no final XOR. CRC covers bytes AFTER '#' up to (not including) '*'.
static uint32_t s_crc32_table[256];
static bool s_crc32_table_built = false;

static void buildCRC32Table() {
    if (s_crc32_table_built) return;
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t crc = i;
        for (int j = 0; j < 8; j++) {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xEDB88320u;
            else
                crc >>= 1;
        }
        s_crc32_table[i] = crc;
    }
    s_crc32_table_built = true;
}

static uint32_t computeUnicoreCRC32(const char* data, size_t len) {
    buildCRC32Table();
    uint32_t crc = 0;  // Unicore/NovAtel init = 0
    for (size_t i = 0; i < len; i++) {
        crc = s_crc32_table[(crc ^ static_cast<uint8_t>(data[i])) & 0xFF] ^ (crc >> 8);
    }
    return crc;  // No final XOR
}

bool UnicoreParser::verifyChecksum(const std::string& line) const {
    size_t asterisk = line.rfind('*');
    if (asterisk == std::string::npos || asterisk + 8 > line.size()) {
        return false;
    }
    if (line.empty() || line[0] != '#') {
        return false;
    }

    // CRC covers everything after '#' up to (not including) '*'
    const char* start = line.c_str() + 1;  // skip '#'
    size_t len = asterisk - 1;

    uint32_t computed = computeUnicoreCRC32(start, len);

    // Parse expected checksum from hex string after '*'
    std::string hexStr = line.substr(asterisk + 1, 8);
    uint32_t expected = 0;
    try {
        expected = static_cast<uint32_t>(std::stoul(hexStr, nullptr, 16));
    } catch (...) {
        return false;
    }

    return computed == expected;
}

std::vector<std::string> UnicoreParser::split(const std::string& str, char delim) const {
    std::vector<std::string> tokens;
    std::istringstream iss(str);
    std::string token;
    while (std::getline(iss, token, delim)) {
        tokens.push_back(token);
    }
    return tokens;
}

double UnicoreParser::getCurrentTime() const {
    auto now = std::chrono::steady_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() / 1000.0;
}

std::string UnicoreParser::mapPosTypeToFix(const std::string& posType) const {
    // Complete position/velocity type mapping from Unicore Reference Manual
    // Table 7.6 position_or_velocity_type (15 entries)
    if (posType == "NARROW_INT" || posType == "WIDE_INT" || posType == "L1_INT") {
        return "RTK_FIXED";
    } else if (posType == "NARROW_FLOAT" || posType == "WIDE_FLOAT" || posType == "L1_FLOAT") {
        return "RTK_FLOAT";
    } else if (posType == "SINGLE") {
        return "3D";
    } else if (posType == "PSRDIFF") {
        return "DGPS";
    } else if (posType == "SBAS") {
        return "SBAS";
    } else if (posType == "FIXEDPOS") {
        return "FIXED";
    } else if (posType == "PPP") {
        return "PPP";
    } else if (posType == "PPP_CONVERGING") {
        return "PPP_CONV";
    } else if (posType == "PPP_AR") {
        return "PPP_AR";
    } else if (posType == "PPP_RTK") {
        return "PPP_RTK";
    } else if (posType == "INS") {
        return "INS";
    } else if (posType == "DOPPLER_VELOCITY") {
        return "DOPPLER";
    } else if (posType == "NONE") {
        return "NONE";
    }
    return posType;
}

double UnicoreParser::gpsToEpoch(uint16_t week, double seconds) const {
    // Convert GPS week + seconds to Unix epoch
    return GPS_EPOCH_UNIX + (week * SECONDS_PER_WEEK) + seconds - GPS_LEAP_SECONDS;
}

double UnicoreParser::getTimeSinceUpdate(UnicoreMessageType type) const {
    auto it = lastUpdateTime_.find(type);
    if (it == lastUpdateTime_.end() || it->second == 0) {
        return -1;
    }
    return getCurrentTime() - it->second;
}

int UnicoreParser::getMessageCount(UnicoreMessageType type) const {
    auto it = messageCounts_.find(type);
    if (it == messageCounts_.end()) {
        return 0;
    }
    return it->second;
}

bool UnicoreParser::hasFix() const {
    if (messageCounts_.at(UnicoreMessageType::BESTNAVA) == 0)
        return false;
    // All position types that represent a valid position fix
    // (from Unicore Reference Manual Table 7.6)
    const std::string& pt = bestnava_.posType;
    return (pt == "SINGLE" ||
            pt == "NARROW_INT" || pt == "WIDE_INT" || pt == "L1_INT" ||
            pt == "NARROW_FLOAT" || pt == "WIDE_FLOAT" || pt == "L1_FLOAT" ||
            pt == "PSRDIFF" || pt == "SBAS" ||
            pt == "FIXEDPOS" || pt == "INS" ||
            pt == "PPP" || pt == "PPP_CONVERGING" ||
            pt == "PPP_AR" || pt == "PPP_RTK");
}

bool UnicoreParser::hasHeading() const {
    return messageCounts_.at(UnicoreMessageType::UNIHEADINGA) > 0 &&
           uniheadinga_.headingValid;
}
