#pragma once

#include <string>
#include <vector>
#include <map>
#include <chrono>
#include <ctime>
#include <functional>
#include <cmath>
#include <sstream>
#include <iomanip>

// For serial communication
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

enum class UnicoreMessageType {
    BESTNAVA,
    UNIHEADINGA,
    BESTVELA,
    PSRDOPA,
    STADOPA,
    STADOPHA,
    RTKSTATUSA,
    UNKNOWN
};

// Callback type for raw data capture
using UnicoreRawDataCallback = std::function<void(const uint8_t*, size_t)>;

class UnicoreParser {
public:
    // Data structures for parsed messages
    struct BestNavAData {
        // Header fields
        std::string port;
        double cpuIdle;
        std::string timeStatus;
        uint16_t gpsWeek;
        double gpsSeconds;

        // Body fields
        std::string solStatus;     // SOL_COMPUTED, etc.
        std::string posType;       // NARROW_INT, SINGLE, etc.
        double lat;                // degrees
        double lon;                // degrees
        double height;             // meters (above ellipsoid)
        double undulation;         // meters
        std::string datumId;
        double latSigma;           // meters
        double lonSigma;           // meters
        double hgtSigma;           // meters
        std::string baseId;
        float diffAge;             // seconds
        float solAge;              // seconds
        uint8_t numTracked;
        uint8_t numUsed;
        uint8_t numL1;
        uint8_t numMulti;

        // Derived values
        double hAcc;               // horizontal accuracy (sqrt of lat^2+lon^2 sigmas)
        double vAcc;               // vertical accuracy
        std::string fixType;       // mapped fix type (RTK_FIXED, 3D, etc.)
        double epochTime;          // Unix epoch timestamp
    };

    struct UniHeadingAData {
        // Header fields
        std::string port;
        double cpuIdle;
        std::string timeStatus;
        uint16_t gpsWeek;
        double gpsSeconds;

        // Body fields
        std::string solStatus;
        std::string posType;
        double baseline;           // meters
        double heading;            // degrees (0-360)
        double pitch;              // degrees
        std::string reserved;
        double headingSigma;       // degrees
        double pitchSigma;         // degrees
        std::string stationId;
        uint8_t numTracked;
        uint8_t numUsed;
        uint8_t numAboveMask;
        uint8_t numL2AboveMask;
        uint8_t solSource;

        // Derived values
        bool headingValid;
        double headingAcc;
        std::string carrSoln;
        double epochTime;
    };

    struct BestVelAData {
        // Header fields
        std::string port;
        std::string timeStatus;
        uint16_t gpsWeek;
        double gpsSeconds;

        // Body fields
        std::string solStatus;     // SOL_COMPUTED, etc.
        std::string velType;       // DOPPLER_VELOCITY, PPP, etc.
        float latency;             // seconds - subtract from time for true velocity time
        float age;                 // differential age (seconds)
        double horSpd;             // horizontal speed over ground (m/s)
        double trkGnd;             // track over ground / course (degrees, True North ref)
        double vertSpd;            // vertical speed (m/s, + = up)

        // Derived values
        double velN;               // North velocity component (m/s)
        double velE;               // East velocity component (m/s)
        double epochTime;          // Unix epoch timestamp
    };

    struct PsrDopAData {
        // Header fields
        std::string port;
        std::string timeStatus;
        uint16_t gpsWeek;
        double gpsSeconds;

        // Body fields
        float gdop;                // Geometric DOP
        float pdop;                // Position DOP
        float hdop;                // Horizontal DOP
        float htdop;               // Horizontal + Time DOP
        float tdop;                // Time DOP

        // Derived values
        float vdop;                // Vertical DOP = sqrt(pdop^2 - hdop^2)
        double epochTime;          // Unix epoch timestamp
    };

    // STADOPA: DOP of the BESTNAV solution (tracks actual solution type)
    struct StaDopAData {
        // Header fields
        std::string port;
        std::string timeStatus;
        uint16_t gpsWeek;
        double gpsSeconds;

        // Body fields (same DOP fields as PSRDOPA)
        float gdop;                // Geometric DOP
        float pdop;                // Position DOP
        float hdop;                // Horizontal DOP
        float htdop;               // Horizontal + Time DOP
        float tdop;                // Time DOP

        // Derived values
        float vdop;                // Vertical DOP = sqrt(pdop^2 - hdop^2)
        double epochTime;          // Unix epoch timestamp
    };

    // STADOPHA: DOP of the heading solution (tracks actual heading solution type)
    struct StaDophAData {
        // Header fields
        std::string port;
        std::string timeStatus;
        uint16_t gpsWeek;
        double gpsSeconds;

        // Body fields (same DOP fields as PSRDOPA/STADOPA)
        float gdop;                // Geometric DOP
        float pdop;                // Position DOP
        float hdop;                // Horizontal DOP
        float htdop;               // Horizontal + Time DOP
        float tdop;                // Time DOP

        // Derived values
        float vdop;                // Vertical DOP = sqrt(pdop^2 - hdop^2)
        double epochTime;          // Unix epoch timestamp
    };

    // RTKSTATUSA: RTK solution status (section 7.5.84 of Unicore Reference Manual)
    struct RtkStatusAData {
        // Header fields
        std::string port;
        std::string timeStatus;
        uint16_t gpsWeek;
        double gpsSeconds;

        // Body fields (per Table 7-153)
        std::string posType;       // Field 13: Position type (NARROW_INT, NONE, etc.)
        uint8_t calcStatus;        // Field 14: Calculate status (0-5, 5=RTK solution available)
        uint8_t ionDetected;       // Field 15: Ionospheric scintillation (0=none, 1-255)
        uint8_t dualRtkFlag;       // Field 16: Dual-antenna baseline flag (0xFF=not configured)
        uint8_t adrNumber;         // Field 17: Valid carrier-phase observation count

        // Derived values
        bool corrHealthy;          // Is RTK solution healthy (has RTK pos type and calcStatus==5)
        double epochTime;          // Unix epoch timestamp
    };

public:
    UnicoreParser(const std::string& port = "/dev/ttyUSB0", int baudrate = 460800);
    ~UnicoreParser();

    bool connect();
    void disconnect();
    bool isConnected() const { return serial_fd_ >= 0; }

    std::map<UnicoreMessageType, bool> update();

    // Getters for parsed data
    const BestNavAData& getBestNavA() const { return bestnava_; }
    const UniHeadingAData& getUniHeadingA() const { return uniheadinga_; }
    const BestVelAData& getBestVelA() const { return bestvela_; }
    const PsrDopAData& getPsrDopA() const { return psrdopa_; }
    const StaDopAData& getStaDopA() const { return stadopa_; }
    const StaDophAData& getStaDophA() const { return stadopha_; }
    const RtkStatusAData& getRtkStatusA() const { return rtkstatusa_; }

    // Convenience getters
    double getLatitude() const { return bestnava_.lat; }
    double getLongitude() const { return bestnava_.lon; }
    double getAltitude() const { return bestnava_.height; }
    double getHorizontalAccuracy() const { return bestnava_.hAcc; }
    double getVerticalAccuracy() const { return bestnava_.vAcc; }
    std::string getFixType() const { return bestnava_.fixType; }
    uint8_t getNumSatellites() const { return bestnava_.numUsed; }

    double getHeading() const { return uniheadinga_.heading; }
    double getBaseline() const { return uniheadinga_.baseline; }
    double getHeadingAccuracy() const { return uniheadinga_.headingAcc; }
    bool isHeadingValid() const { return uniheadinga_.headingValid; }
    double getPitch() const { return uniheadinga_.pitch; }
    std::string getCarrierSolution() const { return uniheadinga_.carrSoln; }

    // Velocity getters (from BESTVELA)
    double getVelNorth() const { return bestvela_.velN; }
    double getVelEast() const { return bestvela_.velE; }
    double getVelDown() const { return -bestvela_.vertSpd; }  // Convert up to down
    double getGroundSpeed() const { return bestvela_.horSpd; }
    double getTrackOverGround() const { return bestvela_.trkGnd; }

    // DOP getters (prefer STADOPA if available, fall back to PSRDOPA)
    float getHDOP() const { return (messageCounts_.count(UnicoreMessageType::STADOPA) && messageCounts_.at(UnicoreMessageType::STADOPA) > 0) ? stadopa_.hdop : psrdopa_.hdop; }
    float getVDOP() const { return (messageCounts_.count(UnicoreMessageType::STADOPA) && messageCounts_.at(UnicoreMessageType::STADOPA) > 0) ? stadopa_.vdop : psrdopa_.vdop; }
    float getPDOP() const { return (messageCounts_.count(UnicoreMessageType::STADOPA) && messageCounts_.at(UnicoreMessageType::STADOPA) > 0) ? stadopa_.pdop : psrdopa_.pdop; }
    float getGDOP() const { return (messageCounts_.count(UnicoreMessageType::STADOPA) && messageCounts_.at(UnicoreMessageType::STADOPA) > 0) ? stadopa_.gdop : psrdopa_.gdop; }

    // Heading DOP getters (from STADOPHA)
    float getHeadingHDOP() const { return stadopha_.hdop; }
    float getHeadingVDOP() const { return stadopha_.vdop; }
    float getHeadingPDOP() const { return stadopha_.pdop; }
    float getHeadingGDOP() const { return stadopha_.gdop; }

    // RTK status getters
    bool isCorrectionHealthy() const { return rtkstatusa_.corrHealthy; }
    uint8_t getRtkCalcStatus() const { return rtkstatusa_.calcStatus; }
    uint8_t getRtkAdrNumber() const { return rtkstatusa_.adrNumber; }

    // Message tracking
    double getTimeSinceUpdate(UnicoreMessageType type) const;
    int getMessageCount(UnicoreMessageType type) const;

    bool hasFix() const;
    bool hasHeading() const;

    // Write raw data to the serial port (e.g. RTCM corrections)
    bool writeRawData(const uint8_t* data, size_t len);

    // Raw data capture callback (for binary logging)
    void setRawDataCallback(UnicoreRawDataCallback callback) { rawDataCallback_ = callback; }

private:
    // Serial connection
    std::string port_;
    int baudrate_;
    int serial_fd_;

    // Line buffer for ASCII parsing
    std::string lineBuffer_;

    // Parsed data
    BestNavAData bestnava_;
    UniHeadingAData uniheadinga_;
    BestVelAData bestvela_;
    PsrDopAData psrdopa_;
    StaDopAData stadopa_;
    StaDophAData stadopha_;
    RtkStatusAData rtkstatusa_;

    // Message tracking
    std::map<UnicoreMessageType, bool> newData_;
    std::map<UnicoreMessageType, int> messageCounts_;
    std::map<UnicoreMessageType, double> lastUpdateTime_;

    // Parsing methods
    bool processLine(const std::string& line);
    bool parseBestNavA(const std::string& line);
    bool parseUniHeadingA(const std::string& line);
    bool parseBestVelA(const std::string& line);
    bool parsePsrDopA(const std::string& line);
    bool parseStaDopA(const std::string& line);
    bool parseStaDophA(const std::string& line);
    bool parseRtkStatusA(const std::string& line);

    // Helper methods
    std::vector<std::string> split(const std::string& str, char delim) const;
    double getCurrentTime() const;
    std::string mapPosTypeToFix(const std::string& posType) const;
    double gpsToEpoch(uint16_t week, double seconds) const;
    bool verifyChecksum(const std::string& line) const;

    // Raw data callback
    UnicoreRawDataCallback rawDataCallback_;
};
