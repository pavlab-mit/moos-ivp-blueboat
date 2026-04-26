/*************************************************************
      Name: Jeremy Wenger
      Orgn: MIT, Cambridge MA
      File: lib_unicore/UnicoreDataDTO.hpp
   Last Ed: 2026-03-04
     Brief:
        Data Transfer Objects (DTOs) for Unicore UM982 GNSS data
        with serialization methods for MOOS communication, CSV
        logging, and human-readable output.
*************************************************************/

#pragma once

#include <string>
#include <sstream>
#include <iomanip>
#include <cstring>
#include <cmath>
#include <chrono>

namespace unicore_dto {

inline double getTimeSinceEpoch() {
    auto now = std::chrono::system_clock::now();
    auto dur = now.time_since_epoch();
    return std::chrono::duration<double>(dur).count();
}

} // namespace unicore_dto

// Position data from BESTNAVA message
class UnicorePositionDTO {
public:
    double timestamp_epoch;
    double latitude;           // degrees
    double longitude;          // degrees
    double altitude;           // meters (above ellipsoid)
    double undulation;         // meters (geoid separation)
    double h_acc;              // horizontal accuracy (m)
    double v_acc;              // vertical accuracy (m)
    double lat_sigma;          // latitude sigma (m)
    double lon_sigma;          // longitude sigma (m)
    double hgt_sigma;          // height sigma (m)
    std::string fix_type;      // RTK_FIXED, RTK_FLOAT, 3D, SINGLE, NONE
    std::string sol_status;    // SOL_COMPUTED, etc.
    uint8_t num_tracked;
    uint8_t num_used;
    double epoch_time;         // GPS-derived epoch time

    UnicorePositionDTO() {
        timestamp_epoch = unicore_dto::getTimeSinceEpoch();
        latitude = 0.0;
        longitude = 0.0;
        altitude = 0.0;
        undulation = 0.0;
        h_acc = 0.0;
        v_acc = 0.0;
        lat_sigma = 0.0;
        lon_sigma = 0.0;
        hgt_sigma = 0.0;
        fix_type = "NONE";
        sol_status = "NONE";
        num_tracked = 0;
        num_used = 0;
        epoch_time = 0.0;
    }

    std::string repr() const {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(7);
        ss << "UnicorePosition: ";
        ss << "Fix=" << fix_type << ", ";
        ss << "Sol=" << sol_status << ", ";
        ss << "Sats=" << static_cast<int>(num_used) << "/" << static_cast<int>(num_tracked) << ", ";
        ss << "Pos=(" << latitude << ", " << longitude << ", " << altitude << "m), ";
        ss << "Acc=(H:" << h_acc << "m, V:" << v_acc << "m)";
        return ss.str();
    }

    std::string as_csv() const {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(9);
        ss << timestamp_epoch << "," << latitude << "," << longitude << ","
           << altitude << "," << undulation << "," << h_acc << ","
           << v_acc << "," << lat_sigma << "," << lon_sigma << ","
           << hgt_sigma << "," << fix_type << "," << sol_status << ","
           << static_cast<int>(num_tracked) << "," << static_cast<int>(num_used) << ","
           << epoch_time;
        return ss.str();
    }

    static std::string get_headers() {
        return "timestamp_epoch,latitude,longitude,altitude,undulation,h_acc,"
               "v_acc,lat_sigma,lon_sigma,hgt_sigma,fix_type,sol_status,"
               "num_tracked,num_used,epoch_time";
    }

    std::string to_moos_string() const {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(9);
        ss << "LAT=" << latitude
           << ",LON=" << longitude
           << ",ALT=" << std::setprecision(3) << altitude
           << ",HACC=" << h_acc
           << ",VACC=" << v_acc
           << ",FIX=" << fix_type
           << ",SOL=" << sol_status
           << ",SAT=" << static_cast<int>(num_used)
           << ",TS=" << std::setprecision(3) << timestamp_epoch;
        return ss.str();
    }
};

// Dual-antenna heading data from UNIHEADINGA message
class UnicoreHeadingDTO {
public:
    double timestamp_epoch;
    double heading;            // degrees (0-360)
    double pitch;              // degrees
    double baseline;           // meters
    double heading_sigma;      // degrees
    double pitch_sigma;        // degrees
    std::string carr_soln;     // NARROW_INT, NARROW_FLOAT, NONE, etc.
    bool heading_valid;
    std::string sol_status;
    uint8_t num_tracked;
    uint8_t num_used;
    double epoch_time;

    UnicoreHeadingDTO() {
        timestamp_epoch = unicore_dto::getTimeSinceEpoch();
        heading = 0.0;
        pitch = 0.0;
        baseline = 0.0;
        heading_sigma = 0.0;
        pitch_sigma = 0.0;
        carr_soln = "NONE";
        heading_valid = false;
        sol_status = "NONE";
        num_tracked = 0;
        num_used = 0;
        epoch_time = 0.0;
    }

    std::string repr() const {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(3);
        ss << "UnicoreHeading: ";
        ss << "Hdg=" << heading << "deg, ";
        ss << "Pitch=" << pitch << "deg, ";
        ss << "BL=" << std::setprecision(4) << baseline << "m, ";
        ss << "Acc=" << heading_sigma << "deg, ";
        ss << "Carr=" << carr_soln << ", ";
        ss << "Valid=" << (heading_valid ? "Y" : "N");
        return ss.str();
    }

    std::string as_csv() const {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(6);
        ss << timestamp_epoch << "," << heading << "," << pitch << ","
           << baseline << "," << heading_sigma << "," << pitch_sigma << ","
           << carr_soln << "," << (heading_valid ? "1" : "0") << ","
           << sol_status << "," << static_cast<int>(num_tracked) << ","
           << static_cast<int>(num_used) << "," << epoch_time;
        return ss.str();
    }

    static std::string get_headers() {
        return "timestamp_epoch,heading,pitch,baseline,heading_sigma,pitch_sigma,"
               "carr_soln,heading_valid,sol_status,num_tracked,num_used,epoch_time";
    }

    std::string to_moos_string() const {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(3);
        ss << "HDG=" << heading
           << ",PITCH=" << pitch
           << ",BL=" << std::setprecision(4) << baseline
           << ",HACC=" << std::setprecision(3) << heading_sigma
           << ",CARR=" << carr_soln
           << ",VALID=" << (heading_valid ? "1" : "0")
           << ",TS=" << timestamp_epoch;
        return ss.str();
    }
};

// Velocity data from BESTVELA message
class UnicoreVelocityDTO {
public:
    double timestamp_epoch;
    double ground_speed;       // m/s
    double track_over_ground;  // degrees (0-360, true north)
    double vert_speed;         // m/s (positive = up)
    double vel_n;              // m/s
    double vel_e;              // m/s
    std::string sol_status;
    float latency;             // seconds
    double epoch_time;

    UnicoreVelocityDTO() {
        timestamp_epoch = unicore_dto::getTimeSinceEpoch();
        ground_speed = 0.0;
        track_over_ground = 0.0;
        vert_speed = 0.0;
        vel_n = 0.0;
        vel_e = 0.0;
        sol_status = "NONE";
        latency = 0.0f;
        epoch_time = 0.0;
    }

    std::string repr() const {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(3);
        ss << "UnicoreVelocity: ";
        ss << "Spd=" << ground_speed << "m/s, ";
        ss << "COG=" << track_over_ground << "deg, ";
        ss << "Vel=(" << vel_n << ", " << vel_e << ", " << -vert_speed << ")m/s";
        return ss.str();
    }

    std::string as_csv() const {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(6);
        ss << timestamp_epoch << "," << ground_speed << "," << track_over_ground << ","
           << vert_speed << "," << vel_n << "," << vel_e << ","
           << sol_status << "," << latency << "," << epoch_time;
        return ss.str();
    }

    static std::string get_headers() {
        return "timestamp_epoch,ground_speed,track_over_ground,vert_speed,"
               "vel_n,vel_e,sol_status,latency,epoch_time";
    }

    std::string to_moos_string() const {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(3);
        ss << "SPD=" << ground_speed
           << ",COG=" << track_over_ground
           << ",VN=" << vel_n
           << ",VE=" << vel_e
           << ",VD=" << -vert_speed
           << ",TS=" << timestamp_epoch;
        return ss.str();
    }
};

// Dilution of Precision from PSRDOPA message
class UnicoreDOPsDTO {
public:
    double timestamp_epoch;
    float gdop;
    float pdop;
    float hdop;
    float vdop;
    float htdop;
    float tdop;
    double epoch_time;

    UnicoreDOPsDTO() {
        timestamp_epoch = unicore_dto::getTimeSinceEpoch();
        gdop = 0.0f;
        pdop = 0.0f;
        hdop = 0.0f;
        vdop = 0.0f;
        htdop = 0.0f;
        tdop = 0.0f;
        epoch_time = 0.0;
    }

    std::string repr() const {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(2);
        ss << "UnicoreDOPs: "
           << "G=" << gdop << ", P=" << pdop
           << ", H=" << hdop << ", V=" << vdop
           << ", T=" << tdop;
        return ss.str();
    }

    std::string as_csv() const {
        std::ostringstream ss;
        ss << timestamp_epoch << "," << gdop << "," << pdop << ","
           << hdop << "," << vdop << "," << htdop << "," << tdop << ","
           << epoch_time;
        return ss.str();
    }

    static std::string get_headers() {
        return "timestamp_epoch,gdop,pdop,hdop,vdop,htdop,tdop,epoch_time";
    }

    std::string to_moos_string() const {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(2);
        ss << "GDOP=" << gdop
           << ",PDOP=" << pdop
           << ",HDOP=" << hdop
           << ",VDOP=" << vdop
           << ",TDOP=" << tdop
           << ",TS=" << std::setprecision(3) << timestamp_epoch;
        return ss.str();
    }
};

// RTK solution status from RTKSTATUSA message (Unicore Ref Manual 7.5.84)
class UnicoreRtkStatusDTO {
public:
    double timestamp_epoch;
    std::string pos_type;      // NARROW_INT, NONE, etc.
    uint8_t calc_status;       // 0-5 (5=RTK solution available)
    uint8_t ion_detected;      // 0=none, 1-255=scintillation severity
    uint8_t dual_rtk_flag;     // 0xFF=not configured, 0=not resolved, 1=in tolerance, 2=out
    uint8_t adr_number;        // valid carrier-phase observation count
    bool corr_healthy;
    double epoch_time;

    UnicoreRtkStatusDTO() {
        timestamp_epoch = unicore_dto::getTimeSinceEpoch();
        pos_type = "NONE";
        calc_status = 0;
        ion_detected = 0;
        dual_rtk_flag = 0xFF;
        adr_number = 0;
        corr_healthy = false;
        epoch_time = 0.0;
    }

    std::string repr() const {
        std::ostringstream ss;
        ss << "UnicoreRtkStatus: ";
        ss << "Type=" << pos_type << ", ";
        ss << "CalcStat=" << static_cast<int>(calc_status) << ", ";
        ss << "ADR=" << static_cast<int>(adr_number) << ", ";
        ss << "Healthy=" << (corr_healthy ? "Y" : "N");
        return ss.str();
    }

    std::string as_csv() const {
        std::ostringstream ss;
        ss << timestamp_epoch << "," << pos_type << ","
           << static_cast<int>(calc_status) << "," << static_cast<int>(ion_detected) << ","
           << static_cast<int>(dual_rtk_flag) << "," << static_cast<int>(adr_number) << ","
           << (corr_healthy ? "1" : "0") << "," << epoch_time;
        return ss.str();
    }

    static std::string get_headers() {
        return "timestamp_epoch,pos_type,calc_status,ion_detected,dual_rtk_flag,"
               "adr_number,corr_healthy,epoch_time";
    }

    std::string to_moos_string() const {
        std::ostringstream ss;
        ss << "TYPE=" << pos_type
           << ",CALC=" << static_cast<int>(calc_status)
           << ",ADR=" << static_cast<int>(adr_number)
           << ",HEALTHY=" << (corr_healthy ? "1" : "0")
           << ",TS=" << std::fixed << std::setprecision(3) << timestamp_epoch;
        return ss.str();
    }
};

// Composite status derived from multiple messages
class UnicoreStatusDTO {
public:
    double timestamp_epoch;
    bool gps_lock;
    std::string fix_type;
    std::string carr_soln;
    uint8_t num_sats;
    bool heading_valid;
    double h_acc;
    double v_acc;
    float hdop;
    float diff_age;        // Differential correction age (seconds, from BESTNAVA)
    float sol_age;         // Solution age (seconds, from BESTNAVA)
    std::string base_id;   // Base station ID (from BESTNAVA)
    double epoch_time;

    UnicoreStatusDTO() {
        timestamp_epoch = unicore_dto::getTimeSinceEpoch();
        gps_lock = false;
        fix_type = "NONE";
        carr_soln = "NONE";
        num_sats = 0;
        heading_valid = false;
        h_acc = 0.0;
        v_acc = 0.0;
        hdop = 0.0f;
        diff_age = 0.0f;
        sol_age = 0.0f;
        base_id = "";
        epoch_time = 0.0;
    }

    std::string repr() const {
        std::ostringstream ss;
        ss << "UnicoreStatus: ";
        ss << "Lock=" << (gps_lock ? "Y" : "N") << ", ";
        ss << "Fix=" << fix_type << ", ";
        ss << "Carr=" << carr_soln << ", ";
        ss << "Sats=" << static_cast<int>(num_sats) << ", ";
        ss << "HdgValid=" << (heading_valid ? "Y" : "N") << ", ";
        ss << std::fixed << std::setprecision(2);
        ss << "Acc=(H:" << h_acc << "m, V:" << v_acc << "m), ";
        ss << std::setprecision(1);
        ss << "DiffAge=" << diff_age << "s, ";
        ss << "Base=" << (base_id.empty() ? "N/A" : base_id);
        return ss.str();
    }

    std::string as_csv() const {
        std::ostringstream ss;
        ss << timestamp_epoch << "," << (gps_lock ? "1" : "0") << ","
           << fix_type << "," << carr_soln << ","
           << static_cast<int>(num_sats) << "," << (heading_valid ? "1" : "0") << ","
           << h_acc << "," << v_acc << "," << hdop << ","
           << diff_age << "," << sol_age << "," << base_id << "," << epoch_time;
        return ss.str();
    }

    static std::string get_headers() {
        return "timestamp_epoch,gps_lock,fix_type,carr_soln,num_sats,heading_valid,"
               "h_acc,v_acc,hdop,diff_age,sol_age,base_id,epoch_time";
    }

    std::string to_moos_string() const {
        std::ostringstream ss;
        ss << "LOCK=" << (gps_lock ? "1" : "0")
           << ",FIX=" << fix_type
           << ",CARR=" << carr_soln
           << ",SAT=" << static_cast<int>(num_sats)
           << ",HDGV=" << (heading_valid ? "1" : "0")
           << ",HDOP=" << std::fixed << std::setprecision(2) << hdop
           << ",DIFF_AGE=" << std::setprecision(1) << diff_age
           << ",SOL_AGE=" << std::setprecision(1) << sol_age
           << ",BASE=" << (base_id.empty() ? "NONE" : base_id)
           << ",TS=" << std::setprecision(3) << timestamp_epoch;
        return ss.str();
    }
};
