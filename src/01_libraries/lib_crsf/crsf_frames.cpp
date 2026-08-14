/*************************************************************
 * CRSF payload decoding implementation
 *
 * See crsf_frames.h for layouts and rationale.
 *
 * Author: Jeremy Wenger
 *************************************************************/

#include "crsf_frames.h"

//---------------------------------------------------------
// crsfDecodeChannels()
//
// Written as a general bit-extraction loop straight from the
// protocol definition ("channel n is the 11 bits starting at bit
// 11*n of the LSB-first bitstream") rather than as a hand-unrolled
// shift chain.
//
// This is deliberate. lib_sbus decodes the same packing with 16
// hardcoded shift expressions; that form is easy to get subtly
// wrong, impossible to check by eye, and has never been tested.
// The loop below can be read against the spec line by line, and
// the unit tests verify it against an independently written
// bit-level reference rather than against lib_sbus.

bool crsfDecodeChannels(const uint8_t* payload, size_t len,
                        CrsfChannels& out)
{
  if (payload == nullptr)
    return false;

  // ELRS 4.0+ appends an arm-status byte. Channel data is
  // identical, so accept both lengths and ignore the extra byte.
  if (len != CRSF_CHANNELS_PAYLOAD_SIZE &&
      len != CRSF_CHANNELS_PAYLOAD_SIZE_ARM)
    return false;

  for (int ch = 0; ch < CRSF_NUM_CHANNELS; ch++) {
    const int start_bit = ch * 11;

    uint16_t value = 0;
    for (int bit = 0; bit < 11; bit++) {
      const int abs_bit  = start_bit + bit;
      const int byte_idx = abs_bit >> 3;    // / 8
      const int bit_idx  = abs_bit & 0x07;  // % 8

      if ((payload[byte_idx] >> bit_idx) & 0x01)
        value |= (uint16_t)(1u << bit);
    }

    out.channel[ch] = value;   // already bounded to 11 bits
  }

  return true;
}

//---------------------------------------------------------
// crsfTicksToMicros()
//
//   us = 1500 + (5/8) * (ticks - 992)

double crsfTicksToMicros(uint16_t ticks)
{
  return 1500.0 + (5.0 / 8.0) * ((double)ticks - (double)CRSF_TICKS_MID);
}

//---------------------------------------------------------
// crsfDecodeLinkStats()
//
// Fixed 10-byte layout. The only subtlety is sign handling:
// RSSI arrives as a positive magnitude denoting negative dBm,
// while SNR is already a signed int8. Getting either wrong
// produces plausible-looking but wrong numbers, so both are
// converted explicitly here and asserted in the tests.

bool crsfDecodeLinkStats(const uint8_t* payload, size_t len,
                         CrsfLinkStats& out)
{
  if (payload == nullptr || len != CRSF_LINK_STATS_PAYLOAD_SIZE)
    return false;

  // Wire value is |dBm|; negate to get true signed dBm.
  out.uplink_rssi_ant1    = (int16_t)(-(int16_t)payload[0]);
  out.uplink_rssi_ant2    = (int16_t)(-(int16_t)payload[1]);
  out.uplink_lq           = payload[2];
  out.uplink_snr          = (int8_t)payload[3];
  out.active_antenna      = payload[4];
  out.rf_mode             = payload[5];
  out.uplink_tx_power_idx = payload[6];
  out.downlink_rssi       = (int16_t)(-(int16_t)payload[7]);
  out.downlink_lq         = payload[8];
  out.downlink_snr        = (int8_t)payload[9];

  return true;
}

//---------------------------------------------------------
// crsfTxPowerMilliwatts()
//
// Index -> mW. Note index 7 is 50 mW, out of ascending order;
// that is the protocol's own table, not a transcription error.

uint16_t crsfTxPowerMilliwatts(uint8_t index)
{
  switch (index) {
    case 0: return 0;
    case 1: return 10;
    case 2: return 25;
    case 3: return 100;
    case 4: return 500;
    case 5: return 1000;
    case 6: return 2000;
    case 7: return 50;
    default: return 0;
  }
}

//---------------------------------------------------------
// crsfEncodeFrame()
//
// Simple (broadcast) header only: [sync][len][type][payload][crc].
// len counts type + payload + crc, matching the parser's view.

size_t crsfEncodeFrame(uint8_t type, const uint8_t* payload, size_t len,
                       std::vector<uint8_t>& out)
{
  if (len > CRSF_MAX_PAYLOAD)
    return 0;
  if (len > 0 && payload == nullptr)
    return 0;

  out.clear();
  out.push_back(CRSF_ADDRESS_FLIGHT_CTRL);      // 0xC8 sync
  out.push_back((uint8_t)(len + 2));            // type + payload + crc
  out.push_back(type);
  for (size_t i = 0; i < len; i++)
    out.push_back(payload[i]);

  // CRC covers type + payload, i.e. everything from out[2].
  out.push_back(crsfCrc8(&out[2], len + 1));
  return out.size();
}

//---------------------------------------------------------
// crsfEncodeBattery()
//
// Voltage/current are BIG-endian int16 in deci-units. Big-endian
// is worth flagging: the channel data elsewhere in this protocol
// is little-endian, so telemetry reverses the convention.

size_t crsfEncodeBattery(double volts, double amps,
                         uint32_t used_mah, uint8_t remaining_pct,
                         std::vector<uint8_t>& out)
{
  // Round rather than truncate, and clamp to int16 so an absurd
  // sensor reading cannot wrap into a plausible-looking value.
  double dv = volts * 10.0;
  double da = amps  * 10.0;
  if (dv >  32767.0) dv =  32767.0;
  if (dv < -32768.0) dv = -32768.0;
  if (da >  32767.0) da =  32767.0;
  if (da < -32768.0) da = -32768.0;

  const int16_t volt_dv = (int16_t)(dv >= 0 ? dv + 0.5 : dv - 0.5);
  const int16_t curr_da = (int16_t)(da >= 0 ? da + 0.5 : da - 0.5);

  if (used_mah > 0xFFFFFF) used_mah = 0xFFFFFF;   // 24-bit field
  if (remaining_pct > 100) remaining_pct = 100;

  uint8_t p[CRSF_BATTERY_PAYLOAD_SIZE];
  p[0] = (uint8_t)((volt_dv >> 8) & 0xFF);
  p[1] = (uint8_t)( volt_dv       & 0xFF);
  p[2] = (uint8_t)((curr_da >> 8) & 0xFF);
  p[3] = (uint8_t)( curr_da       & 0xFF);
  p[4] = (uint8_t)((used_mah >> 16) & 0xFF);
  p[5] = (uint8_t)((used_mah >>  8) & 0xFF);
  p[6] = (uint8_t)( used_mah        & 0xFF);
  p[7] = remaining_pct;

  return crsfEncodeFrame(CRSF_FRAMETYPE_BATTERY_SENSOR, p,
                         CRSF_BATTERY_PAYLOAD_SIZE, out);
}

//---------------------------------------------------------
// crsfEncodeFlightMode()
//
// Null-terminated string, max 14 bytes including the null. Longer
// strings are truncated rather than rejected: a too-long mode name
// should still show something useful on the handset.

size_t crsfEncodeFlightMode(const char* mode, std::vector<uint8_t>& out)
{
  if (mode == nullptr)
    return 0;

  uint8_t p[CRSF_FLIGHT_MODE_MAX_LEN];
  size_t n = 0;
  while (mode[n] != '\0' && n < CRSF_FLIGHT_MODE_MAX_LEN - 1) {
    p[n] = (uint8_t)mode[n];
    n++;
  }
  p[n++] = 0x00;   // null terminator is part of the payload

  return crsfEncodeFrame(CRSF_FRAMETYPE_FLIGHT_MODE, p, n, out);
}

//---------------------------------------------------------
// crsfEncodeGps()
//
// All fields BIG-endian, matching the battery frame. Unit
// conversions live here so callers stay in vehicle-native units
// (degrees / m/s / meters) and the wire quirks -- km/h deci-units,
// centi-degrees, the +1000 m altitude offset -- exist in exactly
// one place.
//
// Heading is normalised into [0, 360) before scaling: MOOS
// headings can arrive negative or > 360 depending on source, and
// the wire field is unsigned.

size_t crsfEncodeGps(double lat_deg, double lon_deg,
                     double speed_mps, double heading_deg,
                     double alt_m, uint8_t sats,
                     std::vector<uint8_t>& out)
{
  // Clamp coordinates to the physical range rather than letting a
  // bad nav solution wrap the int32 into a plausible position.
  if (lat_deg >  90.0)  lat_deg =  90.0;
  if (lat_deg < -90.0)  lat_deg = -90.0;
  if (lon_deg >  180.0) lon_deg =  180.0;
  if (lon_deg < -180.0) lon_deg = -180.0;

  const int32_t lat = (int32_t)(lat_deg * 1e7 + (lat_deg >= 0 ? 0.5 : -0.5));
  const int32_t lon = (int32_t)(lon_deg * 1e7 + (lon_deg >= 0 ? 0.5 : -0.5));

  // m/s -> (km/h)*10. Negative speed makes no sense on this
  // field; clamp to zero rather than casting a negative into
  // a huge unsigned value.
  double kph10 = speed_mps * 3.6 * 10.0;
  if (kph10 < 0.0)     kph10 = 0.0;
  if (kph10 > 65535.0) kph10 = 65535.0;
  const uint16_t spd = (uint16_t)(kph10 + 0.5);

  // Normalise heading into [0, 360), then centi-degrees.
  double hdg = heading_deg;
  while (hdg < 0.0)    hdg += 360.0;
  while (hdg >= 360.0) hdg -= 360.0;
  const uint16_t hdg_cd = (uint16_t)(hdg * 100.0 + 0.5);

  // Altitude carries a +1000 m offset (wire 0 = -1000 m).
  double alt = alt_m + 1000.0;
  if (alt < 0.0)     alt = 0.0;
  if (alt > 65535.0) alt = 65535.0;
  const uint16_t alt_w = (uint16_t)(alt + 0.5);

  uint8_t p[CRSF_GPS_PAYLOAD_SIZE];
  p[0]  = (uint8_t)((lat >> 24) & 0xFF);
  p[1]  = (uint8_t)((lat >> 16) & 0xFF);
  p[2]  = (uint8_t)((lat >>  8) & 0xFF);
  p[3]  = (uint8_t)( lat        & 0xFF);
  p[4]  = (uint8_t)((lon >> 24) & 0xFF);
  p[5]  = (uint8_t)((lon >> 16) & 0xFF);
  p[6]  = (uint8_t)((lon >>  8) & 0xFF);
  p[7]  = (uint8_t)( lon        & 0xFF);
  p[8]  = (uint8_t)((spd >> 8) & 0xFF);
  p[9]  = (uint8_t)( spd       & 0xFF);
  p[10] = (uint8_t)((hdg_cd >> 8) & 0xFF);
  p[11] = (uint8_t)( hdg_cd       & 0xFF);
  p[12] = (uint8_t)((alt_w >> 8) & 0xFF);
  p[13] = (uint8_t)( alt_w       & 0xFF);
  p[14] = sats;

  return crsfEncodeFrame(CRSF_FRAMETYPE_GPS, p,
                         CRSF_GPS_PAYLOAD_SIZE, out);
}

//---------------------------------------------------------
// crsfEncodeCustomDistance()
//
// Custom telemetry (0x80), subtype 0xB1: distance from the MOOS
// origin in whole meters, uint16 big-endian. See crsf_frames.h
// for the routing story (ELRS tunnel -> EdgeTX Lua -> "Dst"
// sensor); the handset-side decoder is SCRIPTS/MIXES/dstrx.lua.

size_t crsfEncodeCustomDistance(double meters,
                                std::vector<uint8_t>& out)
{
  if (meters < 0.0)     meters = 0.0;
  if (meters > 65535.0) meters = 65535.0;
  const uint16_t m = (uint16_t)(meters + 0.5);

  uint8_t p[3];
  p[0] = CRSF_CUSTOM_SUBTYPE_DISTANCE;
  p[1] = (uint8_t)((m >> 8) & 0xFF);
  p[2] = (uint8_t)( m       & 0xFF);

  return crsfEncodeFrame(CRSF_FRAMETYPE_CUSTOM_TELEM, p, 3, out);
}
