/*************************************************************
 * rc_telem_test - CRSF telemetry downlink test
 *
 * Sends battery + flight-mode telemetry to an ELRS receiver so
 * the return path can be verified on the handset.
 *
 * THE VERDICT IS ON THE HANDSET, not in this tool's output. The
 * "downlink" fields of the receiver's 0x14 LINK_STATISTICS frame
 * are NEVER populated by ELRS receiver firmware -- rx_main.cpp
 * writes them only under the DEBUG_BF_LINK_STATS build flag -- so
 * Pi-side downlink LQ reads 0 unconditionally, at every telemetry
 * ratio. The receiver cannot measure telemetry reception; only
 * the TX module can, and it reports that to the handset directly
 * as the TQly/TRSS/TSNR sensors.
 *
 * SUCCESS CRITERIA
 *   Telemetry appears on the handset (battery voltage on the
 *   TX16S telemetry page; mode string as "Flight mode") and TQly
 *   reads ~100 at bench range.
 *
 * If the handset shows nothing while this runs, work the list:
 *   1. Is Pi TX (GPIO 4 on ttyAMA1 / Navigator SERIAL 3) wired to
 *      the receiver's Serial RX pad?   raspi-gpio get 4   (then: raspi-gpio get 5)
 *      -> expect func=TXD3 / func=RXD3
 *   2. Is that receiver pad assigned to "Serial RX" in the ELRS
 *      WebUI Model tab? On PWM receivers the serial pins are
 *      channel pads reassigned to serial duty.
 *   3. Is the handset's telemetry ratio Off? Fleet setting is 1:8.
 *   4. Common gnd between Pi and receiver.
 *
 * Writes are non-blocking and never allowed to stall the read
 * path: on a boat, RC control latency is safety-critical and
 * telemetry is not. This tool models that discipline so the same
 * shape can move into iRCInterface.
 *
 * Author: Jeremy Wenger
 *************************************************************/

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <cerrno>
#include <cmath>
#include <string>
#include <vector>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <signal.h>
#include <time.h>

#include "crsf_parser.h"
#include "crsf_frames.h"

#define PROBE_BOTHER  0010000
#define PROBE_TCGETS2 0x802C542A
#define PROBE_TCSETS2 0x402C542B
#define CRSF_BAUDRATE 420000

static volatile sig_atomic_t g_stop = 0;
static void onSigint(int) { g_stop = 1; }

static uint64_t microsNow() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1000000ull + (uint64_t)ts.tv_nsec / 1000ull;
}

//---------------------------------------------------------------
// Open the port read-write at CRSF line settings.

static int openCrsfPort(const std::string &dev) {
  struct stat st;
  if (stat(dev.c_str(), &st) != 0) {
    printf("FAIL: %s does not exist (%s)\n", dev.c_str(), strerror(errno));
    return -1;
  }

  int fd = open(dev.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    printf("FAIL: open(): %s\n", strerror(errno));
    if (errno == EACCES)
      printf("  -> run as root or join the 'dialout' group.\n");
    return -1;
  }

  struct termios tio;
  if (tcgetattr(fd, &tio) < 0) {
    printf("FAIL: tcgetattr(): %s\n", strerror(errno));
    close(fd);
    return -1;
  }
  cfsetispeed(&tio, B38400);
  cfsetospeed(&tio, B38400);
  tio.c_cflag &= ~CSIZE;
  tio.c_cflag |= CS8;
  tio.c_cflag |= (CLOCAL | CREAD);
  tio.c_cflag &= ~PARENB;    // 8N1
  tio.c_cflag &= ~CSTOPB;
#ifdef CRTSCTS
  tio.c_cflag &= ~CRTSCTS;
#endif
  tio.c_iflag &= ~(IXON | IXOFF | IXANY);
  tio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tio.c_iflag &= ~(INLCR | ICRNL | INPCK | ISTRIP |
                   IGNBRK | BRKINT | PARMRK);

  // OUTPUT post-processing MUST be off. This matters more on the
  // TX path than it ever did for read-only SBUS work, and the
  // failure is vicious: with OPOST/ONLCR set, the driver rewrites
  // any 0x0A byte to 0x0D 0x0A. The battery frame's length byte
  // is 0x0A, so every battery frame would be silently corrupted
  // on the wire while flight-mode frames sailed through -- a
  // per-frame-type corruption with no error anywhere.
  // (Observed for real in a loopback harness that skipped this.)
  tio.c_oflag &= ~OPOST;
#ifdef ONLCR
  tio.c_oflag &= ~ONLCR;
#endif
#ifdef OCRNL
  tio.c_oflag &= ~OCRNL;
#endif
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;
  if (tcsetattr(fd, TCSANOW, &tio) < 0) {
    printf("FAIL: tcsetattr(): %s\n", strerror(errno));
    close(fd);
    return -1;
  }

#ifdef __linux__
  struct custom_termios2 {
    tcflag_t c_iflag, c_oflag, c_cflag, c_lflag;
    cc_t c_line;
    cc_t c_cc[19];
    speed_t c_ispeed, c_ospeed;
  };
  struct custom_termios2 tio2;
  if (ioctl(fd, PROBE_TCGETS2, &tio2) < 0) {
    printf("FAIL: TCGETS2: %s\n", strerror(errno));
    close(fd);
    return -1;
  }
  tio2.c_cflag &= ~0000017;
  tio2.c_cflag |= PROBE_BOTHER;
  tio2.c_ispeed = CRSF_BAUDRATE;
  tio2.c_ospeed = CRSF_BAUDRATE;
  if (ioctl(fd, PROBE_TCSETS2, &tio2) < 0) {
    printf("FAIL: TCSETS2: %s\n", strerror(errno));
    close(fd);
    return -1;
  }

  struct custom_termios2 chk;
  if (ioctl(fd, PROBE_TCGETS2, &chk) == 0) {
    printf("Port: %s @ %lu baud 8N1%s\n", dev.c_str(),
           (unsigned long)chk.c_ispeed,
           (chk.c_ispeed == CRSF_BAUDRATE) ? "" : "  <-- BAUD MISMATCH");
  }
#endif

  tcflush(fd, TCIFLUSH);
  return fd;
}

//---------------------------------------------------------------
// Non-blocking write. Telemetry must never stall the read path,
// so a frame that cannot be written immediately is DROPPED, not
// queued and not retried. Dropping is the correct behaviour: the
// next frame carries fresher data anyway.

static bool writeFrameNonBlocking(int fd, const std::vector<uint8_t> &f,
                                  uint64_t &dropped)
{
  ssize_t n = write(fd, f.data(), f.size());
  if (n == (ssize_t)f.size())
    return true;

  // EAGAIN (buffer full) or a short write: drop it.
  dropped++;
  return false;
}

static bool waitReadable(int fd, int timeout_ms) {
  fd_set rf;
  FD_ZERO(&rf);
  FD_SET(fd, &rf);
  struct timeval tv;
  tv.tv_sec  = timeout_ms / 1000;
  tv.tv_usec = (timeout_ms % 1000) * 1000;
  return select(fd + 1, &rf, nullptr, nullptr, &tv) > 0;
}

//---------------------------------------------------------------

static void usage(const char *prog) {
  printf("Usage: %s [OPTIONS]\n\n", prog);
  printf("  -d, --device DEV   serial device (default /dev/ttyAMA1)\n");
  printf("  -t, --time SEC     run duration, 0 = until Ctrl-C (default 30)\n");
  printf("  -r, --rate HZ      telemetry send rate (default 2)\n");
  printf("  -v, --volts V      battery voltage to report (default 24.6)\n");
  printf("  -a, --amps A       current to report (default 3.2)\n");
  printf("  -m, --mode STR     flight-mode string, max 13 chars (default BOAT)\n");
  printf("      --sweep        ramp the reported voltage over time, so the\n");
  printf("                     handset display can be seen changing\n");
  printf("      --no-mode      send battery only\n");
  printf("      --no-battery   send mode string only\n");
  printf("      --gps          also send GPS (0x02) with a simulated track:\n");
  printf("                     a slow circle at 1.5 m/s so GSpd/Hdg/GPS all\n");
  printf("                     visibly move on the handset\n");
  printf("      --lat DEG      simulated track centre latitude  (default 42.3585)\n");
  printf("      --lon DEG      simulated track centre longitude (default -71.0870)\n");
  printf("      --dist         also send custom distance telemetry (0x80/0xB1).\n");
  printf("                     With --gps: true distance from the track start.\n");
  printf("                     Without: a 0-150 m sawtooth. Needs dstrx.lua on\n");
  printf("                     the handset to appear as the \"Dst\" sensor.\n");
  printf("  -h, --help\n\n");
  printf("Run with the MOOS RC app STOPPED.\n");
}

int main(int argc, char **argv) {
  signal(SIGINT, onSigint);

  std::string dev = "/dev/ttyAMA1";
  std::string mode = "BOAT";
  double seconds = 30.0, rate_hz = 2.0;
  double volts = 24.6, amps = 3.2;
  bool sweep = false, send_mode = true, send_batt = true;
  bool send_gps = false, send_dist = false;
  double gps_lat = 42.3585, gps_lon = -71.0870;   // Charles River basin

  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    if      ((a == "-d" || a == "--device") && i + 1 < argc) dev = argv[++i];
    else if ((a == "-t" || a == "--time")   && i + 1 < argc) seconds = atof(argv[++i]);
    else if ((a == "-r" || a == "--rate")   && i + 1 < argc) rate_hz = atof(argv[++i]);
    else if ((a == "-v" || a == "--volts")  && i + 1 < argc) volts = atof(argv[++i]);
    else if ((a == "-a" || a == "--amps")   && i + 1 < argc) amps = atof(argv[++i]);
    else if ((a == "-m" || a == "--mode")   && i + 1 < argc) mode = argv[++i];
    else if (a == "--sweep")      sweep = true;
    else if (a == "--no-mode")    send_mode = false;
    else if (a == "--no-battery") send_batt = false;
    else if (a == "--gps")        send_gps = true;
    else if (a == "--dist")       send_dist = true;
    else if (a == "--lat" && i + 1 < argc) gps_lat = atof(argv[++i]);
    else if (a == "--lon" && i + 1 < argc) gps_lon = atof(argv[++i]);
    else if (a == "-h" || a == "--help") { usage(argv[0]); return 0; }
    else { printf("Unknown option: %s\n\n", a.c_str()); usage(argv[0]); return 1; }
  }

  if (rate_hz <= 0.0) rate_hz = 2.0;
  if (!send_mode && !send_batt && !send_gps && !send_dist) {
    printf("Nothing to send (both --no-mode and --no-battery given).\n");
    return 1;
  }

  printf("rc_telem_test -- CRSF telemetry downlink\n");
  printf("=====================================================\n");

  int fd = openCrsfPort(dev);
  if (fd < 0) return 1;

  printf("Sending: %s%s%s%s at %.1f Hz\n",
         send_batt ? "battery" : "",
         (send_batt && send_mode) ? " + " : "",
         send_mode ? ("mode \"" + mode + "\"").c_str() : "",
         send_gps ? " + gps (simulated track)" : "",
         rate_hz);
  printf("Reading receiver link stats to confirm the serial path is up.\n");
  printf("Judge arrival on the handset: sensors live + TQly ~100.\n");
  printf("Ctrl-C to stop.\n\n");

  CrsfParser parser;
  std::vector<CrsfFrame> frames;

  const uint64_t t0 = microsNow();
  const uint64_t t_end = (seconds > 0)
    ? t0 + (uint64_t)(seconds * 1e6) : (uint64_t)-1;
  const uint64_t send_interval_us = (uint64_t)(1e6 / rate_hz);

  uint64_t next_send = t0;
  uint64_t sent_batt = 0, sent_mode = 0, sent_gps = 0, sent_dist = 0,
           dropped = 0;
  uint64_t rc_frames = 0, link_frames = 0;
  uint64_t last_report = t0;

  bool have_link = false;
  CrsfLinkStats link = {};

  uint8_t rbuf[512];

  while (!g_stop && microsNow() < t_end) {

    // --- read side: keep consuming so link stats stay fresh ---
    if (waitReadable(fd, 20)) {
      ssize_t n = read(fd, rbuf, sizeof(rbuf));
      if (n > 0) {
        frames.clear();
        parser.feed(rbuf, (size_t)n, frames);
        for (const auto &f : frames) {
          if (f.type == CRSF_FRAMETYPE_RC_CHANNELS) {
            rc_frames++;
          } else if (f.type == CRSF_FRAMETYPE_LINK_STATISTICS) {
            link_frames++;
            CrsfLinkStats ls;
            if (crsfDecodeLinkStats(f.payload, ls)) {
              link = ls;
              have_link = true;
            }
          }
        }
      }
    }

    // --- write side: paced, non-blocking, drop on backpressure ---
    const uint64_t now = microsNow();
    if (now >= next_send) {
      next_send = now + send_interval_us;

      double v = volts;
      if (sweep) {
        // Sawtooth 20.0 -> 29.9 V over 10 s, so the handset value
        // visibly moves. A changing number proves the display is
        // live rather than a stale first reading.
        double elapsed = (double)(now - t0) / 1e6;
        v = 20.0 + (elapsed - 10.0 * (double)(long)(elapsed / 10.0));
      }

      std::vector<uint8_t> f;
      if (send_batt) {
        uint8_t pct = (uint8_t)((v - 20.0) / 9.0 * 100.0);
        if (v < 20.0) pct = 0;
        if (pct > 100) pct = 100;
        crsfEncodeBattery(v, amps, 0, pct, f);
        if (writeFrameNonBlocking(fd, f, dropped)) sent_batt++;
      }
      if (send_mode) {
        crsfEncodeFlightMode(mode.c_str(), f);
        if (writeFrameNonBlocking(fd, f, dropped)) sent_mode++;
      }
      if (send_gps) {
        // Simulated track: a 60 m radius circle at ~1.5 m/s
        // (~4 min per lap) around the configured centre. Every
        // GPS-derived handset sensor visibly changes: position
        // creeps, GSpd holds 1.5 m/s, Hdg sweeps 0-360.
        const double elapsed = (double)(now - t0) / 1e6;
        const double R = 60.0;                       // meters
        const double omega = 1.5 / R;                // rad/s
        const double th = omega * elapsed;
        // meters -> degrees, longitude scaled by cos(lat)
        const double m2deg = 1.0 / 111320.0;
        const double coslat = cos(gps_lat * M_PI / 180.0);
        const double lat = gps_lat + (R * sin(th)) * m2deg;
        const double lon = gps_lon + (R * cos(th)) * m2deg / coslat;
        // Tangent heading: circle runs counterclockwise in
        // (east,north) => course = th converted to compass.
        double hdg = 360.0 - (th * 180.0 / M_PI);
        crsfEncodeGps(lat, lon, 1.5, hdg, 2.0, 12, f);
        if (writeFrameNonBlocking(fd, f, dropped)) sent_gps++;
      }
      if (send_dist) {
        // Vehicle-computed distance from origin, as the real
        // iRCInterface will send it (hypot(NAV_X, NAV_Y)). Here:
        // with --gps, the true chord distance from the simulated
        // track's start point (0 -> 120 m -> 0 per lap); without,
        // a 0-150 m sawtooth over 30 s.
        const double elapsed = (double)(now - t0) / 1e6;
        double d;
        if (send_gps) {
          const double th = (1.5 / 60.0) * elapsed;
          d = 2.0 * 60.0 * fabs(sin(th / 2.0));
        } else {
          d = 5.0 * (elapsed - 30.0 * (double)(long)(elapsed / 30.0));
        }
        crsfEncodeCustomDistance(d, f);
        if (writeFrameNonBlocking(fd, f, dropped)) sent_dist++;
      }
    }

    // --- status line, 1 Hz ---
    if (now - last_report >= 1000000ull) {
      last_report = now;
      printf("tx: batt=%-4llu mode=%-4llu gps=%-4llu dst=%-4llu drop=%-3llu | "
             "rx: rc=%-5llu ls=%-4llu | ",
             (unsigned long long)sent_batt,
             (unsigned long long)sent_mode,
             (unsigned long long)sent_gps,
             (unsigned long long)sent_dist,
             (unsigned long long)dropped,
             (unsigned long long)rc_frames,
             (unsigned long long)link_frames);
      if (have_link) {
        printf("up LQ %3u%%\n", link.uplink_lq);
      } else {
        printf("no link stats yet\n");
      }
      fflush(stdout);
    }
  }

  close(fd);

  //---------------------------------------------------------------
  printf("\n=====================================================\n");
  printf(" RESULT\n");
  printf("=====================================================\n");
  printf("  Frames sent:      battery %llu, mode %llu, gps %llu, dist %llu\n",
         (unsigned long long)sent_batt, (unsigned long long)sent_mode,
         (unsigned long long)sent_gps, (unsigned long long)sent_dist);
  printf("  Write drops:      %llu\n", (unsigned long long)dropped);
  printf("  Frames received:  RC %llu, link-stats %llu\n",
         (unsigned long long)rc_frames, (unsigned long long)link_frames);
  printf("  Parser CRC fails: %llu\n",
         (unsigned long long)parser.framesCrcFail());
  if (have_link) {
    printf("  Uplink   LQ:      %u%%\n", link.uplink_lq);
  }
  printf("\n-----------------------------------------------------\n");

  if (sent_batt == 0 && sent_mode == 0 && sent_gps == 0 && sent_dist == 0) {
    printf(" NOTHING WAS SENT -- every write failed.\n\n"
           "  The port opened but would not accept writes. Check that\n"
           "  it was opened O_RDWR and that the UART has a TX line\n"
           "  enabled at all.\n");
    return 1;
  }

  if (link_frames == 0) {
    printf(" NO LINK STATISTICS RECEIVED\n\n"
           "  Telemetry was transmitted, but the receiver never sent\n"
           "  link-stats frames back, so the result cannot be judged.\n"
           "  Confirm the receiver is powered, bound, and in CRSF mode\n"
           "  (rc_probe --scan).\n");
    return 1;
  }

  printf(" FRAMES SENT -- the verdict is on the handset.\n\n");
  printf("  %llu frames were written with no errors and the receiver's\n"
         "  link stats kept arriving, so the serial path is up both\n"
         "  ways. Whether telemetry crossed the RF link can only be\n"
         "  judged on the TX16S:\n\n",
         (unsigned long long)(sent_batt + sent_mode + sent_gps + sent_dist));
  printf("    MDL -> PAGE to TELEMETRY -> \"Discover new sensors\",\n"
         "    with this tool running. If RxBt / Curr / Capa / Bat%%\n"
         "    appear and go stale when it stops, the return path\n"
         "    WORKS. TQly ~100 confirms downlink slots are healthy;\n"
         "    TQly missing or 0 = Telem Ratio drifted (expect 1:8).\n\n");
  printf("  Note: Pi-side downlink LQ in 0x14 is ALWAYS 0 -- ELRS rx\n"
         "  firmware writes those fields only in debug builds. It is\n"
         "  not a health indicator, so this tool does not report it.\n\n");
  printf("  Only if the handset shows NO sensors at all is this a\n"
         "  wiring or pin-assignment fault:\n"
         "   1. Pi TX -> receiver Serial RX pad?\n"
         "        raspi-gpio get 4     -> expect func=TXD3\n"
         "        raspi-gpio get 5     -> expect func=RXD3\n"
         "      (ttyAMA1 = Navigator SERIAL 3 = GPIO 4/5)\n"
         "   2. Receiver pad assigned to \"Serial RX\" in the ELRS\n"
         "      WebUI Model tab?\n"
         "   3. Common ground between Pi and receiver?\n");
  return 0;
}
