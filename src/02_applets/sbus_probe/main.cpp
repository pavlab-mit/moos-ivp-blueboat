/*************************************************************
 * sbus_probe - SBUS / RC link diagnostic utility
 *
 * Standalone CLI tool (no MOOS required) for debugging the
 * RadioLink AT9S Pro -> receiver -> UART -> iRCReader chain.
 *
 * Stages the diagnosis so each failure mode is distinguishable:
 *   1. Can the port be opened and configured for SBUS (100000
 *      baud, 8E2)?
 *   2. Are ANY bytes arriving on the wire?
 *   3. Do the bytes frame up as SBUS (0x0F header, valid footer)?
 *   4. Do frames decode with sane channel values?
 *   5. Are the receiver's failsafe / frame-lost flags clear?
 *
 * Modes:
 *   (default)      timed capture + automatic verdict
 *   --raw          live hex dump with inter-byte gap markers
 *   --frames       live decoded channel display (instrumented)
 *   --lib          live view through the real SbusHandler class
 *                  (reproduces exactly what iRCReader computes)
 *   --record FILE  capture timestamped raw bytes for offline
 *                  analysis
 *
 * NOTE: run this with iRCReader STOPPED. Two processes reading
 * the same tty split the byte stream between them and both see
 * corrupt frames. The tool detects other openers and refuses to
 * run unless --force is given.
 ************************************************************/

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <cerrno>
#include <string>
#include <vector>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <dirent.h>
#include <signal.h>
#include <time.h>

#include "sbus_handler.h"   // SBUS_* protocol constants + SbusHandler

// Custom BOTHER flag for custom baud rate (matches sbus_handler.cpp)
#define PROBE_BOTHER 0010000

// Linux termios2 ioctl numbers (matches sbus_handler.cpp)
#define PROBE_TCGETS2 0x802C542A
#define PROBE_TCSETS2 0x402C542B

// Same inter-byte gap the library uses to resync on a start byte.
static const uint32_t FRAME_GAP_THRESHOLD_US = 3000;

static volatile sig_atomic_t g_stop = 0;
static void onSigint(int) { g_stop = 1; }

static uint64_t microsNow() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
}

//---------------------------------------------------------------
// Port-holder detection: scan /proc/[pid]/fd for the device so we
// can warn when iRCReader (or a getty) already owns the port.
// Linux-only; returns empty list elsewhere.
struct PortHolder { int pid; std::string comm; };

static std::vector<PortHolder> findPortHolders(const std::string &dev) {
  std::vector<PortHolder> holders;
#ifdef __linux__
  DIR *proc = opendir("/proc");
  if (!proc) return holders;

  struct dirent *pe;
  while ((pe = readdir(proc)) != nullptr) {
    int pid = atoi(pe->d_name);
    if (pid <= 0 || pid == getpid()) continue;

    char fddir[64];
    snprintf(fddir, sizeof(fddir), "/proc/%d/fd", pid);
    DIR *fds = opendir(fddir);
    if (!fds) continue;   // usually EACCES; can't see other users' fds

    struct dirent *fe;
    bool holds = false;
    while ((fe = readdir(fds)) != nullptr) {
      char linkpath[128], target[256];
      snprintf(linkpath, sizeof(linkpath), "%s/%s", fddir, fe->d_name);
      ssize_t n = readlink(linkpath, target, sizeof(target) - 1);
      if (n <= 0) continue;
      target[n] = '\0';
      if (dev == target) { holds = true; break; }
    }
    closedir(fds);

    if (holds) {
      PortHolder h;
      h.pid = pid;
      char commpath[64];
      snprintf(commpath, sizeof(commpath), "/proc/%d/comm", pid);
      FILE *cf = fopen(commpath, "r");
      if (cf) {
        char comm[64] = {0};
        if (fgets(comm, sizeof(comm), cf)) {
          comm[strcspn(comm, "\n")] = '\0';
          h.comm = comm;
        }
        fclose(cf);
      }
      holders.push_back(h);
    }
  }
  closedir(proc);
#else
  (void)dev;
#endif
  return holders;
}

//---------------------------------------------------------------
// Raspberry Pi UART sanity hints. The Pi mini UART (ttyS0 on most
// models) has NO parity support in hardware -- SBUS 8E2 cannot be
// framed on it and every byte reads misaligned. The PL011
// (ttyAMA*) is required.
static void printPiUartHints(const std::string &dev) {
#ifdef __linux__
  FILE *mf = fopen("/proc/device-tree/model", "r");
  if (!mf) return;
  char model[128] = {0};
  size_t got = fread(model, 1, sizeof(model) - 1, mf);
  model[got] = '\0';
  fclose(mf);
  if (strstr(model, "Raspberry Pi") == nullptr) return;

  printf("  Platform: %s\n", model);

  // Resolve the serial0/serial1 aliases if present.
  for (const char *alias : {"serial0", "serial1"}) {
    char linkpath[64], target[128];
    snprintf(linkpath, sizeof(linkpath), "/dev/%s", alias);
    ssize_t n = readlink(linkpath, target, sizeof(target) - 1);
    if (n > 0) {
      target[n] = '\0';
      printf("  /dev/%s -> %s\n", alias, target);
    }
  }

  if (dev.find("ttyS") != std::string::npos) {
    printf("  WARNING: %s is the Pi *mini UART* on most Pi models.\n"
           "           The mini UART has NO parity support; SBUS is\n"
           "           8E2 and cannot be framed reliably on it. It\n"
           "           may partially decode on some board revisions\n"
           "           (timing luck), but expect heavy frame loss or\n"
           "           total garbage. Use a PL011 UART (/dev/ttyAMA*).\n",
           dev.c_str());
  }
#else
  (void)dev;
#endif
}

//---------------------------------------------------------------
// Open + configure the port exactly the way SbusHandler does, but
// report every step and read the config back for verification.
static int openSbusPort(const std::string &dev, bool verbose) {
  if (verbose) printf("[1] Opening %s ...\n", dev.c_str());

  struct stat st;
  if (stat(dev.c_str(), &st) != 0) {
    printf("  FAIL: %s does not exist (%s)\n", dev.c_str(), strerror(errno));
    printf("  -> Wrong device path, or the UART is not enabled.\n");
    return -1;
  }
  if (!S_ISCHR(st.st_mode)) {
    printf("  FAIL: %s is not a character device\n", dev.c_str());
    return -1;
  }

  int fd = open(dev.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    printf("  FAIL: open(): %s\n", strerror(errno));
    if (errno == EACCES)
      printf("  -> Permission denied: run as root or add user to the"
             " 'dialout' group.\n");
    if (errno == EBUSY)
      printf("  -> Device busy: another process has it open exclusively.\n");
    return -1;
  }
  if (verbose) printf("  OK: opened (fd=%d)\n", fd);

  // --- termios base config: raw, 8E2 (same as SbusHandler) ---
  if (verbose) printf("[2] Configuring 100000 baud, 8E2, raw ...\n");
  struct termios tio;
  if (tcgetattr(fd, &tio) < 0) {
    printf("  FAIL: tcgetattr(): %s\n", strerror(errno));
    close(fd);
    return -1;
  }
  cfsetispeed(&tio, B38400);
  cfsetospeed(&tio, B38400);
  tio.c_cflag &= ~CSIZE;
  tio.c_cflag |= CS8;
  tio.c_cflag |= PARENB;
  tio.c_cflag &= ~PARODD;
  tio.c_cflag |= CSTOPB;
#ifdef CRTSCTS
  tio.c_cflag &= ~CRTSCTS;
#endif
  tio.c_iflag &= ~(IXON | IXOFF | IXANY);
  tio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tio.c_iflag &= ~(INLCR | ICRNL);
  // Do NOT set INPCK: let bytes with parity errors through, they
  // are diagnostic signal (misalignment / inverted line).
  tio.c_iflag &= ~(INPCK | ISTRIP | IGNBRK | BRKINT | PARMRK);
  tio.c_oflag &= ~OPOST;
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;
  if (tcsetattr(fd, TCSANOW, &tio) < 0) {
    printf("  FAIL: tcsetattr(): %s\n", strerror(errno));
    close(fd);
    return -1;
  }

  // --- custom baud via termios2 (Linux only) ---
  struct custom_termios2 {
    tcflag_t c_iflag, c_oflag, c_cflag, c_lflag;
    cc_t c_line;
    cc_t c_cc[19];
    speed_t c_ispeed, c_ospeed;
  };
  struct custom_termios2 tio2;
  if (ioctl(fd, PROBE_TCGETS2, &tio2) < 0) {
    printf("  FAIL: TCGETS2 ioctl: %s\n", strerror(errno));
    printf("  -> Custom 100000 baud unavailable on this driver/OS.\n");
    close(fd);
    return -1;
  }
  tio2.c_cflag &= ~0000017;   // clear CBAUD
  tio2.c_cflag |= PROBE_BOTHER;
  tio2.c_ispeed = SBUS_BAUDRATE;
  tio2.c_ospeed = SBUS_BAUDRATE;
  if (ioctl(fd, PROBE_TCSETS2, &tio2) < 0) {
    printf("  FAIL: TCSETS2 ioctl: %s\n", strerror(errno));
    close(fd);
    return -1;
  }

  // Read back and verify the settings actually stuck. Some
  // drivers (notably the Pi mini UART) silently drop parity.
  struct custom_termios2 chk;
  if (ioctl(fd, PROBE_TCGETS2, &chk) == 0) {
    bool baud_ok   = (chk.c_ispeed == SBUS_BAUDRATE);
    bool parity_ok = (chk.c_cflag & PARENB) && !(chk.c_cflag & PARODD);
    bool stop_ok   = (chk.c_cflag & CSTOPB);
    if (verbose) {
      printf("  Readback: baud=%lu (%s)  even-parity=%s  2-stop=%s\n",
             (unsigned long)chk.c_ispeed,
             baud_ok ? "OK" : "MISMATCH",
             parity_ok ? "OK" : "MISSING",
             stop_ok ? "OK" : "MISSING");
    }
    if (!baud_ok || !parity_ok || !stop_ok)
      printf("  WARNING: driver did not accept the full SBUS line\n"
             "           config -- frames will likely be corrupt.\n");
  }

  tcflush(fd, TCIFLUSH);
  if (verbose) printf("  OK: port configured\n");
  return fd;
}

//---------------------------------------------------------------
// Instrumented SBUS parser: identical sync + decode rules to
// SbusHandler, but counts the REASON each frame is rejected.
struct ProbeStats {
  // byte level
  uint64_t bytes_total = 0;
  uint64_t byte_hist[256] = {0};
  uint64_t first_byte_us = 0;
  uint64_t last_byte_us = 0;

  // framing level
  uint64_t syncs = 0;            // start-byte resyncs
  uint64_t frames_complete = 0;  // 25 bytes assembled
  uint64_t incomplete = 0;       // resync fired mid-frame
  uint64_t overlong = 0;

  // decode level
  uint64_t rej_footer = 0;
  uint64_t footer_hist[256] = {0};
  uint64_t rej_range = 0;
  uint64_t range_viol[SBUS_NUM_CHANNELS] = {0};
  uint64_t frames_valid = 0;     // header+footer+range all good

  // receiver flags on valid frames
  uint64_t flagged_frame_lost = 0;
  uint64_t flagged_failsafe = 0;
  uint64_t frames_clean = 0;     // valid AND no lost/failsafe flags

  // channel content
  uint16_t ch_last[SBUS_NUM_CHANNELS] = {0};
  uint16_t ch_min[SBUS_NUM_CHANNELS];
  uint16_t ch_max[SBUS_NUM_CHANNELS];
  uint8_t  last_flags = 0;

  // frame timing
  uint64_t prev_valid_us = 0;
  uint64_t last_valid_us = 0;
  double interval_sum = 0, interval_min = 1e18, interval_max = 0;
  uint64_t interval_n = 0;

  // library-equivalent connection state machine
  bool     lib_connected = false;
  uint64_t consecutive_good = 0;
  uint64_t consecutive_losses = 0;

  ProbeStats() {
    for (int i = 0; i < SBUS_NUM_CHANNELS; i++) {
      ch_min[i] = 0xFFFF;
      ch_max[i] = 0;
    }
  }
};

class ProbeParser {
public:
  explicit ProbeParser(ProbeStats &stats) : s_(stats) {}

  void feed(const uint8_t *buf, ssize_t n, uint64_t t_us) {
    for (ssize_t i = 0; i < n; i++) {
      uint8_t byte = buf[i];
      uint64_t gap = t_us - last_byte_us_;
      last_byte_us_ = t_us;

      s_.bytes_total++;
      s_.byte_hist[byte]++;
      if (s_.first_byte_us == 0) s_.first_byte_us = t_us;
      s_.last_byte_us = t_us;

      // Same resync rule as SbusHandler::update()
      if (byte == SBUS_START_BYTE && (gap > FRAME_GAP_THRESHOLD_US || !in_frame_)) {
        if (in_frame_ && idx_ > 0) {
          s_.incomplete++;
          s_.consecutive_losses++;
        }
        in_frame_ = true;
        idx_ = 0;
        frame_[idx_++] = byte;
        s_.syncs++;
        continue;
      }
      if (!in_frame_) continue;

      if (idx_ < SBUS_FRAME_SIZE) {
        frame_[idx_++] = byte;
        if (idx_ == SBUS_FRAME_SIZE) {
          s_.frames_complete++;
          decode(t_us);
          in_frame_ = false;
          idx_ = 0;
        }
      } else {
        in_frame_ = false;
        idx_ = 0;
        s_.overlong++;
        s_.consecutive_losses++;
      }
    }

    // Staleness backstop (same thresholds as the library). Uses the
    // caller-supplied clock so --replay ages correctly too.
    bool stale =
        (s_.last_valid_us != 0 &&
         t_us - s_.last_valid_us > (uint64_t)SBUS_SIGNAL_LOSS_TIMEOUT_MS * 1000) ||
        (s_.consecutive_losses > SBUS_MAX_CONSECUTIVE_LOSS);
    if (stale) {
      s_.lib_connected = false;
      s_.consecutive_good = 0;
    }
  }

private:
  void decode(uint64_t t_us) {
    // Footer check (0x00, or SBUS2 telemetry slots x4/x14/x24/x34)
    uint8_t footer = frame_[SBUS_FOOTER_OFFSET];
    s_.footer_hist[footer]++;
    if (footer != SBUS_END_BYTE && (footer & 0x0F) != 0x04) {
      s_.rej_footer++;
      s_.consecutive_losses++;
      return;
    }

    uint16_t ch[SBUS_NUM_CHANNELS];
    ch[0]  = ((frame_[1] | (frame_[2] << 8)) & 0x07FF);
    ch[1]  = ((frame_[2] >> 3) | (frame_[3] << 5)) & 0x07FF;
    ch[2]  = ((frame_[3] >> 6) | (frame_[4] << 2) | (frame_[5] << 10)) & 0x07FF;
    ch[3]  = ((frame_[5] >> 1) | (frame_[6] << 7)) & 0x07FF;
    ch[4]  = ((frame_[6] >> 4) | (frame_[7] << 4)) & 0x07FF;
    ch[5]  = ((frame_[7] >> 7) | (frame_[8] << 1) | (frame_[9] << 9)) & 0x07FF;
    ch[6]  = ((frame_[9] >> 2) | (frame_[10] << 6)) & 0x07FF;
    ch[7]  = ((frame_[10] >> 5) | (frame_[11] << 3)) & 0x07FF;
    ch[8]  = ((frame_[12] | (frame_[13] << 8)) & 0x07FF);
    ch[9]  = ((frame_[13] >> 3) | (frame_[14] << 5)) & 0x07FF;
    ch[10] = ((frame_[14] >> 6) | (frame_[15] << 2) | (frame_[16] << 10)) & 0x07FF;
    ch[11] = ((frame_[16] >> 1) | (frame_[17] << 7)) & 0x07FF;
    ch[12] = ((frame_[17] >> 4) | (frame_[18] << 4)) & 0x07FF;
    ch[13] = ((frame_[18] >> 7) | (frame_[19] << 1) | (frame_[20] << 9)) & 0x07FF;
    ch[14] = ((frame_[20] >> 2) | (frame_[21] << 6)) & 0x07FF;
    ch[15] = ((frame_[21] >> 5) | (frame_[22] << 3)) & 0x07FF;

    // Range check -- record WHICH channels violate. This is the
    // check that silently discards frames in the library when a
    // receiver emits 0 (or another out-of-band value) on channels
    // the transmitter isn't driving.
    bool range_ok = true;
    for (int i = 0; i < SBUS_NUM_CHANNELS; i++) {
      if (ch[i] < s_.ch_min[i]) s_.ch_min[i] = ch[i];
      if (ch[i] > s_.ch_max[i]) s_.ch_max[i] = ch[i];
      if (ch[i] < SBUS_MIN_VALUE || ch[i] > SBUS_MAX_VALUE) {
        s_.range_viol[i]++;
        range_ok = false;
      }
    }
    if (!range_ok) {
      s_.rej_range++;
      s_.consecutive_losses++;
      return;
    }

    // Valid frame
    s_.frames_valid++;
    memcpy(s_.ch_last, ch, sizeof(ch));
    uint8_t flags = frame_[SBUS_FLAGS_OFFSET];
    s_.last_flags = flags;
    bool lost = (flags & SBUS_FLAG_FRAME_LOST) != 0;
    bool fs   = (flags & SBUS_FLAG_FAILSAFE) != 0;
    if (lost) s_.flagged_frame_lost++;
    if (fs)   s_.flagged_failsafe++;

    s_.consecutive_losses = 0;
    if (s_.prev_valid_us != 0) {
      double dt = (t_us - s_.prev_valid_us) / 1000.0;
      s_.interval_sum += dt;
      s_.interval_n++;
      if (dt < s_.interval_min) s_.interval_min = dt;
      if (dt > s_.interval_max) s_.interval_max = dt;
    }
    s_.prev_valid_us = t_us;
    s_.last_valid_us = t_us;

    // Library connection state machine (asymmetric hysteresis)
    if (!lost && !fs) {
      s_.frames_clean++;
      s_.consecutive_good++;
      if (s_.consecutive_good >= SBUS_HYSTERESIS_GOOD_FRAMES)
        s_.lib_connected = true;
    } else {
      s_.consecutive_good = 0;
      s_.lib_connected = false;
    }
  }

  ProbeStats &s_;
  uint8_t frame_[SBUS_FRAME_SIZE] = {0};
  int idx_ = 0;
  bool in_frame_ = false;
  uint64_t last_byte_us_ = 0;
};

//---------------------------------------------------------------
// Replay a --record capture through the instrumented parser and
// print the same verdict as a live diag run. Lets captures taken
// on the boat be analyzed anywhere.
static void printVerdict(const ProbeStats &s, double seconds);

static int runReplay(const std::string &file, ProbeStats &stats) {
  FILE *f = fopen(file.c_str(), "r");
  if (!f) {
    printf("Cannot open %s: %s\n", file.c_str(), strerror(errno));
    return 1;
  }
  ProbeParser parser(stats);
  char line[2048];
  uint64_t first_us = 0, last_us = 0;
  while (fgets(line, sizeof(line), f)) {
    char *p = line;
    unsigned long long t = strtoull(p, &p, 10);
    if (first_us == 0) first_us = t;
    last_us = t;
    uint8_t buf[256];
    ssize_t n = 0;
    while (n < (ssize_t)sizeof(buf)) {
      while (*p == ' ') p++;
      if (*p == '\0' || *p == '\n') break;
      char *end;
      long v = strtol(p, &end, 16);
      if (end == p) break;
      buf[n++] = (uint8_t)v;
      p = end;
    }
    parser.feed(buf, n, t);
  }
  fclose(f);
  double seconds = (last_us - first_us) / 1e6;
  printf("Replayed %s: %llu bytes over %.1fs\n", file.c_str(),
         (unsigned long long)stats.bytes_total, seconds);
  printVerdict(stats, seconds > 0 ? seconds : 1.0);
  return 0;
}

//---------------------------------------------------------------
// Wait up to timeout_ms for readable data.
static bool waitReadable(int fd, int timeout_ms) {
  fd_set rfds;
  FD_ZERO(&rfds);
  FD_SET(fd, &rfds);
  struct timeval tv;
  tv.tv_sec = timeout_ms / 1000;
  tv.tv_usec = (timeout_ms % 1000) * 1000;
  return select(fd + 1, &rfds, nullptr, nullptr, &tv) > 0;
}

//---------------------------------------------------------------
// Verdict printer for the default diag mode.
static void printVerdict(const ProbeStats &s, double seconds) {
  printf("\n================ DIAGNOSIS ================\n");

  double bps = s.bytes_total / seconds;

  if (s.bytes_total == 0) {
    printf("VERDICT: NO DATA ON THE WIRE\n\n");
    printf("The port opened and configured fine, but zero bytes\n"
           "arrived. The transmitter being 'connected' only means\n"
           "the RF link is bound -- it says nothing about the\n"
           "receiver's serial output. Check, in order:\n"
           "  1. Receiver output mode. RadioLink receivers (R9DS/\n"
           "     R12DS) ship in PWM mode; SBUS must be enabled on\n"
           "     the receiver itself (R9DS: short-press the ID SET\n"
           "     button twice within 1s; blue LED = SBUS on CH9\n"
           "     port, red = PWM). In PWM mode the SBUS pin is\n"
           "     silent -- exactly this symptom.\n"
           "  2. Wiring: receiver SBUS signal pin -> UART RX pin,\n"
           "     common ground, receiver actually powered.\n"
           "  3. Right UART device (try the other /dev/ttyAMA* /\n"
           "     /dev/ttyS* candidates; see scripts/sbus_diag.sh).\n"
           "  4. UART enabled in boot config / not claimed by a\n"
           "     serial console.\n");
    return;
  }

  printf("Bytes: %llu in %.1fs (%.0f B/s; healthy SBUS is ~1700-3600 B/s)\n",
         (unsigned long long)s.bytes_total, seconds, bps);

  double pct00 = 100.0 * s.byte_hist[0x00] / s.bytes_total;
  double pctFF = 100.0 * s.byte_hist[0xFF] / s.bytes_total;
  double pct0F = 100.0 * s.byte_hist[0x0F] / s.bytes_total;
  printf("Byte mix: 0x00=%.1f%%  0xFF=%.1f%%  0x0F=%.1f%% (expect ~4%% 0x0F)\n",
         pct00, pctFF, pct0F);

  // Random garbage occasionally assembles a "frame" by luck; judge
  // framing by yield against the byte count, not by count > 0.
  uint64_t expected_frames = s.bytes_total / SBUS_FRAME_SIZE;
  bool framing_broken =
      s.frames_complete == 0 ||
      (s.frames_valid == 0 &&
       s.frames_complete < (expected_frames / 5 > 3 ? expected_frames / 5 : 3));

  if (framing_broken) {
    printf("\nVERDICT: DATA ARRIVES BUT NEVER FRAMES UP\n\n");
    if (s.frames_complete > 0)
      printf("(%llu of ~%llu possible frames assembled by chance -- noise.)\n\n",
             (unsigned long long)s.frames_complete,
             (unsigned long long)expected_frames);
    if (pct00 > 60.0 || pctFF > 60.0) {
      printf("The byte stream is dominated by 0x%02X, which is the\n"
             "signature of an ELECTRICALLY INVERTED line (SBUS is an\n"
             "inverted-logic protocol; a UART without an inverter\n"
             "reads mostly break/idle garbage). Fix: use a hardware\n"
             "signal inverter, or a receiver/port that outputs\n"
             "non-inverted serial.\n", pct00 > pctFF ? 0x00 : 0xFF);
    } else {
      printf("Bytes flow but no 25-byte frame ever assembled behind\n"
             "an 0x0F header. Likely line-config mismatch:\n"
             "  - Port is a Pi mini UART (no parity HW) -> use PL011\n"
             "  - Wrong baud/parity -> every byte misframed\n"
             "  - Severe RF interference corrupting most bytes\n");
    }
    return;
  }

  printf("Frames: %llu assembled, %llu valid, %llu clean (no failsafe/lost)\n",
         (unsigned long long)s.frames_complete,
         (unsigned long long)s.frames_valid,
         (unsigned long long)s.frames_clean);
  if (s.interval_n > 0)
    printf("Frame interval: mean %.1f ms (min %.1f / max %.1f); RadioLink nominal ~14 ms\n",
           s.interval_sum / s.interval_n, s.interval_min, s.interval_max);

  if (s.frames_valid == 0) {
    if (s.rej_range > s.rej_footer) {
      printf("\nVERDICT: FRAMES DECODE BUT CHANNEL VALUES ARE OUT OF RANGE\n\n");
      printf("Every frame was rejected by the [%d..%d] channel range\n"
             "check in sbus_handler.cpp. Out-of-range channels:\n",
             SBUS_MIN_VALUE, SBUS_MAX_VALUE);
      for (int i = 0; i < SBUS_NUM_CHANNELS; i++) {
        if (s.range_viol[i] > 0)
          printf("  CH%-2d: %llu violations (observed %u..%u)\n", i + 1,
                 (unsigned long long)s.range_viol[i], s.ch_min[i], s.ch_max[i]);
      }
      printf("\nIf only high channels (e.g. CH10-16) violate and they\n"
             "sit at a constant value like 0: the receiver is not\n"
             "driving those channels and the library's all-16-channel\n"
             "range check is discarding otherwise-good frames. Fix in\n"
             "the transmitter (enable all channels / set the AT9S Pro\n"
             "to 12CH mode in SYSTEM->CH SELECT) or relax the check\n"
             "to only validate driven channels.\n");
    } else {
      printf("\nVERDICT: FRAME ALIGNMENT / FOOTER FAILURES\n\n");
      printf("Footer bytes observed (should be 0x00 or 0xX4):\n");
      for (int b = 0; b < 256; b++)
        if (s.footer_hist[b] > 0)
          printf("  0x%02X: %llu\n", b, (unsigned long long)s.footer_hist[b]);
      printf("Consistent wrong footers = byte misalignment, usually a\n"
             "parity/baud problem (mini UART?) or a non-SBUS protocol\n"
             "on this pin (e.g. i-BUS/PPM).\n");
    }
    return;
  }

  // Wire-level yield: how many of the frames the receiver SENT
  // actually survived to a valid decode? The shortest observed
  // interval between valid frames is the true frame period, so
  // seconds/interval_min estimates the sent count. A link can look
  // "healthy" from the valid frames alone while losing half of
  // them (classic mini-UART / marginal-clock behavior).
  if (s.interval_n >= 10 && s.interval_min > 1.0) {
    double sent_est = seconds * 1000.0 / s.interval_min;
    double yield = s.frames_valid / sent_est;
    if (yield < 0.85) {
      printf("\nVERDICT: DECODES, BUT HEAVY WIRE-LEVEL FRAME LOSS\n\n");
      printf("  ~%.0f frames sent, %llu decoded valid (%.0f%% yield)\n",
             sent_est, (unsigned long long)s.frames_valid, yield * 100.0);
      printf("  assembled-but-rejected: %llu   incomplete: %llu\n\n",
             (unsigned long long)(s.frames_complete - s.frames_valid),
             (unsigned long long)s.incomplete);
      printf("The frames that decode are fine, but a large share die\n"
             "on the wire. On a mini UART (parity refused in the\n"
             "readback above) this is expected: it tolerates SBUS only\n"
             "by timing luck, which drifts with board revision, core\n"
             "clock, and temperature. RC_CONNECTED will flap in the\n"
             "field. Move to a PL011 (/dev/ttyAMA*) for a 100%% yield\n"
             "link. If this IS a PL011, suspect wiring/signal quality.\n");
      return;
    }
  }

  double clean_pct = 100.0 * s.frames_clean / s.frames_valid;

  // Even occasional flagged frames matter: one bad frame resets the
  // library's hysteresis, so RC_CONNECTED needs
  // SBUS_HYSTERESIS_GOOD_FRAMES consecutive clean frames to recover.
  if (clean_pct >= 50.0 && clean_pct < 99.5) {
    printf("\nVERDICT: LINK DECODES BUT DROPS FRAMES INTERMITTENTLY\n\n");
    printf("  %.1f%% of valid frames clean; failsafe=%llu frame_lost=%llu\n",
           clean_pct,
           (unsigned long long)s.flagged_failsafe,
           (unsigned long long)s.flagged_frame_lost);
    printf("  Library-equivalent RC_CONNECTED ended: %s\n\n",
           s.lib_connected ? "TRUE" : "FALSE");
    printf("The serial side decodes fine but the receiver keeps\n"
           "flagging RF dropouts. A single flagged frame drops\n"
           "RC_CONNECTED and it takes %d consecutive clean frames to\n"
           "recover, so even moderate flag rates read as 'not\n"
           "connected'. Check antenna placement/orientation, distance,\n"
           "2.4GHz interference (WiFi!), and receiver mounting near\n"
           "power electronics.\n", SBUS_HYSTERESIS_GOOD_FRAMES);
    return;
  }

  if (clean_pct < 50.0) {
    printf("\nVERDICT: LINK DECODES BUT RECEIVER FLAGS FAILSAFE/FRAME-LOST\n\n");
    printf("  failsafe flagged:   %llu frames\n"
           "  frame_lost flagged: %llu frames\n\n",
           (unsigned long long)s.flagged_failsafe,
           (unsigned long long)s.flagged_frame_lost);
    printf("The serial side is healthy -- the RECEIVER itself says the\n"
           "RF link is bad or in failsafe. The transmitter's connect\n"
           "indicator can lie. Try: re-bind, check antenna placement,\n"
           "move transmitter closer, verify no 2nd transmitter bound.\n");
    return;
  }

  printf("\nVERDICT: SBUS LINK IS HEALTHY\n\n");
  printf("Library-equivalent RC_CONNECTED would be: %s\n\n",
         s.lib_connected ? "TRUE" : "FALSE");
  printf("If iRCReader still reports not connected, the problem is\n"
         "app-side, not the link:\n"
         "  - iRCReader reading a DIFFERENT device: set 'device = <dev>'\n"
         "    in its ProcessConfig block (default is %s); the\n"
         "    appcast 'Device:' line shows what it actually opened\n"
         "  - iRCReader failed to open the port (check its appcast\n"
         "    run warnings for 'Failed to initialize SBUS handler')\n"
         "  - another process was competing for the port while\n"
         "    iRCReader ran\n", SBUS_UART_DEV);
}

//---------------------------------------------------------------
// Live channel table (used by --frames)
static void printLiveFrame(const ProbeStats &s) {
  printf("\033[2J\033[H");   // clear screen, home cursor
  printf("sbus_probe --frames   (Ctrl-C to quit)\n");
  printf("============================================\n");
  printf("bytes=%llu  frames: complete=%llu valid=%llu clean=%llu\n",
         (unsigned long long)s.bytes_total,
         (unsigned long long)s.frames_complete,
         (unsigned long long)s.frames_valid,
         (unsigned long long)s.frames_clean);
  printf("rejects: footer=%llu range=%llu incomplete=%llu\n",
         (unsigned long long)s.rej_footer,
         (unsigned long long)s.rej_range,
         (unsigned long long)s.incomplete);
  bool lost = s.last_flags & SBUS_FLAG_FRAME_LOST;
  bool fs   = s.last_flags & SBUS_FLAG_FAILSAFE;
  printf("flags: frame_lost=%s failsafe=%s   RC_CONNECTED(equiv)=%s\n\n",
         lost ? "YES" : "no", fs ? "YES" : "no",
         s.lib_connected ? "TRUE" : "FALSE");

  printf("  CH   RAW    min..max   [%d..%d ok]\n", SBUS_MIN_VALUE, SBUS_MAX_VALUE);
  for (int i = 0; i < SBUS_NUM_CHANNELS; i++) {
    bool bad = s.ch_last[i] < SBUS_MIN_VALUE || s.ch_last[i] > SBUS_MAX_VALUE;
    // 40-char bar over the full 11-bit range
    int fill = (int)(40.0 * s.ch_last[i] / 2047.0);
    char bar[41];
    for (int b = 0; b < 40; b++) bar[b] = (b < fill) ? '#' : '.';
    bar[40] = '\0';
    printf("  %-4d %-6u %4u..%-4u %s %s\n", i + 1, s.ch_last[i],
           s.ch_min[i] == 0xFFFF ? 0 : s.ch_min[i], s.ch_max[i], bar,
           bad ? "<-- OUT OF RANGE" : "");
  }
  fflush(stdout);
}

//---------------------------------------------------------------
// Mode: sweep all candidate UARTs and report which one carries
// SBUS traffic. Answers "which /dev/tty* did the wire land on?"
static int runScan(double seconds_per_port) {
  const char *candidates[] = {
    "/dev/ttyS0",   "/dev/ttyAMA0", "/dev/ttyAMA1", "/dev/ttyAMA2",
    "/dev/ttyAMA3", "/dev/ttyAMA4", "/dev/ttyAMA5",
  };
  printf("Scanning candidate UARTs (%.0fs each) ...\n\n", seconds_per_port);
  printf("  %-14s %-10s %-8s %-8s %s\n",
         "device", "bytes", "headers", "valid", "note");

  std::string best;
  for (const char *dev : candidates) {
    struct stat st;
    if (stat(dev, &st) != 0 || !S_ISCHR(st.st_mode)) continue;
    if (g_stop) break;

    std::vector<PortHolder> holders = findPortHolders(dev);
    if (!holders.empty()) {
      printf("  %-14s %-10s %-8s %-8s in use by pid %d (%s) - skipped\n",
             dev, "-", "-", "-", holders[0].pid, holders[0].comm.c_str());
      continue;
    }

    int fd = open(dev, O_RDONLY | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
      printf("  %-14s %-10s %-8s %-8s open failed: %s\n",
             dev, "-", "-", "-", strerror(errno));
      continue;
    }
    close(fd);
    // Reopen with full SBUS config, quietly.
    fd = openSbusPort(dev, false);
    if (fd < 0) continue;

    ProbeStats stats;
    ProbeParser parser(stats);
    uint64_t t_end = microsNow() + (uint64_t)(seconds_per_port * 1e6);
    while (!g_stop && microsNow() < t_end) {
      if (waitReadable(fd, 100)) {
        uint8_t buf[256];
        ssize_t n = read(fd, buf, sizeof(buf));
        if (n > 0) parser.feed(buf, n, microsNow());
      }
    }
    close(fd);

    const char *note = "";
    if (stats.bytes_total == 0)
      note = "silent";
    else if (stats.frames_valid > 0) {
      note = "SBUS TRAFFIC <-- use this port";
      best = dev;
    } else if (stats.frames_complete > 0)
      note = "frames but none valid (run full diag here)";
    else
      note = "bytes but no SBUS framing (parity-less UART? noise?)";

    printf("  %-14s %-10llu %-8llu %-8llu %s\n", dev,
           (unsigned long long)stats.bytes_total,
           (unsigned long long)stats.syncs,
           (unsigned long long)stats.frames_valid, note);
  }

  if (!best.empty())
    printf("\nNext: sbus_probe -d %s   (full diag + verdict)\n", best.c_str());
  else
    printf("\nNo port showed valid SBUS frames. If one showed bytes,\n"
           "run the full diag on it; if all are silent, the receiver\n"
           "is not outputting (mode/wiring/power).\n");
  return 0;
}

//---------------------------------------------------------------
// Mode: run the REAL SbusHandler and display its getters -- this
// reproduces exactly what iRCReader::Iterate() computes.
static int runLibMode(const std::string &dev) {
  printf("Running the real SbusHandler on %s ...\n", dev.c_str());
  SbusHandler sbus(dev);
  if (!sbus.initialize()) {
    printf("SbusHandler::initialize() FAILED -- same failure iRCReader\n"
           "would hit (it only surfaces this as an appcast run warning).\n");
    return 1;
  }
  uint64_t last_draw = 0;
  while (!g_stop) {
    sbus.update();
    uint64_t now = microsNow();
    if (now - last_draw > 200000) {
      last_draw = now;
      printf("\033[2J\033[H");
      printf("sbus_probe --lib   (real SbusHandler; Ctrl-C to quit)\n");
      printf("============================================\n");
      printf("isControllerConnected (RC_CONNECTED): %s\n",
             sbus.isControllerConnected() ? "TRUE" : "FALSE");
      printf("isFrameValid       (RC_FRAME_VALID): %s\n",
             sbus.isFrameValid() ? "TRUE" : "FALSE");
      printf("failsafe=%s frame_lost=%s\n",
             sbus.isFailsafe() ? "YES" : "no",
             sbus.isFrameLost() ? "YES" : "no");
      printf("frames=%lu errors=%lu rate=%.1f Hz\n",
             sbus.getFramesReceived(), sbus.getFrameErrors(),
             sbus.getFrameRate());
      printf("consec good=%lu losses=%lu  since last frame=%.1f ms\n",
             sbus.getConsecutiveGoodFrames(),
             sbus.getConsecutiveFrameLosses(),
             sbus.getTimeSinceLastFrame() / 1000.0);
      printf("\n  CH   RAW\n");
      for (int i = 0; i < SBUS_NUM_CHANNELS; i++)
        printf("  %-4d %u\n", i + 1, sbus.getChannel(i));
      fflush(stdout);
    }
    usleep(1000);
  }
  return 0;
}

//---------------------------------------------------------------
static void usage(const char *prog) {
  printf("Usage: %s [options]\n", prog);
  printf("SBUS / RC-link diagnostic for the BlueBoat RC chain.\n\n");
  printf("Options:\n");
  printf("  -d, --device DEV   Serial device (default: %s)\n", SBUS_UART_DEV);
  printf("  -t, --time SEC     Capture duration for diag/record (default 5)\n");
  printf("      --scan         Sweep all candidate UARTs, report which has SBUS\n");
  printf("      --raw          Live raw hex dump with gap markers\n");
  printf("      --frames       Live decoded channel view (instrumented parser)\n");
  printf("      --lib          Live view through the real SbusHandler class\n");
  printf("      --record FILE  Record timestamped raw bytes to FILE\n");
  printf("      --replay FILE  Re-analyze a --record capture (works off-vehicle)\n");
  printf("      --force        Run even if another process holds the port\n");
  printf("  -h, --help         This text\n\n");
  printf("Run with iRCReader/MOOS stopped -- concurrent readers on a tty\n");
  printf("split the byte stream and both see corrupt frames.\n");
}

int main(int argc, char **argv) {
  std::string dev = SBUS_UART_DEV;
  double duration = 5.0;
  bool mode_raw = false, mode_frames = false, mode_lib = false, force = false;
  bool mode_scan = false;
  std::string record_file, replay_file;

  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    if ((a == "-d" || a == "--device") && i + 1 < argc) dev = argv[++i];
    else if ((a == "-t" || a == "--time") && i + 1 < argc) duration = atof(argv[++i]);
    else if (a == "--scan") mode_scan = true;
    else if (a == "--raw") mode_raw = true;
    else if (a == "--frames") mode_frames = true;
    else if (a == "--lib") mode_lib = true;
    else if (a == "--record" && i + 1 < argc) record_file = argv[++i];
    else if (a == "--replay" && i + 1 < argc) replay_file = argv[++i];
    else if (a == "--force") force = true;
    else if (a == "-h" || a == "--help") { usage(argv[0]); return 0; }
    else { printf("Unknown option: %s\n", a.c_str()); usage(argv[0]); return 1; }
  }

  signal(SIGINT, onSigint);

  if (!replay_file.empty()) {
    ProbeStats stats;
    return runReplay(replay_file, stats);
  }

  if (mode_scan) return runScan(3.0);

  printf("sbus_probe: device=%s\n", dev.c_str());
  printPiUartHints(dev);

  // Warn (or stop) if another process already reads this port.
  std::vector<PortHolder> holders = findPortHolders(dev);
  if (!holders.empty()) {
    printf("\nWARNING: %s is already open by:\n", dev.c_str());
    for (const auto &h : holders)
      printf("  pid %d (%s)\n", h.pid, h.comm.c_str());
    printf("Concurrent readers steal bytes from each other; results\n"
           "will be garbage. Stop iRCReader/MOOS first");
    if (!force) {
      printf(", or re-run with --force.\n");
      return 1;
    }
    printf(". Continuing (--force).\n");
  }
  printf("\n");

  if (mode_lib) return runLibMode(dev);

  int fd = openSbusPort(dev, true);
  if (fd < 0) return 1;

  FILE *rec = nullptr;
  if (!record_file.empty()) {
    rec = fopen(record_file.c_str(), "w");
    if (!rec) {
      printf("Cannot open record file %s: %s\n", record_file.c_str(), strerror(errno));
      close(fd);
      return 1;
    }
    printf("Recording to %s for %.0fs ...\n", record_file.c_str(), duration);
  }

  ProbeStats stats;
  ProbeParser parser(stats);

  bool timed = !mode_raw && !mode_frames;   // diag + record are timed
  uint64_t t_start = microsNow();
  uint64_t t_end = t_start + (uint64_t)(duration * 1e6);
  uint64_t last_draw = 0;
  uint64_t last_raw_byte_us = 0;

  if (timed && rec == nullptr)
    printf("[3] Listening for %.0fs -- move the sticks around ...\n", duration);

  while (!g_stop) {
    uint64_t now = microsNow();
    if (timed && now >= t_end) break;

    if (waitReadable(fd, 100)) {
      uint8_t buf[256];
      ssize_t n = read(fd, buf, sizeof(buf));
      now = microsNow();
      if (n > 0) {
        parser.feed(buf, n, now);

        if (rec) {
          fprintf(rec, "%llu", (unsigned long long)(now - t_start));
          for (ssize_t i = 0; i < n; i++) fprintf(rec, " %02X", buf[i]);
          fprintf(rec, "\n");
        }

        if (mode_raw) {
          if (last_raw_byte_us != 0 && now - last_raw_byte_us > 1000)
            printf("\n[gap %.1f ms] ", (now - last_raw_byte_us) / 1000.0);
          last_raw_byte_us = now;
          for (ssize_t i = 0; i < n; i++) {
            // start bytes stand out to make frame cadence visible
            if (buf[i] == SBUS_START_BYTE) printf("\n0F ");
            else printf("%02X ", buf[i]);
          }
          fflush(stdout);
        }
      } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
        printf("read error: %s\n", strerror(errno));
        break;
      }
    } else {
      // select timeout: still let the parser age its staleness state
      parser.feed(nullptr, 0, microsNow());
    }

    if (mode_frames && microsNow() - last_draw > 100000) {
      last_draw = microsNow();
      printLiveFrame(stats);
    }
  }

  double seconds = (microsNow() - t_start) / 1e6;
  if (rec) {
    fclose(rec);
    printf("Recorded %llu bytes over %.1fs to %s\n",
           (unsigned long long)stats.bytes_total, seconds, record_file.c_str());
  }
  close(fd);

  if (mode_raw || mode_frames) printf("\n");
  printVerdict(stats, seconds > 0 ? seconds : 1.0);
  return 0;
}
