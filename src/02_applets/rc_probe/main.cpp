/*************************************************************
 * rc_probe - RC link diagnostic utility (SBUS + CRSF)
 *
 * Standalone CLI tool (no MOOS required) for debugging the
 * transmitter -> receiver -> UART -> MOOS chain, for either
 * protocol.
 *
 * Supersedes sbus_probe. Two things are new and are the reason
 * this tool exists:
 *
 *   1. Protocol auto-detection. During a mixed-fleet migration
 *      the first field question is "what is this receiver
 *      actually emitting?" --protocol auto answers it by trying
 *      both line configs and reporting which one frames up.
 *
 *   2. It drives the REAL CrsfParser from lib_crsf rather than a
 *      private reimplementation. sbus_probe had to reimplement
 *      the SBUS parser because SbusHandler welds parsing to
 *      read(), leaving the repo with two SBUS parsers that can
 *      disagree. lib_crsf separates parsing from I/O precisely
 *      so the probe, the app, and the unit tests all exercise
 *      one implementation.
 *
 * Line settings:
 *   SBUS  100000 baud, 8E2, (usually) non-inverted on RadioLink
 *         and ELRS receivers. Needs a parity-capable UART: the
 *         Pi mini UART (ttyS0) CANNOT frame it.
 *   CRSF  420000 baud, 8N1. No parity requirement, so any Pi
 *         UART can carry it - the mini-UART restriction that
 *         governs SBUS does not apply.
 *
 * Modes:
 *   (default)      timed capture + automatic verdict
 *   --scan         sweep candidate ports, both protocols
 *   --frames       live decoded channel display
 *   --link         live CRSF link statistics (CRSF only)
 *   --record FILE  capture timestamped raw bytes
 *   --replay FILE  re-analyze a capture off-vehicle
 *
 * NOTE: run with iRCReader / iRCInterface STOPPED. Two processes
 * reading one tty split the byte stream and both see corruption.
 * The tool detects other openers and refuses unless --force.
 *
 * Author: Jeremy Wenger
 *************************************************************/

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <cerrno>
#include <string>
#include <vector>
#include <algorithm>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <dirent.h>
#include <signal.h>
#include <time.h>

#include "crsf_parser.h"
#include "crsf_frames.h"
#include "sbus_handler.h"

// Linux custom-baud plumbing (same values sbus_handler.cpp uses).
#define PROBE_BOTHER  0010000
#define PROBE_TCGETS2 0x802C542A
#define PROBE_TCSETS2 0x402C542B

#define CRSF_BAUDRATE 420000

// Inter-byte gap that marks a frame boundary for SBUS resync.
static const uint32_t FRAME_GAP_THRESHOLD_US = 3000;

enum Protocol { PROTO_AUTO = 0, PROTO_SBUS, PROTO_CRSF };

static const char* protoName(Protocol p) {
  switch (p) {
    case PROTO_SBUS: return "SBUS";
    case PROTO_CRSF: return "CRSF";
    default:         return "auto";
  }
}

static volatile sig_atomic_t g_stop = 0;
static void onSigint(int) { g_stop = 1; }

static int g_validated_channels = 12;   // ELRS drives 12 of 16

static uint64_t microsNow() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1000000ull + (uint64_t)ts.tv_nsec / 1000ull;
}

//---------------------------------------------------------------
// Port holders: two readers on one tty is the single most common
// self-inflicted cause of "corrupt frames".

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
    if (!fds) continue;

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
// Pi UART hints. The mini-UART parity warning applies to SBUS
// only -- CRSF is 8N1 and frames fine on ttyS0.

static void printPiUartHints(const std::string &dev, Protocol proto) {
#ifdef __linux__
  FILE *mf = fopen("/proc/device-tree/model", "r");
  if (!mf) return;
  char model[128] = {0};
  size_t got = fread(model, 1, sizeof(model) - 1, mf);
  model[got] = '\0';
  fclose(mf);
  if (strstr(model, "Raspberry Pi") == nullptr) return;

  printf("  Platform: %s\n", model);

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
    if (proto == PROTO_CRSF) {
      printf("  NOTE: %s is the Pi mini UART. That is FINE for CRSF\n"
             "        (8N1, no parity needed) but would break SBUS.\n",
             dev.c_str());
    } else {
      printf("  WARNING: %s is the Pi *mini UART* on most Pi models.\n"
             "           No parity support; SBUS is 8E2 and cannot be\n"
             "           framed reliably on it. Use /dev/ttyAMA*.\n",
             dev.c_str());
    }
  }
#else
  (void)dev; (void)proto;
#endif
}

//---------------------------------------------------------------
// Port open + configure, parameterised by protocol.
//
// Opened O_RDWR (not O_RDONLY as sbus_probe used) because CRSF
// telemetry needs the TX direction; --link and future telemetry
// tests write on this same fd.

static int openRcPort(const std::string &dev, Protocol proto, bool verbose) {
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

  int fd = open(dev.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    printf("  FAIL: open(): %s\n", strerror(errno));
    if (errno == EACCES)
      printf("  -> Permission denied: run as root or join 'dialout'.\n");
    if (errno == EBUSY)
      printf("  -> Device busy: another process holds it exclusively.\n");
    return -1;
  }
  if (verbose) printf("  OK: opened (fd=%d)\n", fd);

  const bool is_crsf = (proto == PROTO_CRSF);
  const unsigned baud = is_crsf ? CRSF_BAUDRATE : SBUS_BAUDRATE;

  if (verbose)
    printf("[2] Configuring %u baud, %s, raw ...\n",
           baud, is_crsf ? "8N1" : "8E2");

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
  tio.c_cflag |= (CLOCAL | CREAD);

  if (is_crsf) {
    tio.c_cflag &= ~PARENB;    // 8N1: no parity
    tio.c_cflag &= ~CSTOPB;    //      one stop bit
  } else {
    tio.c_cflag |= PARENB;     // 8E2: even parity
    tio.c_cflag &= ~PARODD;
    tio.c_cflag |= CSTOPB;     //      two stop bits
  }

#ifdef CRTSCTS
  tio.c_cflag &= ~CRTSCTS;
#endif
  tio.c_iflag &= ~(IXON | IXOFF | IXANY);
  tio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tio.c_iflag &= ~(INLCR | ICRNL);
  // Do NOT set INPCK: bytes with parity errors are diagnostic
  // signal (misalignment / inverted line), not noise to suppress.
  tio.c_iflag &= ~(INPCK | ISTRIP | IGNBRK | BRKINT | PARMRK);

  // Output post-processing off. Harmless for read-only capture,
  // but this port is opened O_RDWR for the telemetry path, and
  // with OPOST/ONLCR set the driver rewrites any 0x0A byte to
  // 0x0D 0x0A on write -- which silently corrupts exactly those
  // frames whose length byte happens to be 0x0A.
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
    printf("  FAIL: tcsetattr(): %s\n", strerror(errno));
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
    printf("  FAIL: TCGETS2 ioctl: %s\n", strerror(errno));
    printf("  -> Custom baud unavailable on this driver/OS.\n");
    close(fd);
    return -1;
  }
  tio2.c_cflag &= ~0000017;   // clear CBAUD
  tio2.c_cflag |= PROBE_BOTHER;
  tio2.c_ispeed = baud;
  tio2.c_ospeed = baud;
  if (ioctl(fd, PROBE_TCSETS2, &tio2) < 0) {
    printf("  FAIL: TCSETS2 ioctl: %s\n", strerror(errno));
    close(fd);
    return -1;
  }

  // Read back: some drivers silently drop parity or clamp baud.
  struct custom_termios2 chk;
  if (ioctl(fd, PROBE_TCGETS2, &chk) == 0) {
    bool baud_ok = (chk.c_ispeed == baud);
    if (is_crsf) {
      bool no_parity = !(chk.c_cflag & PARENB);
      bool one_stop  = !(chk.c_cflag & CSTOPB);
      if (verbose)
        printf("  Readback: baud=%lu (%s)  no-parity=%s  1-stop=%s\n",
               (unsigned long)chk.c_ispeed, baud_ok ? "OK" : "MISMATCH",
               no_parity ? "OK" : "UNEXPECTED",
               one_stop  ? "OK" : "UNEXPECTED");
      if (!baud_ok)
        printf("  WARNING: driver did not accept %u baud -- CRSF will\n"
               "           not frame up.\n", CRSF_BAUDRATE);
    } else {
      bool parity_ok = (chk.c_cflag & PARENB) && !(chk.c_cflag & PARODD);
      bool stop_ok   = (chk.c_cflag & CSTOPB);
      if (verbose)
        printf("  Readback: baud=%lu (%s)  even-parity=%s  2-stop=%s\n",
               (unsigned long)chk.c_ispeed, baud_ok ? "OK" : "MISMATCH",
               parity_ok ? "OK" : "MISSING",
               stop_ok   ? "OK" : "MISSING");
      if (!baud_ok || !parity_ok || !stop_ok)
        printf("  WARNING: driver did not accept the full SBUS line\n"
               "           config -- frames will likely be corrupt.\n");
    }
  }
#endif

  tcflush(fd, TCIFLUSH);
  if (verbose) printf("  OK: port configured\n");
  return fd;
}

//---------------------------------------------------------------
// Statistics, shared shape across both protocols so the verdict
// logic can be written once.

struct ProbeStats {
  Protocol proto = PROTO_AUTO;

  // Byte level
  uint64_t bytes_total = 0;
  uint64_t byte_hist[256] = {0};
  uint64_t reads_returning_data = 0;

  // Frame level (both protocols)
  uint64_t frames_valid = 0;
  uint64_t frames_rejected = 0;

  // SBUS-specific rejection reasons
  uint64_t sbus_bad_header = 0;
  uint64_t sbus_bad_footer = 0;
  uint64_t sbus_out_of_range = 0;
  uint64_t sbus_range_violations[SBUS_NUM_CHANNELS] = {0};

  // CRSF-specific
  uint64_t crsf_crc_fail = 0;
  uint64_t crsf_bad_length = 0;
  uint64_t crsf_bytes_discarded = 0;
  uint64_t crsf_type_hist[256] = {0};

  // Receiver-reported link health
  uint64_t frames_failsafe = 0;
  uint64_t frames_lost_flag = 0;

  // Last decoded state
  uint16_t channels[16] = {0};
  bool have_channels = false;

  CrsfLinkStats link = {};
  bool have_link = false;

  // Timing
  uint64_t first_byte_us = 0;
  uint64_t last_frame_us = 0;
  uint64_t max_frame_gap_us = 0;
};

//---------------------------------------------------------------
// CRSF byte pump: feeds the REAL parser, then decodes payloads.

static void crsfConsume(CrsfParser &parser, const uint8_t *buf, size_t n,
                        ProbeStats &s, uint64_t now_us,
                        bool print_frames, bool print_link)
{
  std::vector<CrsfFrame> frames;
  parser.feed(buf, n, frames);

  for (const auto &f : frames) {
    s.frames_valid++;
    s.crsf_type_hist[f.type]++;

    if (s.last_frame_us != 0) {
      uint64_t gap = now_us - s.last_frame_us;
      if (gap > s.max_frame_gap_us) s.max_frame_gap_us = gap;
    }
    s.last_frame_us = now_us;

    if (f.type == CRSF_FRAMETYPE_RC_CHANNELS) {
      CrsfChannels ch;
      if (crsfDecodeChannels(f.payload, ch)) {
        for (int i = 0; i < 16; i++) s.channels[i] = ch.channel[i];
        s.have_channels = true;

        if (print_frames) {
          printf("CH:");
          for (int i = 0; i < 16; i++) printf(" %4u", ch.channel[i]);
          printf("\n");
          fflush(stdout);
        }
      }
    } else if (f.type == CRSF_FRAMETYPE_LINK_STATISTICS) {
      CrsfLinkStats ls;
      if (crsfDecodeLinkStats(f.payload, ls)) {
        s.link = ls;
        s.have_link = true;

        // ELRS reports uplink LQ 0 when the link is down. This is
        // the direct signal SBUS never had - no inference needed.
        if (ls.uplink_lq == 0) s.frames_failsafe++;

        if (print_link) {
          printf("LQ up=%3u%% dn=%3u%%  RSSI up=%4d dn=%4d dBm  "
                 "SNR up=%+3d dn=%+3d dB  ant=%u rf=%u pwr=%umW\n",
                 ls.uplink_lq, ls.downlink_lq,
                 ls.uplink_rssi_ant1, ls.downlink_rssi,
                 ls.uplink_snr, ls.downlink_snr,
                 ls.active_antenna, ls.rf_mode,
                 crsfTxPowerMilliwatts(ls.uplink_tx_power_idx));
          fflush(stdout);
        }
      }
    }
  }

  // Mirror the parser's own counters into the stats block.
  s.crsf_crc_fail        = parser.framesCrcFail();
  s.crsf_bad_length      = parser.framesBadLength();
  s.crsf_bytes_discarded = parser.bytesDiscarded();
  s.frames_rejected      = s.crsf_crc_fail + s.crsf_bad_length;
}

//---------------------------------------------------------------
// SBUS byte pump: gap-based resync + decode, instrumented to
// record WHY each frame was rejected.

struct SbusAssembler {
  uint8_t frame[SBUS_FRAME_SIZE];
  int idx = 0;
  bool in_frame = false;
  uint64_t last_byte_us = 0;
};

static void sbusConsume(SbusAssembler &a, const uint8_t *buf, size_t n,
                        ProbeStats &s, uint64_t now_us, bool print_frames)
{
  for (size_t i = 0; i < n; i++) {
    uint8_t b = buf[i];

    uint64_t gap = (a.last_byte_us == 0) ? 0 : (now_us - a.last_byte_us);
    a.last_byte_us = now_us;

    if (!a.in_frame) {
      if (b == SBUS_START_BYTE && (gap > FRAME_GAP_THRESHOLD_US || gap == 0)) {
        a.in_frame = true;
        a.idx = 0;
        a.frame[a.idx++] = b;
      } else if (b == SBUS_START_BYTE) {
        a.in_frame = true;
        a.idx = 0;
        a.frame[a.idx++] = b;
      }
      continue;
    }

    a.frame[a.idx++] = b;
    if (a.idx < SBUS_FRAME_SIZE) continue;

    a.in_frame = false;

    // --- full 25-byte candidate ---
    if (a.frame[0] != SBUS_START_BYTE) {
      s.sbus_bad_header++;
      s.frames_rejected++;
      continue;
    }
    uint8_t footer = a.frame[SBUS_FOOTER_OFFSET];
    if (footer != SBUS_END_BYTE && (footer & 0x0F) != 0x04) {
      s.sbus_bad_footer++;
      s.frames_rejected++;
      continue;
    }

    uint16_t ch[SBUS_NUM_CHANNELS];
    const uint8_t *f = a.frame;
    ch[0]  = ((f[1]       | (f[2]  << 8)) & 0x07FF);
    ch[1]  = ((f[2]  >> 3) | (f[3]  << 5)) & 0x07FF;
    ch[2]  = ((f[3]  >> 6) | (f[4]  << 2) | (f[5]  << 10)) & 0x07FF;
    ch[3]  = ((f[5]  >> 1) | (f[6]  << 7)) & 0x07FF;
    ch[4]  = ((f[6]  >> 4) | (f[7]  << 4)) & 0x07FF;
    ch[5]  = ((f[7]  >> 7) | (f[8]  << 1) | (f[9]  << 9)) & 0x07FF;
    ch[6]  = ((f[9]  >> 2) | (f[10] << 6)) & 0x07FF;
    ch[7]  = ((f[10] >> 5) | (f[11] << 3)) & 0x07FF;
    ch[8]  = ((f[12]      | (f[13] << 8)) & 0x07FF);
    ch[9]  = ((f[13] >> 3) | (f[14] << 5)) & 0x07FF;
    ch[10] = ((f[14] >> 6) | (f[15] << 2) | (f[16] << 10)) & 0x07FF;
    ch[11] = ((f[16] >> 1) | (f[17] << 7)) & 0x07FF;
    ch[12] = ((f[17] >> 4) | (f[18] << 4)) & 0x07FF;
    ch[13] = ((f[18] >> 7) | (f[19] << 1) | (f[20] << 9)) & 0x07FF;
    ch[14] = ((f[20] >> 2) | (f[21] << 6)) & 0x07FF;
    ch[15] = ((f[21] >> 5) | (f[22] << 3)) & 0x07FF;

    bool range_ok = true;
    for (int c = 0; c < g_validated_channels && c < SBUS_NUM_CHANNELS; c++) {
      if (ch[c] < SBUS_MIN_VALUE || ch[c] > SBUS_MAX_VALUE) {
        s.sbus_range_violations[c]++;
        range_ok = false;
      }
    }
    if (!range_ok) {
      s.sbus_out_of_range++;
      s.frames_rejected++;
      continue;
    }

    uint8_t flags = a.frame[SBUS_FLAGS_OFFSET];
    if (flags & SBUS_FLAG_FAILSAFE)   s.frames_failsafe++;
    if (flags & SBUS_FLAG_FRAME_LOST) s.frames_lost_flag++;

    for (int c = 0; c < SBUS_NUM_CHANNELS; c++) s.channels[c] = ch[c];
    s.have_channels = true;
    s.frames_valid++;

    if (s.last_frame_us != 0) {
      uint64_t g = now_us - s.last_frame_us;
      if (g > s.max_frame_gap_us) s.max_frame_gap_us = g;
    }
    s.last_frame_us = now_us;

    if (print_frames) {
      printf("CH:");
      for (int c = 0; c < 16; c++) printf(" %4u", ch[c]);
      printf("  %s%s\n",
             (flags & SBUS_FLAG_FAILSAFE) ? "[FAILSAFE]" : "",
             (flags & SBUS_FLAG_FRAME_LOST) ? "[LOST]" : "");
      fflush(stdout);
    }
  }
}

//---------------------------------------------------------------
// Wait for readable data.

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
// Capture loop, shared by both protocols.

static void captureFor(int fd, Protocol proto, double seconds,
                       ProbeStats &s, FILE *rec,
                       bool print_frames, bool print_link)
{
  CrsfParser parser;
  SbusAssembler asm_;

  s.proto = proto;
  const uint64_t t0 = microsNow();
  const uint64_t t_end = t0 + (uint64_t)(seconds * 1e6);

  uint8_t buf[512];
  while (!g_stop && microsNow() < t_end) {
    if (!waitReadable(fd, 100)) continue;

    ssize_t n = read(fd, buf, sizeof(buf));
    if (n <= 0) continue;

    const uint64_t now = microsNow();
    if (s.first_byte_us == 0) s.first_byte_us = now;
    s.reads_returning_data++;
    s.bytes_total += (uint64_t)n;
    for (ssize_t i = 0; i < n; i++) s.byte_hist[buf[i]]++;

    if (rec) {
      fprintf(rec, "%llu", (unsigned long long)(now - t0));
      for (ssize_t i = 0; i < n; i++) fprintf(rec, " %02X", buf[i]);
      fprintf(rec, "\n");
    }

    if (proto == PROTO_CRSF)
      crsfConsume(parser, buf, (size_t)n, s, now, print_frames, print_link);
    else
      sbusConsume(asm_, buf, (size_t)n, s, now, print_frames);
  }
}

//---------------------------------------------------------------
// Verdict.

static void printVerdict(const ProbeStats &s, double seconds) {
  printf("\n=====================================================\n");
  printf(" VERDICT (%s, %.1f s)\n", protoName(s.proto), seconds);
  printf("=====================================================\n");

  printf("  Bytes received:   %llu (%.0f B/s)\n",
         (unsigned long long)s.bytes_total,
         seconds > 0 ? s.bytes_total / seconds : 0.0);
  printf("  Frames valid:     %llu (%.1f Hz)\n",
         (unsigned long long)s.frames_valid,
         seconds > 0 ? s.frames_valid / seconds : 0.0);
  printf("  Frames rejected:  %llu\n",
         (unsigned long long)s.frames_rejected);

  if (s.proto == PROTO_CRSF) {
    printf("  CRC failures:     %llu\n", (unsigned long long)s.crsf_crc_fail);
    printf("  Bad length:       %llu\n", (unsigned long long)s.crsf_bad_length);
    printf("  Bytes discarded:  %llu\n",
           (unsigned long long)s.crsf_bytes_discarded);

    bool any_types = false;
    for (int t = 0; t < 256; t++) if (s.crsf_type_hist[t]) any_types = true;
    if (any_types) {
      printf("\n  Frame types seen:\n");
      for (int t = 0; t < 256; t++) {
        if (!s.crsf_type_hist[t]) continue;
        const char *nm = "";
        switch (t) {
          case CRSF_FRAMETYPE_GPS:             nm = " (GPS)"; break;
          case CRSF_FRAMETYPE_BATTERY_SENSOR:  nm = " (battery)"; break;
          case CRSF_FRAMETYPE_LINK_STATISTICS: nm = " (link stats)"; break;
          case CRSF_FRAMETYPE_RC_CHANNELS:     nm = " (RC channels)"; break;
          case CRSF_FRAMETYPE_ATTITUDE:        nm = " (attitude)"; break;
          case CRSF_FRAMETYPE_FLIGHT_MODE:     nm = " (flight mode)"; break;
          default: break;
        }
        printf("    0x%02X%-16s %llu\n", t, nm,
               (unsigned long long)s.crsf_type_hist[t]);
      }
    }
  } else {
    printf("  Bad header:       %llu\n", (unsigned long long)s.sbus_bad_header);
    printf("  Bad footer:       %llu\n", (unsigned long long)s.sbus_bad_footer);
    printf("  Out of range:     %llu\n", (unsigned long long)s.sbus_out_of_range);
  }

  if (s.have_link) {
    printf("\n  Link statistics (from receiver):\n");
    printf("    Uplink   LQ %3u%%  RSSI %4d dBm  SNR %+3d dB\n",
           s.link.uplink_lq, s.link.uplink_rssi_ant1, s.link.uplink_snr);
    printf("    Downlink LQ %3u%%  RSSI %4d dBm  SNR %+3d dB"
           "  (always 0 on ELRS: debug-only fields; see handset TQly)\n",
           s.link.downlink_lq, s.link.downlink_rssi, s.link.downlink_snr);
    printf("    TX power %u mW   RF mode %u   active antenna %u\n",
           crsfTxPowerMilliwatts(s.link.uplink_tx_power_idx),
           s.link.rf_mode, s.link.active_antenna);
  }

  if (s.have_channels) {
    printf("\n  Last channels:\n   ");
    for (int i = 0; i < 16; i++) {
      printf(" %4u", s.channels[i]);
      if (i == 7) printf("\n   ");
    }
    printf("\n");
  }

  printf("\n-----------------------------------------------------\n");

  // ---- Layered diagnosis, most fundamental failure first ----

  if (s.bytes_total == 0) {
    printf(" NO DATA -- nothing arrived on the wire.\n\n"
           " Layer: receiver or wiring.\n"
           "  * Receiver output mode: is it emitting %s at all?\n"
           "    ELRS: set the output protocol in the ELRS Lua script\n"
           "    or the receiver WebUI (Model tab).\n"
           "  * Signal pin on the UART RX pin, common ground, power.\n"
           "  * Is the UART overlay enabled for this connector?\n"
           "    Navigator: uart3->ttyAMA1 GPIO4/5, uart4->ttyAMA2\n"
           "    GPIO8/9, uart5->ttyAMA3 GPIO12/13.\n",
           protoName(s.proto));
    return;
  }

  if (s.frames_valid == 0) {
    printf(" DATA BUT NEVER FRAMES UP -- bytes arrive, none parse.\n\n"
           " Layer: line configuration or wrong protocol.\n");

    uint64_t zeros = s.byte_hist[0x00];
    uint64_t ffs   = s.byte_hist[0xFF];
    double zf = 100.0 * (double)(zeros + ffs) / (double)s.bytes_total;
    printf("  * 0x00/0xFF make up %.0f%% of bytes.\n", zf);
    if (zf > 50.0)
      printf("    >50%% suggests an INVERTED line or a badly wrong baud.\n");

    if (s.proto == PROTO_CRSF) {
      printf("  * Wrong baud is the usual cause. CRSF is 420000 8N1.\n"
             "  * Is the receiver actually in CRSF mode? Try:\n"
             "      rc_probe --protocol sbus -d <dev>\n"
             "    If SBUS frames up, the receiver is still in SBUS mode.\n");
    } else {
      printf("  * SBUS is 100000 8E2 and needs a parity-capable UART.\n"
             "    On a Pi that means PL011 (/dev/ttyAMA*), never ttyS0.\n"
             "  * Is the receiver actually in SBUS mode? Try:\n"
             "      rc_probe --protocol crsf -d <dev>\n");
    }
    printf("  * Or run: rc_probe --protocol auto -d <dev>\n");
    return;
  }

  const double reject_rate = (s.frames_valid + s.frames_rejected) > 0
    ? 100.0 * (double)s.frames_rejected /
      (double)(s.frames_valid + s.frames_rejected)
    : 0.0;

  if (s.proto == PROTO_CRSF && s.crsf_crc_fail > 0 && reject_rate > 5.0) {
    printf(" CRC FAILURES (%.0f%% of frames)\n\n"
           " Layer: physical / electrical.\n"
           "  * CRSF has a real checksum, so this is a definite\n"
           "    wire-level fault, not a decode guess: noise, a\n"
           "    marginal ground, an over-long cable, or contention\n"
           "    from another process on this tty.\n"
           "  * A healthy wired CRSF link should sit at 0.\n",
           reject_rate);
    return;
  }

  if (s.proto == PROTO_SBUS && s.sbus_out_of_range > 0 && reject_rate > 5.0) {
    printf(" CHANNEL VALUES OUT OF RANGE (%.0f%% of frames)\n\n"
           " Layer: decode.\n", reject_rate);
    printf("  Violating channels (of the first %d validated):\n",
           g_validated_channels);
    for (int c = 0; c < SBUS_NUM_CHANNELS; c++) {
      if (s.sbus_range_violations[c])
        printf("    CH%-2d  %llu times\n", c + 1,
               (unsigned long long)s.sbus_range_violations[c]);
    }
    printf("  * Undriven channels sit at 0. ELRS drives 12 of 16, so\n"
           "    use --channels 12 (this is what validated_channels in\n"
           "    the mission plug is for). CRSF has no such problem:\n"
           "    the CRC does this job properly.\n");
    return;
  }

  if (s.proto == PROTO_SBUS && s.sbus_bad_footer > 0 && reject_rate > 5.0) {
    printf(" FOOTER FAILURES (%.0f%% of frames)\n\n"
           " Layer: framing/misalignment -- wrong baud or parity, or\n"
           " the pin carries a different protocol (i-BUS/PPM).\n",
           reject_rate);
    return;
  }

  // Receiver-reported link problems. This is the case that looks
  // healthy at the byte layer and is easy to misread.
  if (s.frames_failsafe > 0) {
    // Denominator differs by protocol: SBUS carries the failsafe
    // flag in every frame, while CRSF reports it via link-stats
    // frames only (a fraction of the total). Dividing by all
    // frames in the CRSF case would understate a total link loss
    // as a single-digit percentage.
    const uint64_t denom = (s.proto == PROTO_CRSF)
      ? s.crsf_type_hist[CRSF_FRAMETYPE_LINK_STATISTICS]
      : s.frames_valid;
    double fs = denom ? 100.0 * (double)s.frames_failsafe / (double)denom : 0.0;
    printf(" RECEIVER REPORTS FAILSAFE (%.0f%% of %s)\n\n",
           fs, (s.proto == PROTO_CRSF) ? "link-stat frames" : "frames");
    printf("%s",
           " Layer: RF link, NOT the serial chain.\n"
           "  * Frames are arriving cleanly -- the serial side is\n"
           "    fine. The receiver itself says it has no valid\n"
           "    uplink from the handset.\n"
           "  * Check: transmitter powered and bound, model match,\n"
           "    antennas, distance, 2.4 GHz interference.\n"
           "  * Note this can persist with a PERFECT frame rate and\n"
           "    zero frame loss, which reads as 'healthy' at every\n"
           "    other layer.\n");
    if (s.have_link)
      printf("  * Uplink LQ is %u%% -- CRSF states this directly\n"
             "    instead of leaving it to be inferred.\n",
             s.link.uplink_lq);
    return;
  }

  if (s.frames_lost_flag > 0) {
    double fl = 100.0 * (double)s.frames_lost_flag / (double)s.frames_valid;
    printf(" RECEIVER REPORTS FRAME LOSS (%.0f%% of frames)\n\n"
           " Layer: RF link, milder. Antennas, range, interference.\n", fl);
    return;
  }

  if (s.have_link && s.link.uplink_lq < 100) {
    printf(" LINK QUALITY BELOW 100%% (uplink LQ %u%%)\n\n"
           " Layer: RF. Serial chain is healthy. Some degradation is\n"
           " normal at range; sustained low LQ is not.\n", s.link.uplink_lq);
    return;
  }

  if (reject_rate > 1.0) {
    printf(" MOSTLY HEALTHY, %.1f%% of frames rejected.\n\n"
           " Watch for intermittent wire-level trouble.\n", reject_rate);
    return;
  }

  printf(" HEALTHY -- %s frames decode cleanly.\n\n",
         protoName(s.proto));
  printf("  The serial chain is good. If the MOOS app still reports\n"
         "  a problem, check that it opens THIS device, and inspect\n"
         "  its appcast for config warnings.\n");
}

//---------------------------------------------------------------
// Auto-detect: try each protocol briefly, keep the better result.

static Protocol autoDetect(const std::string &dev, double per_proto_s) {
  printf("Auto-detecting protocol on %s (%.1fs each) ...\n\n",
         dev.c_str(), per_proto_s);

  Protocol best = PROTO_AUTO;
  uint64_t best_frames = 0;

  for (Protocol p : {PROTO_CRSF, PROTO_SBUS}) {
    int fd = openRcPort(dev, p, false);
    if (fd < 0) continue;

    ProbeStats s;
    captureFor(fd, p, per_proto_s, s, nullptr, false, false);
    close(fd);

    printf("  %-5s : %6llu bytes, %5llu valid frames",
           protoName(p),
           (unsigned long long)s.bytes_total,
           (unsigned long long)s.frames_valid);
    if (p == PROTO_CRSF && s.crsf_crc_fail)
      printf(", %llu CRC fail", (unsigned long long)s.crsf_crc_fail);
    printf("\n");

    if (s.frames_valid > best_frames) {
      best_frames = s.frames_valid;
      best = p;
    }
    if (g_stop) break;
  }

  printf("\n");
  if (best == PROTO_AUTO || best_frames == 0) {
    printf("  No protocol framed up. Falling back to CRSF for the\n"
           "  full diagnosis (use --protocol to force the other).\n\n");
    return PROTO_CRSF;
  }
  printf("  Detected: %s\n\n", protoName(best));
  return best;
}

//---------------------------------------------------------------
// Replay a --record capture.

static int runReplay(const std::string &file, Protocol proto) {
  FILE *f = fopen(file.c_str(), "r");
  if (!f) {
    printf("FAIL: cannot open %s: %s\n", file.c_str(), strerror(errno));
    return 1;
  }

  ProbeStats s;
  s.proto = proto;
  CrsfParser parser;
  SbusAssembler asm_;

  char line[8192];
  uint64_t first_us = 0, last_us = 0;

  while (fgets(line, sizeof(line), f)) {
    char *p = line;
    unsigned long long ts = strtoull(p, &p, 10);
    if (first_us == 0) first_us = ts;
    last_us = ts;

    std::vector<uint8_t> bytes;
    while (*p) {
      while (*p == ' ' || *p == '\t') p++;
      if (!*p || *p == '\n') break;
      char *end = nullptr;
      long v = strtol(p, &end, 16);
      if (end == p) break;
      bytes.push_back((uint8_t)v);
      p = end;
    }
    if (bytes.empty()) continue;

    if (s.first_byte_us == 0) s.first_byte_us = ts;
    s.bytes_total += bytes.size();
    for (uint8_t b : bytes) s.byte_hist[b]++;

    if (proto == PROTO_CRSF)
      crsfConsume(parser, bytes.data(), bytes.size(), s, ts, false, false);
    else
      sbusConsume(asm_, bytes.data(), bytes.size(), s, ts, false);
  }
  fclose(f);

  double secs = (last_us > first_us) ? (double)(last_us - first_us) / 1e6 : 0.0;
  printf("Replaying %s as %s\n", file.c_str(), protoName(proto));
  printVerdict(s, secs > 0 ? secs : 1.0);
  return 0;
}

//---------------------------------------------------------------
// Scan candidate ports.

static int runScan(double per_port_s) {
  std::vector<std::string> candidates;
  candidates.push_back("/dev/ttyS0");
  for (int i = 0; i <= 4; i++)
    candidates.push_back("/dev/ttyAMA" + std::to_string(i));

  printf("Scanning ports (%.1fs per protocol per port)\n", per_port_s);
  printf("=====================================================\n");

  for (const auto &dev : candidates) {
    struct stat st;
    if (stat(dev.c_str(), &st) != 0) continue;

    printf("\n%s\n", dev.c_str());

    auto holders = findPortHolders(dev);
    if (!holders.empty()) {
      printf("  SKIP: held by");
      for (const auto &h : holders)
        printf(" %s(pid %d)", h.comm.c_str(), h.pid);
      printf("\n");
      continue;
    }

    for (Protocol p : {PROTO_CRSF, PROTO_SBUS}) {
      int fd = openRcPort(dev, p, false);
      if (fd < 0) { printf("  %-5s : cannot open\n", protoName(p)); continue; }

      ProbeStats s;
      captureFor(fd, p, per_port_s, s, nullptr, false, false);
      close(fd);

      printf("  %-5s : %6llu bytes  %5llu frames",
             protoName(p),
             (unsigned long long)s.bytes_total,
             (unsigned long long)s.frames_valid);
      if (s.frames_valid > 0) printf("   <-- FRAMES UP");
      printf("\n");

      if (g_stop) return 0;
    }
  }
  printf("\nDone. Run the full diagnosis on the port that framed up:\n"
         "  rc_probe -d <dev> --protocol <sbus|crsf>\n");
  return 0;
}

//---------------------------------------------------------------

static void usage(const char *prog) {
  printf("Usage: %s [OPTIONS]\n\n", prog);
  printf("  -d, --device DEV      serial device (default /dev/ttyAMA1)\n");
  printf("  -p, --protocol P      sbus | crsf | auto (default auto)\n");
  printf("  -t, --time SEC        capture duration (default 5)\n");
  printf("      --channels N      SBUS channels to range-check (default 12)\n");
  printf("      --scan            sweep ports, both protocols\n");
  printf("      --frames          live decoded channel display\n");
  printf("      --link            live CRSF link statistics\n");
  printf("      --record FILE     record timestamped raw bytes\n");
  printf("      --replay FILE     re-analyze a recording off-vehicle\n");
  printf("      --force           run even if another process holds the port\n");
  printf("  -h, --help\n\n");
  printf("Examples:\n");
  printf("  %s -d /dev/ttyAMA1                 # auto-detect + diagnose\n", prog);
  printf("  %s --scan                          # which port, which protocol\n", prog);
  printf("  %s -p crsf --link -d /dev/ttyAMA1  # live link quality\n", prog);
  printf("  %s -p crsf --frames -d /dev/ttyAMA1  # live stick values\n", prog);
  printf("\nRun with the MOOS RC app STOPPED: two readers on one tty\n");
  printf("split the byte stream and both see corruption.\n");
}

int main(int argc, char **argv) {
  signal(SIGINT, onSigint);

  std::string dev = "/dev/ttyAMA1";
  Protocol proto = PROTO_AUTO;
  double seconds = 5.0;
  bool scan = false, frames = false, link = false, force = false;
  std::string record_file, replay_file;

  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    if      ((a == "-d" || a == "--device")   && i + 1 < argc) dev = argv[++i];
    else if ((a == "-t" || a == "--time")     && i + 1 < argc) seconds = atof(argv[++i]);
    else if ((a == "-p" || a == "--protocol") && i + 1 < argc) {
      std::string v = argv[++i];
      if      (v == "sbus") proto = PROTO_SBUS;
      else if (v == "crsf") proto = PROTO_CRSF;
      else if (v == "auto") proto = PROTO_AUTO;
      else { printf("Unknown protocol: %s\n", v.c_str()); return 1; }
    }
    else if (a == "--channels" && i + 1 < argc) g_validated_channels = atoi(argv[++i]);
    else if (a == "--scan")    scan = true;
    else if (a == "--frames")  frames = true;
    else if (a == "--link")    link = true;
    else if (a == "--force")   force = true;
    else if (a == "--record" && i + 1 < argc) record_file = argv[++i];
    else if (a == "--replay" && i + 1 < argc) replay_file = argv[++i];
    else if (a == "-h" || a == "--help") { usage(argv[0]); return 0; }
    else { printf("Unknown option: %s\n\n", a.c_str()); usage(argv[0]); return 1; }
  }

  if (!replay_file.empty()) {
    // A recording carries no line config, so the protocol must be
    // stated. Default to CRSF but say so.
    if (proto == PROTO_AUTO) {
      printf("NOTE: --replay needs an explicit --protocol; assuming crsf.\n\n");
      proto = PROTO_CRSF;
    }
    return runReplay(replay_file, proto);
  }

  if (scan) return runScan(seconds > 3.0 ? 3.0 : seconds);

  printf("rc_probe -- RC link diagnostic\n");
  printf("=====================================================\n");
  printf("Device: %s   Protocol: %s   Duration: %.1fs\n\n",
         dev.c_str(), protoName(proto), seconds);

  auto holders = findPortHolders(dev);
  if (!holders.empty()) {
    printf("WARNING: %s is already open by:\n", dev.c_str());
    for (const auto &h : holders)
      printf("   %s (pid %d)\n", h.comm.c_str(), h.pid);
    if (!force) {
      printf("\nRefusing to run: two readers on one tty split the byte\n"
             "stream and BOTH see corrupt frames. Stop the other\n"
             "process (or pass --force, but don't).\n");
      return 1;
    }
    printf("\n--force given; results are unreliable.\n\n");
  }

  printPiUartHints(dev, proto);
  printf("\n");

  if (proto == PROTO_AUTO)
    proto = autoDetect(dev, seconds > 2.0 ? 2.0 : seconds);

  int fd = openRcPort(dev, proto, true);
  if (fd < 0) return 1;

  FILE *rec = nullptr;
  if (!record_file.empty()) {
    rec = fopen(record_file.c_str(), "w");
    if (!rec) {
      printf("WARNING: cannot write %s: %s\n",
             record_file.c_str(), strerror(errno));
    } else {
      printf("Recording raw bytes to %s\n", record_file.c_str());
    }
  }

  if (frames || link) {
    printf("\nLive view -- Ctrl-C to stop.\n\n");
    ProbeStats s;
    captureFor(fd, proto, 1e9, s, rec, frames, link);
    if (rec) fclose(rec);
    close(fd);
    double secs = s.first_byte_us
      ? (double)(microsNow() - s.first_byte_us) / 1e6 : 1.0;
    printVerdict(s, secs);
    return 0;
  }

  printf("\n[3] Capturing for %.1f s ...\n", seconds);
  ProbeStats s;
  captureFor(fd, proto, seconds, s, rec, false, false);
  if (rec) fclose(rec);
  close(fd);

  printVerdict(s, seconds);
  return 0;
}
