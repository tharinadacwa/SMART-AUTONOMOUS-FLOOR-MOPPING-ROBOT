// ============================================================================
//  minibot_hardware/serial_link.hpp
//
//  POSIX termios serial link speaking the STM32's CRC-8 framed protocol.
//  Header-only, no external dependencies -- deliberately. The `serial` package
//  is a recurring source of build pain on ROS 2 and there is nothing here that
//  justifies it: this is 150 lines of termios.
//
//  WIRE FORMAT (mirrors stm32/Core/Inc/minibot_protocol.h EXACTLY)
//      $<PAYLOAD>*<CRC8_HEX><LF>
//      CRC-8/ATM, poly 0x07, init 0x00, over PAYLOAD only.
// ============================================================================

#ifndef MINIBOT_HARDWARE__SERIAL_LINK_HPP_
#define MINIBOT_HARDWARE__SERIAL_LINK_HPP_

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

namespace minibot_hardware
{

struct Feedback
{
  int32_t  left_steps{0};
  int32_t  right_steps{0};
  int32_t  left_sps{0};
  int32_t  right_sps{0};
  uint16_t flags{0};
  uint16_t seq{0};
};

class SerialLink
{
public:
  ~SerialLink() { close(); }

  bool open(const std::string & device, int baud)
  {
    fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      last_error_ = std::string("open ") + device + ": " + std::strerror(errno);
      return false;
    }

    struct termios tty {};
    if (::tcgetattr(fd_, &tty) != 0) {
      last_error_ = std::string("tcgetattr: ") + std::strerror(errno);
      close();
      return false;
    }

    ::cfmakeraw(&tty);
    const speed_t sp = to_speed(baud);
    ::cfsetispeed(&tty, sp);
    ::cfsetospeed(&tty, sp);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSTOPB;      // 1 stop bit
    tty.c_cflag &= ~PARENB;      // no parity
    tty.c_cflag &= ~CRTSCTS;     // no hw flow control
    tty.c_cc[VMIN]  = 0;         // fully non-blocking
    tty.c_cc[VTIME] = 0;

    if (::tcsetattr(fd_, TCSANOW, &tty) != 0) {
      last_error_ = std::string("tcsetattr: ") + std::strerror(errno);
      close();
      return false;
    }
    ::tcflush(fd_, TCIOFLUSH);
    rx_.clear();
    return true;
  }

  void close()
  {
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
  }

  bool is_open() const { return fd_ >= 0; }
  const std::string & last_error() const { return last_error_; }

  // ---- CRC-8/ATM: poly 0x07, init 0x00. Identical to MB_Crc8 on the MCU. ----
  static uint8_t crc8(const char * d, size_t n)
  {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < n; ++i) {
      crc ^= static_cast<uint8_t>(d[i]);
      for (int b = 0; b < 8; ++b) {
        crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x07)
                           : static_cast<uint8_t>(crc << 1);
      }
    }
    return crc;
  }

  bool send_velocity(int32_t left_sps, int32_t right_sps)
  {
    char payload[48];
    std::snprintf(payload, sizeof(payload), "V,%ld,%ld",
                  static_cast<long>(left_sps), static_cast<long>(right_sps));
    return send_payload(payload);
  }

  bool send_enable(bool on)  { return send_payload(on ? "E,1" : "E,0"); }
  bool send_estop()          { return send_payload("S"); }
  bool send_reset()          { return send_payload("R"); }
  bool send_ping()           { return send_payload("P"); }

  // Drain the RX buffer. Returns true if at least one VALID frame arrived; the
  // NEWEST one is written to `out`. Frames with a bad CRC are counted and
  // discarded -- never acted upon.
  bool poll(Feedback & out, uint64_t & crc_errors)
  {
    if (fd_ < 0) return false;

    char buf[512];
    for (;;) {
      ssize_t n = ::read(fd_, buf, sizeof(buf));
      if (n > 0) {
        rx_.append(buf, static_cast<size_t>(n));
        if (rx_.size() > 8192) {                    // runaway guard
          rx_.erase(0, rx_.size() - 2048);
        }
        continue;
      }
      break;                                        // EAGAIN / EOF
    }

    bool got = false;
    size_t start;
    while ((start = rx_.find('$')) != std::string::npos) {
      size_t star = rx_.find('*', start);
      if (star == std::string::npos) break;         // incomplete
      if (rx_.size() < star + 3) break;             // CRC not here yet

      const std::string payload = rx_.substr(start + 1, star - start - 1);
      const std::string crc_hex = rx_.substr(star + 1, 2);

      rx_.erase(0, star + 3);

      unsigned want = 0;
      if (std::sscanf(crc_hex.c_str(), "%2x", &want) != 1) continue;
      if (crc8(payload.data(), payload.size()) != static_cast<uint8_t>(want)) {
        ++crc_errors;                               // CORRUPT -> DROP
        continue;
      }
      if (payload.empty() || payload[0] != 'F') continue;

      Feedback f;
      long ls, rs, lv, rv;
      unsigned fl, sq;
      if (std::sscanf(payload.c_str(), "F,%ld,%ld,%ld,%ld,%u,%u",
                      &ls, &rs, &lv, &rv, &fl, &sq) != 6) {
        continue;
      }
      f.left_steps  = static_cast<int32_t>(ls);
      f.right_steps = static_cast<int32_t>(rs);
      f.left_sps    = static_cast<int32_t>(lv);
      f.right_sps   = static_cast<int32_t>(rv);
      f.flags       = static_cast<uint16_t>(fl);
      f.seq         = static_cast<uint16_t>(sq);
      out = f;
      got = true;
    }
    return got;
  }

private:
  bool send_payload(const char * payload)
  {
    if (fd_ < 0) return false;
    char frame[64];
    const uint8_t c = crc8(payload, std::strlen(payload));
    int n = std::snprintf(frame, sizeof(frame), "$%s*%02X\n", payload, c);
    if (n <= 0) return false;

    const char * p = frame;
    size_t left = static_cast<size_t>(n);
    while (left > 0) {
      ssize_t w = ::write(fd_, p, left);
      if (w < 0) {
        if (errno == EAGAIN || errno == EINTR) continue;
        last_error_ = std::string("write: ") + std::strerror(errno);
        return false;
      }
      p += w;
      left -= static_cast<size_t>(w);
    }
    return true;
  }

  static speed_t to_speed(int baud)
  {
    switch (baud) {
      case 9600:   return B9600;
      case 19200:  return B19200;
      case 38400:  return B38400;
      case 57600:  return B57600;
      case 115200: return B115200;
      case 230400: return B230400;
      case 460800: return B460800;
      case 921600: return B921600;
      default:     return B115200;
    }
  }

  int fd_{-1};
  std::string rx_;
  std::string last_error_;
};

}  // namespace minibot_hardware

#endif  // MINIBOT_HARDWARE__SERIAL_LINK_HPP_
