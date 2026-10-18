// gptpet_hardware: ros2_control SystemInterface for the ESP32 motor driver
// (embedded/arduino_driver/arduino_driver.ino) over USB serial.
//
// Units: ros2_control speaks rad/s (velocity command and state) and rad (position state).
// The firmware speaks motor rpm and encoder counts. The conversion lives here and only here;
// the firmware never needs to know the wheel radius and this file never needs the motor's
// rated speed except to warn about saturation.
//
// Wire protocol (115200 8N1, little-endian), see the firmware header for details:
//   host -> ESP32  'W' + 4 x int16 (rpm * 100, order L1, L2, R1, R2) + XOR checksum of the
//                  8 payload bytes. Sent every write() cycle. The firmware stops the motors
//                  if it hears nothing for 500 ms.
//   ESP32 -> host  'M' + 24 + 4 x (int16 rpm * 100, int32 encoder counts) + XOR checksum of
//                  the 24 payload bytes, every 20 ms. rpm is signed and measured; counts are
//                  cumulative with + = forward.
//
// Hardware parameters (URDF <ros2_control><hardware><param name="...">):
//   serial_ports            comma-separated candidates, the first that opens wins
//                           (default "/dev/ttyACM0,/dev/ttyACM1")
//   baud_rate               default 115200
//   max_rpm                 motor rated speed; only used to warn when the controller asks
//                           for more than the motors can deliver (default 76)
//   encoder_counts_per_rev  encoder counts per wheel revolution, must match the firmware's
//                           COUNTS_PER_REV (default 5764 = 11 ppr * 4 edges * 131:1)
//   feedback_timeout        seconds without a valid report before the reported wheel
//                           velocities are zeroed (default 0.5)
//
// Joint order in the URDF must be L1, L2, R1, R2 (front left, rear left, front right, rear
// right), matching the firmware's motor order.

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <sstream>
#include <string>
#include <vector>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace gptpet_hardware {

namespace {

constexpr size_t NUM_MOTORS = 4;

constexpr double RPM_TO_RAD_S = 2.0 * M_PI / 60.0;
constexpr double RAD_S_TO_RPM = 60.0 / (2.0 * M_PI);
constexpr double WIRE_RPM_SCALE = 100.0;  // rpm travel as int16 rpm * WIRE_RPM_SCALE

constexpr uint8_t CMD_HEADER = 'W';
constexpr size_t CMD_PAYLOAD_LEN = 2 * NUM_MOTORS;      // 4 x int16
constexpr size_t CMD_LEN = 1 + CMD_PAYLOAD_LEN + 1;      // header + payload + checksum

constexpr uint8_t REPORT_HEADER = 'M';
constexpr uint8_t REPORT_PAYLOAD_LEN = 6 * NUM_MOTORS;  // 4 x (int16 rpm + int32 counts)
constexpr size_t REPORT_LEN = 2 + REPORT_PAYLOAD_LEN + 1;  // header + len + payload + checksum

constexpr size_t RX_BUFFER_MAX = 1024;  // the firmware sends ~1.4 KB/s; this is ~0.7 s of data

uint8_t xor_checksum(const uint8_t * data, size_t len) {
  uint8_t cs = 0;
  for (size_t k = 0; k < len; ++k) {
    cs ^= data[k];
  }
  return cs;
}

int16_t read_i16_le(const uint8_t * p) {
  return static_cast<int16_t>(static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8));
}

int32_t read_i32_le(const uint8_t * p) {
  uint32_t u = static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
               (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
  return static_cast<int32_t>(u);
}

void write_i16_le(uint8_t * p, int16_t v) {
  uint16_t u = static_cast<uint16_t>(v);
  p[0] = static_cast<uint8_t>(u & 0xFF);
  p[1] = static_cast<uint8_t>((u >> 8) & 0xFF);
}

bool baud_to_speed(int baud, speed_t & out) {
  switch (baud) {
    case 9600: out = B9600; return true;
    case 19200: out = B19200; return true;
    case 38400: out = B38400; return true;
    case 57600: out = B57600; return true;
    case 115200: out = B115200; return true;
    case 230400: out = B230400; return true;
#ifdef B460800
    case 460800: out = B460800; return true;
#endif
#ifdef B921600
    case 921600: out = B921600; return true;
#endif
    default: return false;
  }
}

std::vector<std::string> split_csv(const std::string & s) {
  std::vector<std::string> out;
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, ',')) {
    const auto start = item.find_first_not_of(" \t");
    const auto end = item.find_last_not_of(" \t");
    if (start != std::string::npos) {
      out.push_back(item.substr(start, end - start + 1));
    }
  }
  return out;
}

}  // namespace

class GptpetHardwareInterface : public hardware_interface::SystemInterface {
public:
  GptpetHardwareInterface()
    : logger_(rclcpp::get_logger("GptpetHardwareInterface")) {}

  ~GptpetHardwareInterface() override {
    close_serial_port();  // sends a stop first
  }

  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
      RCLCPP_ERROR(logger_, "Failed to initialize hardware interface");
      return CallbackReturn::ERROR;
    }

    if (info_.joints.size() != NUM_MOTORS) {
      RCLCPP_ERROR(
        logger_, "Expected %zu joints (L1, L2, R1, R2), got %zu", NUM_MOTORS, info_.joints.size());
      return CallbackReturn::ERROR;
    }

    hw_positions_.assign(NUM_MOTORS, 0.0);
    hw_velocities_.assign(NUM_MOTORS, 0.0);
    hw_commands_.assign(NUM_MOTORS, 0.0);
    last_counts_.assign(NUM_MOTORS, 0);
    rx_buffer_.reserve(2 * RX_BUFFER_MAX);

    // Parameters from the URDF
    serial_ports_ = split_csv(param_or("serial_ports", "/dev/ttyACM0,/dev/ttyACM1"));
    if (serial_ports_.empty()) {
      RCLCPP_ERROR(logger_, "serial_ports parameter is empty");
      return CallbackReturn::ERROR;
    }
    const int baud = static_cast<int>(param_double("baud_rate", 115200.0));
    if (!baud_to_speed(baud, baud_speed_)) {
      RCLCPP_WARN(logger_, "Unsupported baud_rate %d, using 115200", baud);
      baud_speed_ = B115200;
    }
    max_rpm_ = param_double("max_rpm", 76.0);
    const double counts_per_rev = param_double("encoder_counts_per_rev", 11.0 * 4.0 * 131.0);
    rad_per_count_ = 2.0 * M_PI / counts_per_rev;
    // A jump larger than this between two reports can't be real motion (4 wheel turns in one
    // 20 ms sample), so it is treated as a firmware reset and the count baseline is re-taken.
    count_jump_limit_ = static_cast<int32_t>(4.0 * counts_per_rev);
    feedback_timeout_s_ = param_double("feedback_timeout", 0.5);

    if (!open_serial_port()) {
      RCLCPP_ERROR(logger_, "Failed to open the motor driver serial port");
      return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(
      logger_, "Initialized: baud %d, max_rpm %.1f, %.0f counts/rev, feedback timeout %.2f s",
      baud, max_rpm_, counts_per_rev, feedback_timeout_s_);
    for (size_t i = 0; i < NUM_MOTORS; ++i) {
      RCLCPP_INFO(logger_, "Joint %zu: %s", i, info_.joints[i].name.c_str());
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
    std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
    std::fill(hw_velocities_.begin(), hw_velocities_.end(), 0.0);
    rx_buffer_.clear();
    have_counts_ = false;  // re-baseline encoder counts; positions keep accumulating
    have_feedback_ = false;
    feedback_stale_ = false;
    if (serial_fd_ >= 0) {
      tcflush(serial_fd_, TCIFLUSH);
    }
    send_stop();
    RCLCPP_INFO(logger_, "Activated, motors stopped");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
    send_stop();
    RCLCPP_INFO(logger_, "Deactivated, motors stopped");
    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
      state_interfaces.emplace_back(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
      state_interfaces.emplace_back(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
    }
    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
      command_interfaces.emplace_back(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]);
    }
    return command_interfaces;
  }

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & /*period*/) override {
    read_serial_data(time);

    // If the ESP32 goes quiet, don't keep integrating a stale velocity into odometry
    if (have_feedback_ && !feedback_stale_ &&
        (time - last_feedback_time_).seconds() > feedback_timeout_s_) {
      feedback_stale_ = true;
      std::fill(hw_velocities_.begin(), hw_velocities_.end(), 0.0);
      RCLCPP_WARN(
        logger_, "No motor feedback for %.2f s, reporting zero wheel velocity",
        feedback_timeout_s_);
    }
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override {
    write_serial_command();
    return hardware_interface::return_type::OK;
  }

private:
  // --- parameters -------------------------------------------------------------------------

  std::string param_or(const std::string & key, const std::string & def) const {
    const auto it = info_.hardware_parameters.find(key);
    return it == info_.hardware_parameters.end() ? def : it->second;
  }

  double param_double(const std::string & key, double def) const {
    const auto it = info_.hardware_parameters.find(key);
    if (it == info_.hardware_parameters.end()) {
      return def;
    }
    try {
      return std::stod(it->second);
    } catch (const std::exception &) {
      RCLCPP_WARN(
        logger_, "Parameter %s='%s' is not a number, using %g", key.c_str(), it->second.c_str(),
        def);
      return def;
    }
  }

  // --- serial port ------------------------------------------------------------------------

  bool open_serial_port() {
    for (const auto & port : serial_ports_) {
      int fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
      if (fd < 0) {
        RCLCPP_WARN(logger_, "Unable to open serial port %s: %s", port.c_str(), strerror(errno));
        continue;
      }

      struct termios tty;
      std::memset(&tty, 0, sizeof(tty));
      if (tcgetattr(fd, &tty) != 0) {
        RCLCPP_ERROR(logger_, "tcgetattr failed for %s: %s", port.c_str(), strerror(errno));
        ::close(fd);
        continue;
      }

      // Fully raw: no CR/NL translation, no echo, no signals, no software flow control.
      // The data is binary, so any translation corrupts it.
      cfmakeraw(&tty);
      cfsetispeed(&tty, baud_speed_);
      cfsetospeed(&tty, baud_speed_);
      tty.c_cflag |= (CLOCAL | CREAD);            // ignore modem lines, enable receiver
      tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);  // no parity, 1 stop bit, no hw flow control
      tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;   // 8 data bits
      tty.c_cc[VMIN] = 0;                           // reads never block (also O_NONBLOCK)
      tty.c_cc[VTIME] = 0;

      if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        RCLCPP_ERROR(logger_, "tcsetattr failed for %s: %s", port.c_str(), strerror(errno));
        ::close(fd);
        continue;
      }

      tcflush(fd, TCIOFLUSH);  // drop the firmware's boot banner and any stale bytes
      serial_fd_ = fd;
      RCLCPP_INFO(logger_, "Opened motor driver serial port %s", port.c_str());
      return true;
    }
    RCLCPP_ERROR(logger_, "None of the configured serial ports could be opened");
    return false;
  }

  void close_serial_port() {
    if (serial_fd_ < 0) {
      return;
    }
    send_stop();
    tcdrain(serial_fd_);
    ::close(serial_fd_);
    serial_fd_ = -1;
  }

  // --- commands: rad/s -> rpm -> 'W' packet ------------------------------------------------

  void send_rpm(const double rpm[NUM_MOTORS]) {
    if (serial_fd_ < 0) {
      return;
    }
    uint8_t pkt[CMD_LEN];
    pkt[0] = CMD_HEADER;
    for (size_t m = 0; m < NUM_MOTORS; ++m) {
      long scaled = std::lround(rpm[m] * WIRE_RPM_SCALE);
      scaled = std::max(-32768L, std::min(32767L, scaled));
      write_i16_le(&pkt[1 + 2 * m], static_cast<int16_t>(scaled));
    }
    pkt[CMD_LEN - 1] = xor_checksum(&pkt[1], CMD_PAYLOAD_LEN);

    const ssize_t written = ::write(serial_fd_, pkt, CMD_LEN);
    if (written != static_cast<ssize_t>(CMD_LEN)) {
      RCLCPP_WARN_THROTTLE(
        logger_, steady_clock_, 2000, "Motor command write failed (%zd of %zu bytes): %s",
        written, CMD_LEN, written < 0 ? strerror(errno) : "short write");
    }
  }

  void send_stop() {
    const double zeros[NUM_MOTORS] = {0.0, 0.0, 0.0, 0.0};
    send_rpm(zeros);
  }

  void write_serial_command() {
    double rpm[NUM_MOTORS];
    bool saturated = false;
    for (size_t m = 0; m < NUM_MOTORS; ++m) {
      double value = hw_commands_[m] * RAD_S_TO_RPM;
      if (!std::isfinite(value)) {
        value = 0.0;  // controller not active yet, or reference timed out
      }
      if (std::fabs(value) > max_rpm_) {
        saturated = true;
      }
      rpm[m] = value;
    }
    if (saturated) {
      RCLCPP_WARN_THROTTLE(
        logger_, steady_clock_, 2000,
        "Wheel command exceeds max_rpm (%.0f rpm = %.2f rad/s); the motors will saturate",
        max_rpm_, max_rpm_ * RPM_TO_RAD_S);
    }
    send_rpm(rpm);
  }

  // --- feedback: 'M' packets -> rad/s and rad ----------------------------------------------

  void read_serial_data(const rclcpp::Time & now) {
    if (serial_fd_ < 0) {
      return;
    }

    uint8_t chunk[256];
    for (int guard = 0; guard < 16; ++guard) {
      const ssize_t n = ::read(serial_fd_, chunk, sizeof(chunk));
      if (n <= 0) {
        break;
      }
      rx_buffer_.insert(rx_buffer_.end(), chunk, chunk + n);
      if (static_cast<size_t>(n) < sizeof(chunk)) {
        break;
      }
    }
    if (rx_buffer_.size() > RX_BUFFER_MAX) {
      rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.end() - RX_BUFFER_MAX);
    }

    // Scan for complete, checksummed reports. Anything else (text replies, torn packets)
    // is skipped byte by byte. The unconsumed tail may hold the start of the next report.
    size_t i = 0;
    while (rx_buffer_.size() - i >= REPORT_LEN) {
      if (rx_buffer_[i] != REPORT_HEADER || rx_buffer_[i + 1] != REPORT_PAYLOAD_LEN) {
        ++i;
        continue;
      }
      const uint8_t * payload = rx_buffer_.data() + i + 2;
      if (xor_checksum(payload, REPORT_PAYLOAD_LEN) != payload[REPORT_PAYLOAD_LEN]) {
        ++i;
        continue;
      }
      parse_report(payload, now);
      i += REPORT_LEN;
    }
    rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + i);
  }

  void parse_report(const uint8_t * payload, const rclcpp::Time & now) {
    for (size_t m = 0; m < NUM_MOTORS; ++m) {
      const int16_t rpm_scaled = read_i16_le(payload + 6 * m);
      const int32_t counts = read_i32_le(payload + 6 * m + 2);

      hw_velocities_[m] = (rpm_scaled / WIRE_RPM_SCALE) * RPM_TO_RAD_S;

      if (have_counts_) {
        // unsigned subtraction so a counter wrap is well defined
        const int32_t delta = static_cast<int32_t>(
          static_cast<uint32_t>(counts) - static_cast<uint32_t>(last_counts_[m]));
        if (delta > count_jump_limit_ || delta < -count_jump_limit_) {
          RCLCPP_WARN_THROTTLE(
            logger_, steady_clock_, 2000,
            "Encoder count jump of %d on %s (firmware reset?), re-baselining", delta,
            info_.joints[m].name.c_str());
        } else {
          hw_positions_[m] += delta * rad_per_count_;
        }
      }
      last_counts_[m] = counts;
    }
    have_counts_ = true;

    if (!have_feedback_) {
      RCLCPP_INFO(logger_, "Receiving motor feedback from the ESP32");
    } else if (feedback_stale_) {
      RCLCPP_INFO(logger_, "Motor feedback restored");
    }
    have_feedback_ = true;
    feedback_stale_ = false;
    last_feedback_time_ = now;
  }

  // --- state ------------------------------------------------------------------------------

  std::vector<double> hw_positions_, hw_velocities_, hw_commands_;
  rclcpp::Logger logger_;
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  std::vector<std::string> serial_ports_;
  speed_t baud_speed_ = B115200;
  double max_rpm_ = 76.0;
  double rad_per_count_ = 0.0;
  int32_t count_jump_limit_ = 0;
  double feedback_timeout_s_ = 0.5;

  int serial_fd_ = -1;
  std::vector<uint8_t> rx_buffer_;

  std::vector<int32_t> last_counts_;
  bool have_counts_ = false;
  bool have_feedback_ = false;
  bool feedback_stale_ = false;
  rclcpp::Time last_feedback_time_;
};

}  // namespace gptpet_hardware

PLUGINLIB_EXPORT_CLASS(gptpet_hardware::GptpetHardwareInterface, hardware_interface::SystemInterface)
