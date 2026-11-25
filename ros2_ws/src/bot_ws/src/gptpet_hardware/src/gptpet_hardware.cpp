// gptpet_hardware: ros2_control SystemInterface for the ESP32 motor driver
// (embedded/arduino_driver/arduino_driver.ino) over USB serial.
//
// Units: ros2_control speaks rad/s (velocity command and state) and rad (position state).
// The firmware speaks motor rpm and encoder counts. The conversion lives here and only here;
// the firmware never needs to know the wheel radius.
//
// Wire protocol (115200 8N1, little-endian), see the firmware header for details:
//   host -> ESP32  'W' + 4 x int16 (rpm * 100, order L1, L2, R1, R2) + XOR checksum of the
//                  8 payload bytes. Sent every write() cycle. The firmware stops the motors
//                  if it hears nothing for watchdog_ms.
//   host -> ESP32  text lines "set <name> <value>\n" for parameter changes (see below)
//   ESP32 -> host  'M' + 24 + 4 x (int16 rpm * 100, int32 encoder counts) + XOR checksum of
//                  the 24 payload bytes, every report_ms. rpm is signed and measured; counts
//                  are cumulative with + = forward.
//   ESP32 -> host  text lines (OK_..., ERR_..., PARAMS ..., WATCHDOG_stopped), logged here.
//
// Hardware parameters (URDF <ros2_control><hardware><param name="...">), fixed at startup:
//   serial_ports            comma-separated candidates, the first that opens wins
//                           (default "/dev/ttyACM0,/dev/ttyACM1")
//   baud_rate               default 115200
//   encoder_counts_per_rev  encoder counts per wheel revolution, must match the firmware's
//                           COUNTS_PER_REV (default 5764 = 11 ppr * 4 edges * 131:1)
//   feedback_timeout        seconds without a valid report before the reported wheel
//                           velocities are zeroed (default 0.5)
// If the port is missing at startup or disappears later (ESP32 re-plugged or re-flashed), the
// interface keeps retrying every second and re-sends the tuning parameters once it is back.
// If the port is present but dead (writes blocked for 1 s, or no feedback for 5 s; seen on the
// Jetson when another device on the same USB hub re-enumerates), it is closed and reopened, and
// on a repeat the USB device itself is reset (needs the udev rule in bin/robot/).
//
// Motor tuning parameters live on a ROS node this plugin creates, /gptpet_motors, so they
// can be set from a YAML file (startup/config/motor_tuning.yaml) and changed live:
//   ros2 param list /gptpet_motors
//   ros2 param set /gptpet_motors kp "[3.5, 3.5, 3.5, 3.5]"
//   ros2 param set /gptpet_motors min_pwm 30
// Every change is validated, forwarded to the ESP32 as "set ..." text commands, and followed
// by a "params" request so the firmware's resulting state is logged. The whole set is pushed
// each time the interface activates, so ROS is the source of truth while the robot runs.
//   kp, ki, kd        double[4]  PID gains per motor (L1, L2, R1, R2)
//   min_pwm           int        PID output floor, 0..254 (firmware "floor")
//   max_rpm           double     target clamp, 1..500; also the saturation warning threshold
//   watchdog_ms       int        firmware stops the motors after this long without a command
//   sample_ms         int        speed measurement and PID period
//   report_ms         int        feedback report period
//   open_loop         bool       bypass the PID, map targets straight to PWM
//   enc_sign          int[4]     1 or -1 per motor, encoder direction
//   motor_reversed    bool[4]    motor wired backwards
//
// Joint order in the URDF must be L1, L2, R1, R2 (front left, rear left, front right, rear
// right), matching the firmware's motor order.

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#ifdef __linux__
#include <linux/usbdevice_fs.h>
#endif

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <fstream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
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

constexpr size_t RX_BUFFER_MAX = 1024;   // the firmware sends ~1.4 KB/s; this is ~0.7 s of data
constexpr double WRITE_STALL_S = 1.0;        // commands blocked (EAGAIN) this long -> port is stuck
constexpr double FEEDBACK_STALL_S = 5.0;     // open port, nothing from the ESP32 this long -> stuck
constexpr double STALL_ESCALATE_WINDOW_S = 30.0;  // a second stall within this -> USB reset
constexpr size_t MAX_TEXT_LINE = 240;    // longest firmware reply we keep (PARAMS line is ~150)
constexpr int MAX_TEXT_CMDS_PER_CYCLE = 2;  // keep parameter pushes gentle on the ESP32's UART

const char * const PARAM_NODE_NAME = "gptpet_motors";
const std::vector<std::string> TUNING_PARAMS = {
  "kp", "ki", "kd", "min_pwm", "max_rpm", "watchdog_ms", "sample_ms", "report_ms",
  "open_loop", "enc_sign", "motor_reversed"};

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

// "set <name> <value>\n" as the firmware expects it
std::string firmware_set(const std::string & name, double value) {
  char buf[96];
  std::snprintf(buf, sizeof(buf), "set %s %.6g\n", name.c_str(), value);
  return std::string(buf);
}

rcl_interfaces::msg::ParameterDescriptor describe(const std::string & text) {
  rcl_interfaces::msg::ParameterDescriptor d;
  d.description = text;
  return d;
}

rcl_interfaces::msg::ParameterDescriptor describe_int(
  const std::string & text, int64_t lo, int64_t hi) {
  auto d = describe(text);
  rcl_interfaces::msg::IntegerRange r;
  r.from_value = lo;
  r.to_value = hi;
  r.step = 0;
  d.integer_range.push_back(r);
  return d;
}

rcl_interfaces::msg::ParameterDescriptor describe_double(
  const std::string & text, double lo, double hi) {
  auto d = describe(text);
  rcl_interfaces::msg::FloatingPointRange r;
  r.from_value = lo;
  r.to_value = hi;
  r.step = 0.0;
  d.floating_point_range.push_back(r);
  return d;
}

}  // namespace

class GptpetHardwareInterface : public hardware_interface::SystemInterface {
public:
  GptpetHardwareInterface()
    : logger_(rclcpp::get_logger("GptpetHardwareInterface")) {}

  ~GptpetHardwareInterface() override {
    stop_parameter_node();
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

    // Fixed parameters from the URDF
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
    const double counts_per_rev = param_double("encoder_counts_per_rev", 11.0 * 4.0 * 131.0);
    rad_per_count_ = 2.0 * M_PI / counts_per_rev;
    // A jump larger than this between two reports can't be real motion (4 wheel turns in one
    // 20 ms sample), so it is treated as a firmware reset and the count baseline is re-taken.
    count_jump_limit_ = static_cast<int32_t>(4.0 * counts_per_rev);
    feedback_timeout_s_ = param_double("feedback_timeout", 0.5);

    if (!open_serial_port(false)) {
      RCLCPP_ERROR(
        logger_, "Motor driver serial port not available yet; will keep trying every second");
      last_reconnect_attempt_ = steady_clock_.now();
    }

    // Tunable parameters on their own ROS node
    try {
      start_parameter_node();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger_, "Failed to create the %s parameter node: %s", PARAM_NODE_NAME, e.what());
      return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(
      logger_, "Initialized: baud %d, %.0f counts/rev, feedback timeout %.2f s, tuning on /%s",
      baud, counts_per_rev, feedback_timeout_s_, PARAM_NODE_NAME);
    for (size_t i = 0; i < NUM_MOTORS; ++i) {
      RCLCPP_INFO(logger_, "Joint %zu: %s", i, info_.joints[i].name.c_str());
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
    std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
    std::fill(hw_velocities_.begin(), hw_velocities_.end(), 0.0);
    rx_buffer_.clear();
    rx_text_line_.clear();
    rx_text_garbage_ = false;
    have_counts_ = false;  // re-baseline encoder counts; positions keep accumulating
    have_feedback_ = false;
    feedback_stale_ = false;
    if (serial_fd_ >= 0) {
      tcflush(serial_fd_, TCIFLUSH);
    }
    send_stop();
    push_all_parameters();
    RCLCPP_INFO(logger_, "Activated, motors stopped, pushing tuning parameters to the ESP32");
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
    maybe_reconnect();
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

    // Port open but silent for a long time: the USB link is stuck (or the ESP32 is hung)
    if (serial_fd_ >= 0) {
      const double quiet_s = have_feedback_
        ? (time - last_feedback_time_).seconds()
        : (steady_clock_.now() - port_opened_at_).seconds();
      if (quiet_s > FEEDBACK_STALL_S) {
        handle_serial_stall("no feedback from the ESP32 for 5 s");
      }
    }
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override {
    write_serial_command();
    flush_text_commands();
    return hardware_interface::return_type::OK;
  }

private:
  // --- fixed parameters (URDF) --------------------------------------------------------------

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

  // --- tuning parameters (ROS node) ---------------------------------------------------------

  void start_parameter_node() {
    param_node_ = std::make_shared<rclcpp::Node>(PARAM_NODE_NAME);

    param_node_->declare_parameter<std::vector<double>>(
      "kp", {3.0, 3.0, 3.0, 3.0},
      describe("PID proportional gain per motor (L1, L2, R1, R2), PWM counts per rpm of error"));
    param_node_->declare_parameter<std::vector<double>>(
      "ki", {15.0, 15.0, 15.0, 15.0},
      describe("PID integral gain per motor, PWM counts per rpm*s"));
    param_node_->declare_parameter<std::vector<double>>(
      "kd", {0.0, 0.0, 0.0, 0.0}, describe("PID derivative gain per motor, PWM counts per rpm/s"));
    param_node_->declare_parameter<int64_t>(
      "min_pwm", 35,
      describe_int("PID output floor in PWM counts; also the slowest the wheels can turn", 0, 254));
    param_node_->declare_parameter<double>(
      "max_rpm", 76.0,
      describe_double("Target clamp in the firmware and saturation warning threshold here", 1.0, 500.0));
    param_node_->declare_parameter<int64_t>(
      "watchdog_ms", 500,
      describe_int("Firmware stops the motors after this long without a command", 50, 10000));
    param_node_->declare_parameter<int64_t>(
      "sample_ms", 20, describe_int("Speed measurement and PID period in the firmware", 5, 200));
    param_node_->declare_parameter<int64_t>(
      "report_ms", 20, describe_int("Feedback report period in the firmware", 5, 1000));
    param_node_->declare_parameter<bool>(
      "open_loop", false, describe("Bypass the PID and map targets straight to PWM"));
    param_node_->declare_parameter<std::vector<int64_t>>(
      "enc_sign", {1, 1, 1, 1},
      describe("Encoder direction per motor, 1 or -1; flip if measured speed opposes the target"));
    param_node_->declare_parameter<std::vector<bool>>(
      "motor_reversed", {true, true, false, true},
      describe("Motor wired backwards per motor; flips PWM and measured sign"));

    max_rpm_.store(param_node_->get_parameter("max_rpm").as_double());

    // Registered after the declarations so startup values go out once, from push_all_parameters()
    param_cb_handle_ = param_node_->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) { return on_parameters_set(params); });

    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(param_node_);
    stop_spinning_.store(false);
    spin_thread_ = std::thread([this]() {
      // spin_once with a timeout keeps this loop responsive to stop_spinning_ and to shutdown
      while (!stop_spinning_.load() && rclcpp::ok()) {
        executor_->spin_once(std::chrono::milliseconds(100));
      }
    });
  }

  void stop_parameter_node() {
    stop_spinning_.store(true);
    if (executor_) {
      executor_->cancel();  // wake spin_once if it is waiting
    }
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    param_cb_handle_.reset();
    if (executor_ && param_node_) {
      executor_->remove_node(param_node_);
    }
    executor_.reset();
    param_node_.reset();
  }

  // Validates one parameter and appends the firmware commands that apply it.
  // Returns an empty string on success, otherwise the reason it was rejected.
  std::string commands_for_parameter(const rclcpp::Parameter & p, std::vector<std::string> & cmds) {
    const std::string & name = p.get_name();
    try {
      if (name == "kp" || name == "ki" || name == "kd") {
        const auto v = p.as_double_array();
        if (v.size() != NUM_MOTORS) {
          return name + " needs exactly 4 values (L1, L2, R1, R2)";
        }
        for (size_t i = 0; i < NUM_MOTORS; ++i) {
          if (!(v[i] >= 0.0) || !std::isfinite(v[i])) {
            return name + "[" + std::to_string(i) + "] must be a finite number >= 0";
          }
          cmds.push_back(firmware_set(name + std::to_string(i), v[i]));
        }
      } else if (name == "min_pwm") {
        cmds.push_back(firmware_set("floor", static_cast<double>(p.as_int())));
      } else if (name == "max_rpm") {
        cmds.push_back(firmware_set("max_rpm", p.as_double()));
      } else if (name == "watchdog_ms" || name == "sample_ms" || name == "report_ms") {
        cmds.push_back(firmware_set(name, static_cast<double>(p.as_int())));
      } else if (name == "open_loop") {
        cmds.push_back(firmware_set("open_loop", p.as_bool() ? 1.0 : 0.0));
      } else if (name == "enc_sign") {
        const auto v = p.as_integer_array();
        if (v.size() != NUM_MOTORS) {
          return "enc_sign needs exactly 4 values (L1, L2, R1, R2)";
        }
        for (size_t i = 0; i < NUM_MOTORS; ++i) {
          if (v[i] != 1 && v[i] != -1) {
            return "enc_sign[" + std::to_string(i) + "] must be 1 or -1";
          }
          cmds.push_back(firmware_set("enc_sign" + std::to_string(i), static_cast<double>(v[i])));
        }
      } else if (name == "motor_reversed") {
        const auto v = p.as_bool_array();
        if (v.size() != NUM_MOTORS) {
          return "motor_reversed needs exactly 4 values (L1, L2, R1, R2)";
        }
        for (size_t i = 0; i < NUM_MOTORS; ++i) {
          cmds.push_back(firmware_set("reversed" + std::to_string(i), v[i] ? 1.0 : 0.0));
        }
      }
      // anything else (use_sim_time, ...) isn't ours: accept and ignore
    } catch (const rclcpp::ParameterTypeException & e) {
      return name + ": " + e.what();
    }
    return "";
  }

  rcl_interfaces::msg::SetParametersResult on_parameters_set(
    const std::vector<rclcpp::Parameter> & params) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    std::vector<std::string> cmds;
    for (const auto & p : params) {
      const std::string error = commands_for_parameter(p, cmds);
      if (!error.empty()) {
        result.successful = false;
        result.reason = error;
        RCLCPP_WARN(logger_, "Rejected parameter change: %s", error.c_str());
        return result;
      }
    }

    if (!cmds.empty()) {
      for (const auto & p : params) {
        if (std::find(TUNING_PARAMS.begin(), TUNING_PARAMS.end(), p.get_name()) != TUNING_PARAMS.end()) {
          RCLCPP_INFO(
            logger_, "Parameter %s = %s, sending to the ESP32", p.get_name().c_str(),
            p.value_to_string().c_str());
          if (p.get_name() == "max_rpm") {
            max_rpm_.store(p.as_double());
          }
        }
      }
      cmds.push_back("params\n");  // so the firmware's resulting state shows up in the log
      enqueue_text_commands(cmds);
    }
    return result;
  }

  // Sends every tuning parameter, so the firmware matches ROS after a reflash or reboot
  void push_all_parameters() {
    if (!param_node_) {
      return;
    }
    std::vector<std::string> cmds;
    for (const auto & name : TUNING_PARAMS) {
      rclcpp::Parameter p;
      if (param_node_->get_parameter(name, p)) {
        const std::string error = commands_for_parameter(p, cmds);
        if (!error.empty()) {
          RCLCPP_WARN(logger_, "Not sending %s: %s", name.c_str(), error.c_str());
        }
      }
    }
    cmds.push_back("params\n");
    enqueue_text_commands(cmds);
  }

  void enqueue_text_commands(const std::vector<std::string> & cmds) {
    std::lock_guard<std::mutex> lock(text_mutex_);
    for (const auto & c : cmds) {
      pending_text_.push_back(c);
    }
  }

  // Called from write(): sends a few queued text commands per control cycle
  void flush_text_commands() {
    if (serial_fd_ < 0) {
      return;
    }
    std::lock_guard<std::mutex> lock(text_mutex_);
    for (int sent = 0; sent < MAX_TEXT_CMDS_PER_CYCLE && !pending_text_.empty(); ++sent) {
      std::string & cmd = pending_text_.front();
      const ssize_t n = ::write(serial_fd_, cmd.data(), cmd.size());
      if (n < 0) {
        if (is_device_gone(errno)) {
          handle_serial_loss("parameter write");
          return;
        }
        RCLCPP_WARN_THROTTLE(
          logger_, steady_clock_, 2000, "Text command write failed: %s", strerror(errno));
        return;  // retry next cycle
      }
      if (static_cast<size_t>(n) < cmd.size()) {
        cmd.erase(0, static_cast<size_t>(n));  // send the rest next cycle
        return;
      }
      RCLCPP_DEBUG(logger_, "-> ESP32: %s", cmd.substr(0, cmd.size() - 1).c_str());
      pending_text_.pop_front();
    }
  }

  // --- serial port --------------------------------------------------------------------------

  bool open_serial_port(bool quiet) {
    for (const auto & port : serial_ports_) {
      int fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
      if (fd < 0) {
        if (!quiet) {
          RCLCPP_WARN(logger_, "Unable to open serial port %s: %s", port.c_str(), strerror(errno));
        }
        continue;
      }

      // Exclusive: a second ros2_control_node (e.g. a launch started twice) must fail to open the
      // port instead of silently interleaving its commands with ours.
      if (ioctl(fd, TIOCEXCL) != 0) {
        RCLCPP_WARN(logger_, "Could not set exclusive mode on %s: %s", port.c_str(), strerror(errno));
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
      open_port_path_ = port;
      port_opened_at_ = steady_clock_.now();
      write_blocked_ = false;
      RCLCPP_INFO(logger_, "Opened motor driver serial port %s", port.c_str());
      return true;
    }
    if (!quiet) {
      RCLCPP_ERROR(logger_, "None of the configured serial ports could be opened");
    }
    return false;
  }

  static bool is_device_gone(int err) {
    return err == EIO || err == ENODEV || err == ENXIO || err == EBADF;
  }

  // The USB device vanished (unplugged, re-flashed): drop the fd and let maybe_reconnect() retry.
  // The firmware's own watchdog stops the motors when our commands stop arriving.
  void handle_serial_loss(const char * during) {
    const int err = errno;
    RCLCPP_ERROR(
      logger_, "Motor driver serial port lost during %s: %s. Retrying every second", during,
      strerror(err));
    if (serial_fd_ >= 0) {
      ::close(serial_fd_);
      serial_fd_ = -1;
    }
    std::fill(hw_velocities_.begin(), hw_velocities_.end(), 0.0);
    rx_buffer_.clear();
    have_feedback_ = false;
    feedback_stale_ = false;
    last_reconnect_attempt_ = steady_clock_.now();
  }

  // The port is open but dead. Closing it makes the kernel abandon the stuck USB transfers, and
  // maybe_reconnect() reopens it a second later. If that didn't cure it last time, reset the USB
  // device (same effect as re-plugging the ESP32).
  void handle_serial_stall(const char * why) {
    const rclcpp::Time now = steady_clock_.now();
    stalls_in_a_row_ = ((now - last_stall_).seconds() < STALL_ESCALATE_WINDOW_S) ? stalls_in_a_row_ + 1 : 1;
    last_stall_ = now;
    RCLCPP_ERROR(
      logger_, "Motor driver serial port stalled: %s. Reopening it (stall %d)", why,
      stalls_in_a_row_);
    if (serial_fd_ >= 0) {
      ::close(serial_fd_);
      serial_fd_ = -1;
    }
    std::fill(hw_velocities_.begin(), hw_velocities_.end(), 0.0);
    rx_buffer_.clear();
    have_feedback_ = false;
    feedback_stale_ = false;
    write_blocked_ = false;
    if (stalls_in_a_row_ >= 2) {
      usb_reset_device();
    }
    last_reconnect_attempt_ = now;
  }

  // Resets the USB device behind the serial port we last opened (Linux only). Needs write access
  // to /dev/bus/usb/BBB/DDD, which the udev rule in bin/robot/99-gptpet-motors.rules grants.
  void usb_reset_device() {
#ifdef __linux__
    char * real = ::realpath(open_port_path_.c_str(), nullptr);
    if (!real) {
      RCLCPP_WARN(logger_, "USB reset: cannot resolve %s: %s", open_port_path_.c_str(), strerror(errno));
      return;
    }
    std::string tty(real);
    std::free(real);
    tty = tty.substr(tty.rfind('/') + 1);

    char * sys = ::realpath(("/sys/class/tty/" + tty + "/device").c_str(), nullptr);
    if (!sys) {
      RCLCPP_WARN(logger_, "USB reset: no sysfs device for %s: %s", tty.c_str(), strerror(errno));
      return;
    }
    std::string usb_interface(sys);
    std::free(sys);
    const std::string usb_device = usb_interface.substr(0, usb_interface.rfind('/'));

    int busnum = 0, devnum = 0;
    std::ifstream bus(usb_device + "/busnum"), dev(usb_device + "/devnum");
    if (!(bus >> busnum) || !(dev >> devnum)) {
      RCLCPP_WARN(logger_, "USB reset: cannot read bus/device number under %s", usb_device.c_str());
      return;
    }
    char node[64];
    std::snprintf(node, sizeof(node), "/dev/bus/usb/%03d/%03d", busnum, devnum);

    int fd = ::open(node, O_WRONLY);
    if (fd < 0) {
      RCLCPP_ERROR(
        logger_,
        "Cannot open %s to reset the ESP32's USB device: %s. Re-plug the ESP32, or install "
        "bin/robot/99-gptpet-motors.rules on the robot so this can be done automatically",
        node, strerror(errno));
      return;
    }
    if (ioctl(fd, USBDEVFS_RESET, 0) < 0) {
      RCLCPP_ERROR(logger_, "USB reset of %s failed: %s", node, strerror(errno));
    } else {
      RCLCPP_WARN(logger_, "USB device %s (%s) reset; waiting for it to re-enumerate", node, tty.c_str());
    }
    ::close(fd);
#else
    RCLCPP_WARN(logger_, "USB reset is only implemented on Linux");
#endif
  }

  void maybe_reconnect() {
    if (serial_fd_ >= 0) {
      return;
    }
    const rclcpp::Time now = steady_clock_.now();
    if ((now - last_reconnect_attempt_).seconds() < 1.0) {
      return;
    }
    last_reconnect_attempt_ = now;
    if (!open_serial_port(true)) {
      RCLCPP_WARN_THROTTLE(
        logger_, steady_clock_, 10000, "Motor driver serial port still unavailable, retrying");
      return;
    }
    rx_buffer_.clear();
    rx_text_line_.clear();
    rx_text_garbage_ = false;
    have_counts_ = false;
    have_feedback_ = false;
    feedback_stale_ = false;
    {
      std::lock_guard<std::mutex> lock(text_mutex_);
      pending_text_.clear();
    }
    send_stop();
    push_all_parameters();
    RCLCPP_INFO(logger_, "Motor driver serial port reconnected, tuning parameters re-sent");
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

  // --- commands: rad/s -> rpm -> 'W' packet --------------------------------------------------

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
    if (written == static_cast<ssize_t>(CMD_LEN)) {
      write_blocked_ = false;
      return;
    }
    if (written < 0 && is_device_gone(errno)) {
      handle_serial_loss("write");
      return;
    }
    if (written < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      // The kernel's USB write queue is full. A short burst is normal after a scheduling stall;
      // a full second of it means the device stopped taking data.
      const rclcpp::Time now = steady_clock_.now();
      if (!write_blocked_) {
        write_blocked_ = true;
        write_blocked_since_ = now;
      } else if ((now - write_blocked_since_).seconds() > WRITE_STALL_S) {
        handle_serial_stall("USB serial output blocked for more than 1 s");
      }
      return;
    }
    RCLCPP_WARN_THROTTLE(
      logger_, steady_clock_, 2000, "Motor command write failed (%zd of %zu bytes): %s",
      written, CMD_LEN, written < 0 ? strerror(errno) : "short write");
  }

  void send_stop() {
    const double zeros[NUM_MOTORS] = {0.0, 0.0, 0.0, 0.0};
    send_rpm(zeros);
  }

  void write_serial_command() {
    double rpm[NUM_MOTORS];
    bool saturated = false;
    const double max_rpm = max_rpm_.load();
    for (size_t m = 0; m < NUM_MOTORS; ++m) {
      double value = hw_commands_[m] * RAD_S_TO_RPM;
      if (!std::isfinite(value)) {
        value = 0.0;  // controller not active yet, or reference timed out
      }
      if (std::fabs(value) > max_rpm) {
        saturated = true;
      }
      rpm[m] = value;
    }
    if (saturated) {
      RCLCPP_WARN_THROTTLE(
        logger_, steady_clock_, 2000,
        "Wheel command exceeds max_rpm (%.0f rpm = %.2f rad/s); the motors will saturate",
        max_rpm, max_rpm * RPM_TO_RAD_S);
    }
    send_rpm(rpm);
  }

  // --- feedback: 'M' packets -> rad/s and rad, text lines -> log -----------------------------

  void read_serial_data(const rclcpp::Time & now) {
    if (serial_fd_ < 0) {
      return;
    }

    uint8_t chunk[256];
    for (int guard = 0; guard < 16; ++guard) {
      const ssize_t n = ::read(serial_fd_, chunk, sizeof(chunk));
      if (n < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK && is_device_gone(errno)) {
          handle_serial_loss("read");
        }
        break;
      }
      if (n == 0) {
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

    // Scan for complete, checksummed reports. Every other byte belongs to a text line from the
    // firmware (OK_..., ERR_..., PARAMS ...) or is a torn packet; text lines are logged, the rest
    // is dropped. The unconsumed tail may hold the start of the next report.
    size_t i = 0;
    while (rx_buffer_.size() - i >= REPORT_LEN) {
      if (rx_buffer_[i] == REPORT_HEADER && rx_buffer_[i + 1] == REPORT_PAYLOAD_LEN) {
        const uint8_t * payload = rx_buffer_.data() + i + 2;
        if (xor_checksum(payload, REPORT_PAYLOAD_LEN) == payload[REPORT_PAYLOAD_LEN]) {
          parse_report(payload, now);
          i += REPORT_LEN;
          continue;
        }
      }
      consume_text_byte(rx_buffer_[i]);
      ++i;
    }
    rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + i);
  }

  void consume_text_byte(uint8_t b) {
    if (b == '\n') {
      if (!rx_text_garbage_ && !rx_text_line_.empty()) {
        log_firmware_line(rx_text_line_);
      }
      rx_text_line_.clear();
      rx_text_garbage_ = false;
      return;
    }
    if (b == '\r') {
      return;
    }
    if (b < 0x20 || b > 0x7E) {
      rx_text_garbage_ = true;  // binary junk (torn packet): ignore until the next newline
      rx_text_line_.clear();
      return;
    }
    if (!rx_text_garbage_ && rx_text_line_.size() < MAX_TEXT_LINE) {
      rx_text_line_.push_back(static_cast<char>(b));
    }
  }

  void log_firmware_line(const std::string & line) {
    if (line.rfind("ERR", 0) == 0 || line.rfind("WATCHDOG", 0) == 0) {
      RCLCPP_WARN(logger_, "ESP32: %s", line.c_str());
    } else if (line.rfind("OK_", 0) == 0) {
      RCLCPP_DEBUG(logger_, "ESP32: %s", line.c_str());  // one per parameter; PARAMS sums it up
    } else {
      RCLCPP_INFO(logger_, "ESP32: %s", line.c_str());
    }
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

  // --- state --------------------------------------------------------------------------------

  std::vector<double> hw_positions_, hw_velocities_, hw_commands_;
  rclcpp::Logger logger_;
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  std::vector<std::string> serial_ports_;
  speed_t baud_speed_ = B115200;
  double rad_per_count_ = 0.0;
  int32_t count_jump_limit_ = 0;
  double feedback_timeout_s_ = 0.5;
  std::atomic<double> max_rpm_{76.0};

  int serial_fd_ = -1;
  std::string open_port_path_;
  rclcpp::Time last_reconnect_attempt_{0, 0, RCL_STEADY_TIME};
  rclcpp::Time port_opened_at_{0, 0, RCL_STEADY_TIME};
  bool write_blocked_ = false;
  rclcpp::Time write_blocked_since_{0, 0, RCL_STEADY_TIME};
  rclcpp::Time last_stall_{0, 0, RCL_STEADY_TIME};
  int stalls_in_a_row_ = 0;
  std::vector<uint8_t> rx_buffer_;
  std::string rx_text_line_;
  bool rx_text_garbage_ = false;

  std::vector<int32_t> last_counts_;
  bool have_counts_ = false;
  bool have_feedback_ = false;
  bool feedback_stale_ = false;
  rclcpp::Time last_feedback_time_;

  // tuning parameter node, spun on its own thread; commands cross to the control thread via a queue
  rclcpp::Node::SharedPtr param_node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::atomic<bool> stop_spinning_{false};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
  std::mutex text_mutex_;
  std::deque<std::string> pending_text_;
};

}  // namespace gptpet_hardware

PLUGINLIB_EXPORT_CLASS(gptpet_hardware::GptpetHardwareInterface, hardware_interface::SystemInterface)
