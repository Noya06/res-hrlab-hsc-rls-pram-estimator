#include <chrono>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <linux/input.h>
#include <unistd.h>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

using namespace std::chrono_literals;

class MozaJoystickBridgeNode : public rclcpp::Node
{
public:
  MozaJoystickBridgeNode() : Node("moza_joystick_bridge_node")
  {
    this->declare_parameter<std::string>("device", "/dev/input/event4");
    this->declare_parameter<int>("axis_x_code", ABS_Y);
    this->declare_parameter<int>("axis_y_code", ABS_X);
    this->declare_parameter<double>("axis_center", 32767.5);
    this->declare_parameter<double>("axis_scale", 32767.5);
    this->declare_parameter<double>("deadzone", 0.02);
    // 定速前進モードフラグ（パラメータとして追加）
    this->declare_parameter<bool>("override_constant_forward_speed", true);

    this->get_parameter("device", device_);
    this->get_parameter("axis_x_code", axis_x_code_);
    this->get_parameter("axis_y_code", axis_y_code_);
    this->get_parameter("axis_center", axis_center_);
    this->get_parameter("axis_scale", axis_scale_);
    this->get_parameter("deadzone", deadzone_);
    this->get_parameter("override_constant_forward_speed", override_constant_forward_speed_);

    arduino_state_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/joystick/joystick_state", 10);

    open_device();

    const auto timer_period = std::chrono::duration<double>(0.01);
    read_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
      std::bind(&MozaJoystickBridgeNode::poll_input, this));

    RCLCPP_INFO(this->get_logger(),
                "MOZA input started. device=%s x_code=%d y_code=%d constant_forward=%s",
                device_.c_str(), axis_x_code_, axis_y_code_,
                override_constant_forward_speed_ ? "true" : "false");
  }

private:
  void open_device()
  {
    input_fd_ = ::open(device_.c_str(), O_RDONLY | O_NONBLOCK);
    if (input_fd_ < 0) {
      RCLCPP_WARN(this->get_logger(), "MOZA input device %s を開けませんでしたが続行します: %s",
                  device_.c_str(), std::strerror(errno));
      input_fd_ = -1;
      return;
    }

    RCLCPP_INFO(this->get_logger(), "MOZA input device を開きました: %s", device_.c_str());
  }

  // コントローラの入力デコード
  void poll_input()
  {
    if (input_fd_ < 0) {
      return;
    }

    input_event ev;
    bool updated = false;

    while (true) {
      const ssize_t bytes = ::read(input_fd_, &ev, sizeof(ev));
      if (bytes < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          break;
        }
        RCLCPP_WARN(this->get_logger(), "MOZA input read error: %s", std::strerror(errno));
        break;
      }

      if (bytes != sizeof(ev)) {
        break;
      }

      if (ev.type == EV_ABS) {
        if (ev.code == axis_x_code_) {
          raw_x_ = static_cast<double>(ev.value);
          updated = true;
        } else if (ev.code == axis_y_code_) {
          raw_y_ = static_cast<double>(ev.value);
          updated = true;
        }
      }
    }

    if (!updated) {
      return;
    }

    // パラメータを動的に取得（ノード実行中の変更にも対応）
    this->get_parameter("override_constant_forward_speed", override_constant_forward_speed_);

    const double normalized_y = apply_deadzone((raw_y_ - axis_center_) / axis_scale_);

    auto state_msg = std_msgs::msg::Float64MultiArray();
    state_msg.data.resize(6);

    if (override_constant_forward_speed_) {
      // Trueのとき: 前後(x)の入力を無視し、前進速度をmax(1024.0)にする
      state_msg.data[0] = -1024.0;
    } else {
      // Falseのとき: 通常の入力値を使用
      const double normalized_x = apply_deadzone((raw_x_ - axis_center_) / axis_scale_);
      state_msg.data[0] = 512.0 + normalized_x * 512.0;
    }

    state_msg.data[1] = 512.0 + normalized_y * 512.0;
    state_msg.data[2] = 0.0;    // angle_value
    state_msg.data[3] = 0.0;    // pwmA
    state_msg.data[4] = 0.0;    // pwmB
    state_msg.data[5] = 0.0;    // delta_t [us]
    arduino_state_pub_->publish(state_msg);
  }

  double apply_deadzone(double value) const
  {
    if (std::abs(value) < deadzone_) {
      return 0.0;
    }
    return std::clamp(value, -1.0, 1.0);
  }

  int input_fd_ = -1;
  int axis_x_code_ = ABS_X;
  int axis_y_code_ = ABS_Y;
  bool x_calibrated_ = false;
  bool y_calibrated_ = false;
  double raw_x_ = 0.0;
  double raw_y_ = 0.0;
  double axis_center_ = 32767.5;
  double axis_scale_ = 32767.5;
  double deadzone_ = 0.02;

  // 追加した変数
  bool override_constant_forward_speed_ = false;

  std::string device_;
  rclcpp::TimerBase::SharedPtr read_timer_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr arduino_state_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MozaJoystickBridgeNode>());
  rclcpp::shutdown();
  return 0;
}