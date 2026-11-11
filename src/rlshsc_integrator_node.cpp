#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class RlshscIntegratorNode final : public rclcpp::Node
{
public:
  RlshscIntegratorNode()
  : Node("rlshsc_integrator_node")
  {
    declare_parameter<double>("k_w", 1.0);
    declare_parameter<int>("phi_ioc_index", 10);
    declare_parameter<int>("feature_start_index", 4);
    declare_parameter<int>("feature_count", 2);
    declare_parameter<double>("joystick_center", 512.0);
    declare_parameter<double>("joystick_max_deviation", 512.0);
    declare_parameter<double>("max_roll_rad", 1.0);
    declare_parameter<bool>("invert_steering", false);

    get_parameter("k_w", k_w_);
    get_parameter("phi_ioc_index", phi_ioc_index_);
    get_parameter("feature_start_index", feature_start_index_);
    get_parameter("feature_count", feature_count_);
    get_parameter("joystick_center", joystick_center_);
    get_parameter("joystick_max_deviation", joystick_max_deviation_);
    get_parameter("max_roll_rad", max_roll_rad_);
    get_parameter("invert_steering", invert_steering_);
    if (feature_count_ <= 0 || feature_start_index_ < 0 || phi_ioc_index_ < 0) {
      throw std::invalid_argument("feature indices must be non-negative and feature_count positive");
    }

    theta_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/rls/theta", 10,
      std::bind(&RlshscIntegratorNode::theta_callback, this, std::placeholders::_1));
    joystick_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/joystick/joystick_state", 10,
      std::bind(&RlshscIntegratorNode::joystick_callback, this, std::placeholders::_1));
    calc_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/joystick/calc_results", 100,
      std::bind(&RlshscIntegratorNode::calc_callback, this, std::placeholders::_1));
    target_roll_pub_ = create_publisher<std_msgs::msg::Float64>("/joystick/target_roll", 10);
    twist_pub_ = create_publisher<geometry_msgs::msg::Twist>("/real_robot_hum/twist", 10);
    phi_pub_ = create_publisher<std_msgs::msg::Float64>("/rlshsc/phi_com_y", 10);
    RCLCPP_INFO(get_logger(), "RLS-HSC統合ノードを起動しました (IOC + RLS加算)");
  }

  void joystick_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 2) {
      return;
    }
    const double normalized = std::clamp(
      (msg->data[1] - joystick_center_) / joystick_max_deviation_, -1.0, 1.0);
    phi_hum_ = (invert_steering_ ? -1.0 : 1.0) * normalized * max_roll_rad_;
    joystick_received_ = true;
  }

private:
  void theta_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() != static_cast<std::size_t>(feature_count_)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "RLS thetaの次元(%zu)が特徴量次元(%d)と一致しません", msg->data.size(), feature_count_);
      return;
    }
    theta_ = msg->data;
    theta_received_ = true;
  }

  void calc_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    const int required_index = std::max(
      phi_ioc_index_, feature_start_index_ + feature_count_ - 1);
    if (required_index < 0 || static_cast<std::size_t>(required_index) >= msg->data.size()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "IOC計算結果の列数が不足しています");
      return;
    }
    if (!theta_received_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "RLS thetaを待機中です");
      return;
    }

    double rls_prediction = 0.0;
    for (int i = 0; i < feature_count_; ++i) {
      rls_prediction += msg->data[feature_start_index_ + i] * theta_[i];
    }
    const double phi_ioc = msg->data[phi_ioc_index_];
    // FFBへ送る指令はIOC出力とRLS補正値の加算値とする。
    const double target_roll = phi_ioc + rls_prediction;

    std_msgs::msg::Float64 phi_msg;
    phi_msg.data = target_roll;
    phi_pub_->publish(phi_msg);
    target_roll_pub_->publish(phi_msg);

    geometry_msgs::msg::Twist twist;
    if (msg->data.size() > 8) {
      twist.linear.x = msg->data[7];
      twist.linear.y = msg->data[8];
    }
    if (joystick_received_) {
      // UnityへはFFB指令ではなく、ジョイスティックから戻った人間入力を送る。
      twist.angular.z = k_w_ * phi_hum_;
      twist_pub_->publish(twist);
    }
  }

  double k_w_{1.0};
  int phi_ioc_index_{10};
  int feature_start_index_{4};
  int feature_count_{2};
  double joystick_center_{512.0};
  double joystick_max_deviation_{512.0};
  double max_roll_rad_{1.0};
  bool invert_steering_{false};
  bool theta_received_{false};
  bool joystick_received_{false};
  double phi_hum_{0.0};
  std::vector<double> theta_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr theta_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joystick_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr calc_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr target_roll_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr phi_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RlshscIntegratorNode>());
  rclcpp::shutdown();
  return 0;
}
