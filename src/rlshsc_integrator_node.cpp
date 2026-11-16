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
    declare_parameter<int>("phi_ioc_index", 10);
    declare_parameter<std::vector<int64_t>>("feature_indices", {4, 5, 12, 13, 14, 15, 16, 17});
    declare_parameter<std::vector<double>>("feature_scales", std::vector<double>{});
    declare_parameter<double>("joystick_center", 512.0);
    declare_parameter<double>("joystick_max_deviation", 512.0);
    declare_parameter<double>("max_roll_rad", 1.0);
    declare_parameter<double>("max_target_roll", 1.0);
    declare_parameter<double>("ioc_gain", 1.0);
    declare_parameter<double>("rls_gain", 1.0);
    declare_parameter<bool>("invert_steering", false);

    std::vector<int64_t> feature_indices;
    std::vector<double> feature_scales;
    get_parameter("phi_ioc_index", phi_ioc_index_);
    get_parameter("feature_indices", feature_indices);
    get_parameter("feature_scales", feature_scales);
    get_parameter("joystick_center", joystick_center_);
    get_parameter("joystick_max_deviation", joystick_max_deviation_);
    get_parameter("max_roll_rad", max_roll_rad_);
    get_parameter("max_target_roll", max_target_roll_);
    get_parameter("ioc_gain", ioc_gain_);
    get_parameter("rls_gain", rls_gain_);
    get_parameter("invert_steering", invert_steering_);
    if (max_target_roll_ <= 0.0 || ioc_gain_ < 0.0 || rls_gain_ < 0.0) {
      throw std::invalid_argument(
        "max_target_roll must be positive and ioc_gain/rls_gain must be non-negative");
    }
    if (feature_indices.empty() || phi_ioc_index_ < 0) {
      throw std::invalid_argument(
        "feature_indices must not be empty and phi_ioc_index must be non-negative");
    }
    for (const auto idx : feature_indices) {
      if (idx < 0) {
        throw std::invalid_argument("feature_indices must be non-negative");
      }
      feature_indices_.push_back(static_cast<std::size_t>(idx));
    }

    // 特徴量の正規化スケール設定
    if (feature_scales.empty()) {
      feature_scales_.assign(feature_indices_.size(), 1.0);
    } else if (feature_scales.size() != feature_indices_.size()) {
      throw std::invalid_argument(
        "feature_scales size must match feature_indices size");
    } else {
      feature_scales_ = feature_scales;
    }
    for (std::size_t i = 0; i < feature_scales_.size(); ++i) {
      if (!std::isfinite(feature_scales_[i]) || std::abs(feature_scales_[i]) < 1e-12) {
        throw std::invalid_argument("feature_scales values must be finite and non-zero");
      }
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
    twist_pub_ = create_publisher<geometry_msgs::msg::Twist>("/real_robot/twist", 10);
    phi_pub_ = create_publisher<std_msgs::msg::Float64>("/rlshsc/phi_com_y", 10);
    log_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/rlshsc/calc_results", 100);
    RCLCPP_INFO(get_logger(), "RLS-HSC統合ノードを起動しました (IOC + RLS加算)");
  }

  void joystick_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 2) {
      return;
    }
    joystick_x_ = msg->data[0];
    joystick_y_ = msg->data[1];
    const double normalized = std::clamp(
      (msg->data[1] - joystick_center_) / joystick_max_deviation_, -1.0, 1.0);
    phi_hum_ = (invert_steering_ ? -1.0 : 1.0) * normalized * max_roll_rad_;
    joystick_received_ = true;
  }

private:
  void theta_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() != feature_indices_.size()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "RLS thetaの次元(%zu)が特徴量次元(%zu)と一致しません",
        msg->data.size(), feature_indices_.size());
      return;
    }
    theta_ = msg->data;
    theta_received_ = true;
  }

  void calc_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    std::size_t required_index = static_cast<std::size_t>(phi_ioc_index_);
    for (const auto idx : feature_indices_) {
      required_index = std::max(required_index, idx);
    }
    if (required_index >= msg->data.size()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "IOC計算結果の列数が不足しています");
      return;
    }
    if (!theta_received_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "RLS thetaを待機中です");
      return;
    }

    double rls_prediction = 0.0;
    for (std::size_t i = 0; i < feature_indices_.size(); ++i) {
      rls_prediction += (msg->data[feature_indices_[i]] / feature_scales_[i]) * theta_[i];
    }
    const double phi_ioc = msg->data[phi_ioc_index_];
    // FFBへ送る指令はIOC出力とRLS補正値の加算値とする。
    const double target_roll = std::clamp(
      ioc_gain_ * phi_ioc + rls_gain_ * rls_prediction,
      -max_target_roll_, max_target_roll_);

    std_msgs::msg::Float64 phi_msg;
    phi_msg.data = target_roll;
    phi_pub_->publish(phi_msg);
    target_roll_pub_->publish(phi_msg);

    const double e2_abs_e2 = (msg->data.size() > 12)
      ? msg->data[12] : (msg->data[4] * std::abs(msg->data[4]));
    const double e3_abs_e3 = (msg->data.size() > 13)
      ? msg->data[13] : (msg->data[5] * std::abs(msg->data[5]));
    const double e2_abs_e3 = (msg->data.size() > 14)
      ? msg->data[14] : (msg->data[4] * std::abs(msg->data[5]));
    const double abs_e2_e3 = (msg->data.size() > 15)
      ? msg->data[15] : (std::abs(msg->data[4]) * msg->data[5]);
    const double bias_term = (msg->data.size() > 16)
      ? msg->data[16] : 1.0;
    const double rho = (msg->data.size() > 17)
      ? msg->data[17] : 0.0;

    std_msgs::msg::Float64MultiArray log_msg;
    log_msg.data = {
      msg->data[0],  // time
      msg->data[4],  // e_2
      msg->data[5],  // e_3
      msg->data[6],  // v_r
      msg->data[7],  // v_xi
      phi_ioc,
      rls_prediction,
      target_roll,
      joystick_x_,
      joystick_y_,
      phi_hum_,
    };
    log_msg.data.insert(log_msg.data.end(), theta_.begin(), theta_.end());
    log_msg.data.push_back(e2_abs_e2);
    log_msg.data.push_back(e3_abs_e3);
    log_msg.data.push_back(e2_abs_e3);
    log_msg.data.push_back(abs_e2_e3);
    log_msg.data.push_back(bias_term);
    log_msg.data.push_back(rho);
    log_pub_->publish(log_msg);

    geometry_msgs::msg::Twist twist;
    if (msg->data.size() > 8) {
      twist.linear.x = msg->data[7];
      twist.linear.y = msg->data[8];
    }
    if (joystick_received_) {
      // UnityへはFFB指令ではなく、ジョイスティックから戻った人間入力を送る。
      twist.angular.z = phi_hum_;
      twist_pub_->publish(twist);
    }
  }

  int phi_ioc_index_{10};
  std::vector<std::size_t> feature_indices_;
  std::vector<double> feature_scales_;
  double joystick_center_{512.0};
  double joystick_max_deviation_{512.0};
  double max_roll_rad_{1.0};
  double max_target_roll_{1.0};
  double ioc_gain_{1.0};
  double rls_gain_{1.0};
  bool invert_steering_{false};
  bool theta_received_{false};
  bool joystick_received_{false};
  double phi_hum_{0.0};
  double joystick_x_{0.0};
  double joystick_y_{0.0};
  std::vector<double> theta_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr theta_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joystick_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr calc_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr target_roll_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr phi_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr log_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RlshscIntegratorNode>());
  rclcpp::shutdown();
  return 0;
}