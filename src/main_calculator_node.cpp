#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class MainCalculatorNode : public rclcpp::Node
{
public:
  MainCalculatorNode()
  : Node("main_calculator_node")
  {
    declare_parameter<double>("dt", 0.01);
    declare_parameter<double>("k_2", 0.5);
    declare_parameter<double>("k_3", -0.3);
    declare_parameter<double>("k_v", 1.0);
    declare_parameter<double>("k_w", 1.0);
    declare_parameter<double>("center_B", 512.0);
    declare_parameter<double>("max_deviation_B", 100.0);
    declare_parameter<double>("max_roll_rad", 0.5);

    get_parameter("dt", dt_);
    get_parameter("k_2", k_2_);
    get_parameter("k_3", k_3_);
    get_parameter("k_v", k_v_);
    get_parameter("k_w", k_w_);
    get_parameter("center_B", center_b_);
    get_parameter("max_deviation_B", max_deviation_b_);
    get_parameter("max_roll_rad", max_roll_rad_);

    target_roll_pub_ =
      create_publisher<std_msgs::msg::Float64>("/joystick/target_roll", 10);
    calc_result_pub_ =
      create_publisher<std_msgs::msg::Float64MultiArray>("/joystick/calc_results", 100);
    twist_pub_ =
      create_publisher<geometry_msgs::msg::Twist>("/real_robot_hum/twist", 10);

    error_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/target/robot_errors", 10,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() < 2) {
          RCLCPP_WARN(this->get_logger(), "robot_errorsにはe2,e3が必要です。");
          return;
        }
        e2_ = msg->data[0];
        e3_ = msg->data[1];
        if (msg->data.size() >= 3) {
          observed_vr_ = msg->data[2];
        }
        errors_received_ = true;
      });

    joystick_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/joystick/joystick_state", 10,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() < 2) {
          RCLCPP_WARN(this->get_logger(), "joystick_stateにはpotA,potBが必要です。");
          return;
        }
        pot_a_ = msg->data[0];
        pot_b_ = msg->data[1];
        joystick_received_ = true;
      });

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(dt_));
    timer_ = create_wall_timer(period, std::bind(&MainCalculatorNode::control_step, this));

    RCLCPP_INFO(
      get_logger(),
      "HSC制御ノードを起動しました: k2=%.6f, k3=%.6f, %.1f Hz",
      k_2_, k_3_, 1.0 / dt_);
  }

private:
  void control_step()
  {
    if (!errors_received_) {
      publish_twist(0.0, 0.0, 0.0);
      return;
    }

    // ROS2側で基本HSCの機械制御量を計算し、UnityのJoystickへ送る。
    const double phi_com_y = k_2_ * e2_ + k_3_ * e3_;
    std_msgs::msg::Float64 target_roll;
    target_roll.data = phi_com_y;
    target_roll_pub_->publish(target_roll);

    // DummyJoystickはこのメッセージを受けてJoystick状態を返すため、
    // 最初の周期ではJoystick受信を待たずにU_comを送信する。
    if (!joystick_received_) {
      publish_twist(0.0, 0.0, 0.0);
      return;
    }

    // DummyJoystickまたは実JoystickのpotBから人間操作量を復元する。
    // 既存系の符号規約は変更せず、現在のmainの定義を維持する。
    const double phi_hum_y =
      std::clamp((pot_b_ - center_b_) / max_deviation_b_, -1.0, 1.0) * max_roll_rad_;
    const double forward_input =
      std::clamp((pot_a_ - 512.0) / 512.0, -1.0, 1.0);
    const double v_xi = k_v_ * forward_input;
    const double omega = k_w_ * phi_hum_y;

    // Unity上の人間操作後ロボットへ、実際に得られたU_humで送信する。
    publish_twist(v_xi, 0.0, omega);

    // e2,e3,U_com,U_humを同一サンプルとしてCSV loggerへ渡す。
    std_msgs::msg::Float64MultiArray result;
    result.data = {
      step_count_ * dt_,  // 0: time
      e2_,                // 1: e2
      e3_,                // 2: e3
      observed_vr_,       // 3: Unityから受信した基準速度
      v_xi,               // 4: 送信前進速度
      phi_com_y,          // 5: U_com
      phi_hum_y           // 6: U_hum
    };
    calc_result_pub_->publish(result);
    ++step_count_;
  }

  void publish_twist(double v_xi, double v_eta, double omega)
  {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = v_xi;
    twist.linear.y = v_eta;
    twist.angular.z = omega;
    twist_pub_->publish(twist);
  }

  double dt_ = 0.01;
  double k_2_ = 0.5;
  double k_3_ = -0.3;
  double k_v_ = 1.0;
  double k_w_ = 1.0;
  double center_b_ = 512.0;
  double max_deviation_b_ = 100.0;
  double max_roll_rad_ = 0.5;

  double e2_ = 0.0;
  double e3_ = 0.0;
  double observed_vr_ = 0.0;
  double pot_a_ = 512.0;
  double pot_b_ = 512.0;
  int step_count_ = 0;
  bool errors_received_ = false;
  bool joystick_received_ = false;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr target_roll_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr calc_result_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr error_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joystick_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MainCalculatorNode>());
  rclcpp::shutdown();
  return 0;
}
