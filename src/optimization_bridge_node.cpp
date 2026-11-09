#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class OptimizationBridgeNode : public rclcpp::Node
{
public:
  OptimizationBridgeNode()
  : Node("optimization_bridge_node")
  {
    // main_calculator_node のパラメータを外部から書き換えるためのクライアントを作成
    param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "main_calculator_node");

    rho_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/optimization/rho_cmd", 10,
      std::bind(&OptimizationBridgeNode::rho_cmd_callback, this, std::placeholders::_1));

    record_trigger_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/optimization/record_trigger", 10,
      std::bind(&OptimizationBridgeNode::record_trigger_callback, this, std::placeholders::_1));

    result_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/optimization/eval_result", 10);

    RCLCPP_INFO(this->get_logger(), "Optimization bridge node started.");
  }

private:
  void rho_cmd_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() != 3) {
      RCLCPP_WARN(this->get_logger(), "rho_cmd expects 3 values: [K2, K3, alpha]");
      return;
    }

    const double k2 = msg->data[0];
    const double k3 = msg->data[1];
    const double alpha = msg->data[2];

    // MainCalculatorNode のパラメータを非同期で更新
    param_client_->set_parameters({
      rclcpp::Parameter("k_2", k2),
      rclcpp::Parameter("k_3", k3),
      rclcpp::Parameter("alpha", alpha)
    });

    RCLCPP_INFO(this->get_logger(), "rho updated on main_calculator_node: K2=%.4f, K3=%.4f, alpha=%.4f", k2, k3, alpha);
  }

  void record_trigger_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg->data) {
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Recording human trajectory for optimization.");

    const auto result = std_msgs::msg::Float64MultiArray();
    (void)result;
  }

  std::shared_ptr<rclcpp::AsyncParametersClient> param_client_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr rho_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr record_trigger_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr result_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OptimizationBridgeNode>());
  rclcpp::shutdown();
  return 0;
}