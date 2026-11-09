#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <sstream>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>

// トピック通信用のヘッダー
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

using namespace std::chrono_literals;

class JoystickSerialBridgeNode : public rclcpp::Node
{
public:
  JoystickSerialBridgeNode() : Node("joystick_serial_bridge_node")
  {
    // パラメータ宣言
    this->declare_parameter<double>("dt", 0.01); // 10ms
    this->declare_parameter<std::string>("port", "/dev/ttyACM0");

    this->get_parameter("dt", dt_);
    this->get_parameter("port", port_);

    try {
      // シリアルポートの設定
      my_serial_.setPort(port_);
      my_serial_.setBaudrate(115200);
      
      serial::Timeout to = serial::Timeout::simpleTimeout(10);
      my_serial_.setTimeout(to);

      my_serial_.open();
    } catch (serial::IOException& e) {
      RCLCPP_ERROR(this->get_logger(), "ポート %s を開けませんでした: %s", port_.c_str(), e.what());
      return;
    }

    if (my_serial_.isOpen()) {
      RCLCPP_INFO(this->get_logger(), "シリアルポートが初期化されました: %s", port_.c_str());
      
      // 1. 他のノードから計算済みの目標ロール角を受信（別ノードが計算した phi_com_y）
      target_roll_sub_ = this->create_subscription<std_msgs::msg::Float64>("/joystick/target_roll", 10, 
        std::bind(&JoystickSerialBridgeNode::target_roll_callback, this, std::placeholders::_1));

      // 3. Arduinoから取得した状態を他ノードへ配るパブリッシャ
      // 配列順: [potA, potB, angle_value, pwmA, pwmB, delta_t]
      arduino_state_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/joystick/joystick_state", 10);

      // 4. Arduinoへデータ送信および受信を行う定周期タイマーループ (dt周期)
      control_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(dt_), 
        std::bind(&JoystickSerialBridgeNode::control_loop, this));
    }
  }

private:
  // 目標ロール角の受信 callback
  void target_roll_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    phi_com_y_ = msg->data;
  }

  // 電流制御・剛性調整等に使われるゲインパラメータの受信 callback
  void gain_param_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    gain_param_ = msg->data;
  }

  // カンマ区切りの文字列を分解するヘルパー関数
  std::vector<std::string> split(const std::string &str, char delimiter)
  {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(str);
    while (std::getline(tokenStream, token, delimiter)) {
      tokens.push_back(token);
    }
    return tokens;
  }

  // メイン制御・シリアル送信ループ
  void control_loop()
  {
    // --- 1. Arduinoからのセンサデータ受信とパブリッシュ ---
    std::string last_line = "";
    while (my_serial_.available() > 0) {
      // Arduino側: potA,potB,angle_value,pwmA,pwmB,delta_t\n
      last_line = my_serial_.readline(); 
    }

    if (!last_line.empty()) {
      // 改行コードなどのトリミング
      last_line.erase(std::remove(last_line.begin(), last_line.end(), '\r'), last_line.end());
      last_line.erase(std::remove(last_line.begin(), last_line.end(), '\n'), last_line.end());

      std::vector<std::string> data_tokens = split(last_line, ',');
      
      // Arduinoから期待通りの6つのデータ（delta_t含む）が届いているか検証
      if (data_tokens.size() >= 6) {
        try {
          double potA = std::stod(data_tokens[0]); // potA (現在のロールパルス)
          double potB = std::stod(data_tokens[1]); // potB (現在のピッチパルス)

          // phi_x, phi_hum_y の計算
          double phi_x = -(potA - potA_min_) / (potA_max_ - potA_min_);
          double phi_hum_y = -(potB - potB_min_) / (potB_max_ - potB_min_);

          double delta_t_us = std::stod(data_tokens[5]); // 1周期の処理時間 [us]

          auto state_msg = std_msgs::msg::Float64MultiArray();
          state_msg.data.resize(6);
          
          state_msg.data[0] = potA;
          state_msg.data[1] = potB;
          state_msg.data[2] = std::stod(data_tokens[2]); // angle_value
          state_msg.data[3] = std::stod(data_tokens[3]); // pwmA
          state_msg.data[4] = std::stod(data_tokens[4]); // pwmB
          state_msg.data[5] = delta_t_us;                // delta_t [us]

          // ROS 2 トピックにパブリッシュ
          arduino_state_pub_->publish(state_msg);

          // ターミナルへ周期時間を表示 (マイクロ秒 および ミリ秒)
          RCLCPP_INFO(this->get_logger(), "Arduino Loop Time: %.0f us (%.2f ms)", 
                      delta_t_us, delta_t_us / 1000.0);

        } catch (const std::invalid_argument& e) {
          RCLCPP_WARN(this->get_logger(), "データ変換に失敗（文字欠損の可能性）: %s", e.what());
        }
      }
    }

    // --- 2. ArduinoのPIDが要求する目標パルス値（位置）へのマッピング ---
    double desiredPulsesB = 485.0 + (phi_com_y_ * 200.0);
    double desiredPulsesA = 535.0;

    // --- 3. Arduinoへの指令値送信（カンマ区切り形式、末尾改行） ---
    char send_buf[128];
    snprintf(send_buf, sizeof(send_buf), "%.2f,%.2f,%.2f\n", desiredPulsesA, desiredPulsesB, gain_param_);
    
    try {
      my_serial_.write(send_buf);
    } catch (serial::IOException& e) {
      RCLCPP_ERROR(this->get_logger(), "シリアル送信エラー: %s", e.what());
    }

    RCLCPP_DEBUG(this->get_logger(), "【送信】A(Roll): %.2f, B(Pitch): %.2f, Gain: %.2f", 
                 desiredPulsesA, desiredPulsesB, gain_param_);
  }

  // ROS2 ＆ シリアル通信オブジェクト
  serial::Serial my_serial_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_roll_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr gain_param_sub_;
  
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr arduino_state_pub_;

  // 設定パラメータ
  double dt_;
  std::string port_;

  // ポテンショメータの初期設定パラメータ
  double potA_max_ = 1023.0;
  double potA_min_ = 360.0;
  double potB_max_ = 915.0;
  double potB_min_ = 220.0;

  // 制御用内部インターフェース変数
  double phi_com_y_ = 0.0;    
  double gain_param_ = 1.0; 
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoystickSerialBridgeNode>());
  rclcpp::shutdown();
  return 0;
}