#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

using namespace std::chrono_literals;

class IocCalculatorNode : public rclcpp::Node
{
public:
  explicit IocCalculatorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) 
  : Node("ioc_calculator_node", options)
  {
    // 制御パラメータの宣言
    this->declare_parameter<double>("alpha", 0.0);
    this->declare_parameter<double>("k_2", 0.0);
    this->declare_parameter<double>("k_3", 0.0);
    this->declare_parameter<double>("c_0", 0.0);
    this->declare_parameter<double>("x0", 0.0);
    this->declare_parameter<double>("y0", -1.0);
    this->declare_parameter<double>("theta0_deg", 0.0);
    this->declare_parameter<double>("dt", 0.01);
    this->declare_parameter<double>("k_v", 1.0);
    this->declare_parameter<double>("k_w", 1.0);
    this->declare_parameter<double>("rho", 0.0);
    this->declare_parameter<double>("ioc_damping", 1e-3);
    this->declare_parameter<double>("max_control_roll", 1.0);

    // CSV参照エラー使用モード用のパラメータ宣言
    this->declare_parameter<bool>("use_csv_errors", false);
    this->declare_parameter<std::string>("human_csv_path", "/home/hurolab/Programs/colcon_ws/data/ref_60.0_10.0_0.5.csv");

    this->get_parameter("alpha", alpha_);
    this->get_parameter("k_2", k_2_);
    this->get_parameter("k_3", k_3_);
    this->get_parameter("c_0", c_0_);
    this->get_parameter("dt", dt_);
    this->get_parameter("k_v", k_v_);
    this->get_parameter("k_w", k_w_);
    this->get_parameter("rho", rho_);
    this->get_parameter("ioc_damping", ioc_damping_);
    this->get_parameter("max_control_roll", max_control_roll_);
    if (ioc_damping_ <= 0.0 || max_control_roll_ <= 0.0) {
      throw std::invalid_argument(
        "ioc_damping and max_control_roll must be positive");
    }
    this->get_parameter("x0", x0_);
    this->get_parameter("y0", y0_);
    this->get_parameter("theta0_deg", theta0_deg_);

    this->get_parameter("use_csv_errors", use_csv_errors_);
    this->get_parameter("human_csv_path", human_csv_path_);

    // CSV参照モードが有効の場合は参照CSVデータを事前読み込み
    if (use_csv_errors_) {
      load_human_csv(human_csv_path_);
    }

    // 初期状態のリセット
    reset_state();

    // 外部からのパラメータ変更を動的に反映するコールバックの登録
    param_callback_handle_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        bool param_updated = false;
        bool csv_updated = false;

        for (const auto & param : parameters) {
          if (param.get_name() == "k_2") {
            k_2_ = param.as_double();
            param_updated = true;
          } else if (param.get_name() == "k_3") {
            k_3_ = param.as_double();
            param_updated = true;
          } else if (param.get_name() == "alpha") {
            alpha_ = param.as_double();
            param_updated = true;
          } else if (param.get_name() == "c_0") {
            c_0_ = param.as_double();
          } else if (param.get_name() == "dt") {
            dt_ = param.as_double();
          } else if (param.get_name() == "k_v") {
            k_v_ = param.as_double();
          } else if (param.get_name() == "k_w") {
            k_w_ = param.as_double();
          } else if (param.get_name() == "rho") {
            rho_ = param.as_double();
          } else if (param.get_name() == "use_csv_errors") {
            use_csv_errors_ = param.as_bool();
            csv_updated = true;
          } else if (param.get_name() == "human_csv_path") {
            human_csv_path_ = param.as_string();
            csv_updated = true;
          }
        }

        if (csv_updated && use_csv_errors_) {
          load_human_csv(human_csv_path_);
        }

        if (param_updated || csv_updated) {
          reset_state();
        }

        return result;
      });

    // 出力トピック
    calc_result_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/joystick/calc_results", 10000);

    // 入力トピック
    robot_errors_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "target/robot_errors", 10,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 3) {
          unity_e2_ = msg->data[0];
          unity_e3_ = msg->data[1];
          observed_vr_ = msg->data[2];
          if (msg->data.size() >= 4) {
            rho_ = msg->data[3];
          }
          errors_received_ = true;
        }
      });

    arduino_state_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/joystick/joystick_state", 10,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 2) {
          latest_potA_ = msg->data[0];
          latest_potB_ = msg->data[1];
          arduino_received_ = true;
        }
      });

    // タイマーの設定（dt=0.01s）
    auto timer_period = std::chrono::duration<double>(dt_);
    control_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
      std::bind(&IocCalculatorNode::control_timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "メイン計算ノード起動（%.1f Hz） [use_csv_errors: %s]",
                1.0 / dt_, use_csv_errors_ ? "true" : "false");
  }

private:
  void load_human_csv(const std::string & path)
  {
    std::ifstream file(path);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "参照CSVファイルを開けませんでした: %s", path.c_str());
      return;
    }

    ref_e2_.clear();
    ref_e3_.clear();
    ref_vr_.clear();
    ref_v_xi_.clear();

    std::string line;
    int e2_idx = -1, e3_idx = -1, vr_idx = -1, v_xi_idx = -1;
    bool header_parsed = false;

    while (std::getline(file, line)) {
      if (line.empty() || line[0] == '#') continue;

      std::stringstream ss(line);
      std::string cell;
      std::vector<std::string> row;

      while (std::getline(ss, cell, ',')) {
        row.push_back(cell);
      }

      if (!header_parsed) {
        for (size_t i = 0; i < row.size(); ++i) {
          if (row[i] == "e_2" || row[i] == "e2") e2_idx = static_cast<int>(i);
          if (row[i] == "e_3" || row[i] == "e3") e3_idx = static_cast<int>(i);
          if (row[i] == "v_r" || row[i] == "vr") vr_idx = static_cast<int>(i);
          if (row[i] == "v_xi" || row[i] == "vxi" || row[i] == "V") v_xi_idx = static_cast<int>(i);
        }
        header_parsed = true;
        continue;
      }

      if (e2_idx != -1 && e3_idx != -1 && static_cast<int>(row.size()) > std::max(e2_idx, e3_idx)) {
        try {
          ref_e2_.push_back(std::stod(row[e2_idx]));
          ref_e3_.push_back(std::stod(row[e3_idx]));
          if (vr_idx != -1 && static_cast<int>(row.size()) > vr_idx) {
            ref_vr_.push_back(std::stod(row[vr_idx]));
          } else {
            ref_vr_.push_back(0.0);
          }
          if (v_xi_idx != -1 && static_cast<int>(row.size()) > v_xi_idx) {
            ref_v_xi_.push_back(std::stod(row[v_xi_idx]));
          } else {
            ref_v_xi_.push_back(0.0);
          }
        } catch (const std::exception & e) {
          continue;
        }
      }
    }
    RCLCPP_INFO(this->get_logger(), "参照CSVを正常にロードしました (%s, データ数: %zu 行)",
                path.c_str(), ref_e2_.size());
  }

  void reset_state()
  {
    robot_x_ = x0_;
    robot_y_ = y0_;
    robot_theta_ = theta0_deg_ * M_PI / 180.0;
    step_count_ = 0;
    RCLCPP_INFO(this->get_logger(),
                "[RESET] 初期位置・姿勢をリセットしました。 robot=(%.2f, %.2f, %.2f deg)",
                robot_x_, robot_y_, theta0_deg_);
  }

  // 制御ループ（タイマー周期ごとに1ステップ実行）
  void control_timer_callback()
  {
    // Unity手動操作時（use_csv_errors_ == false）のみUnityからの受信を待機
    if (!use_csv_errors_ && !errors_received_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Unityからの誤差データ(target/robot_errors)を待機中...");
      return;
    }

    // 1. 全前進速度 V の決定
    double V = 0.0;
    if (use_csv_errors_ && !ref_v_xi_.empty()) {
      size_t idx = std::min(static_cast<size_t>(step_count_), ref_v_xi_.size() - 1);
      V = ref_v_xi_[idx];
    } else {
      // potA=1024 is forward, potA=0 is reverse.
      double phi_x = std::max(std::min((latest_potA_ - 512.0) / 512.0, 1.0), -1.0);
      V = k_v_ * phi_x;
    }

    // 2. 旋回速度 omega の決定
    double potB = arduino_received_ ? latest_potB_ : center_B_;
    double phi_hum_y = std::max(std::min(
        (potB - center_B_) / max_deviation_B_, 1.0), -1.0) * max_roll_rad_;
    double omega = k_w_ * phi_hum_y;

    // 3. 逆最適制御則による速度・入力の計算
    double current_e2 = unity_e2_;
    double current_e3 = unity_e3_;
    double v_r = observed_vr_;

    // CSV参照エラーモード時はCSVから取得
    if (use_csv_errors_ && !ref_e2_.empty()) {
      size_t idx = std::min(static_cast<size_t>(step_count_), ref_e2_.size() - 1);
      current_e2 = ref_e2_[idx];
      current_e3 = ref_e3_[idx];
      if (!ref_vr_.empty()) {
        v_r = ref_vr_[idx];
      }
    }

    double v_xi = 0.0, v_eta = 0.0, u = 0.0;
    inversely_optimal_control(current_e2, current_e3, V, v_r, v_xi, v_eta, u);

    // IOC出力はcalc_results経由でRLS統合ノードへ渡す。
    double phi_com_y = 0.0;
    if (std::abs(k_w_) > 1e-9) {
      phi_com_y = std::clamp(-u / k_w_, -max_control_roll_, max_control_roll_);
    } else {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "k_wがゼロに近いため、IOCの操舵出力を無効化します");
    }
    // CSV用ログデータの出力
    auto calc_result_msg = std_msgs::msg::Float64MultiArray();
    calc_result_msg.data = {
      round(step_count_ * dt_ * 100.0) / 100.0, // 0: time
      robot_x_,                                 // 1: x
      robot_y_,                                 // 2: y
      robot_theta_,                             // 3: theta
      current_e2,                               // 4: e_2
      current_e3,                               // 5: e_3
      v_r,                                      // 6: v_r
      v_xi,                                     // 7: v_xi
      v_eta,                                    // 8: v_eta
      u,                                        // 9: u
      phi_com_y,                                // 10: phi_com_y
      phi_hum_y                                 // 11: phi_hum_y
    };
    calc_result_pub_->publish(calc_result_msg);

    // 4. ロボット位置・姿勢の数値積分
    double x_dot = v_xi * std::cos(robot_theta_) - v_eta * std::sin(robot_theta_);
    double y_dot = v_xi * std::sin(robot_theta_) + v_eta * std::cos(robot_theta_);

    robot_x_ += x_dot * dt_;
    robot_y_ += y_dot * dt_;
    robot_theta_ += omega * dt_;
    robot_theta_ = normalize_angle(robot_theta_);

    step_count_++;
  }

  static double normalize_angle(double angle)
  {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  void inversely_optimal_control(double e_2, double e_3, double V, double v_r, double& v_xi, double& v_eta, double& u)
  {
    double beta = std::atan(alpha_ * e_2);
    v_xi = V * std::cos(beta);
    v_eta = V * std::sin(beta);

    double w_r = rho_ * v_r;
    double a = k_2_ * e_2 * (v_r * std::sin(e_3) - V * std::sin(beta)) + k_3_ * e_3 * w_r;
    double b = -k_3_ * e_3;

    const double b_square = b * b;
    const double denominator = b_square + ioc_damping_;
    const double p = (a + std::sqrt(a * a + b_square * b_square)) / denominator + c_0_;
    u = -p * b;
  }

  // ROS 2 オブジェクト
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr calc_result_pub_;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr robot_errors_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr arduino_state_sub_;

  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // 制御パラメータ
  double alpha_, k_2_, k_3_, c_0_, dt_;
  double k_v_, k_w_, rho_, ioc_damping_, max_control_roll_;
  double x0_, y0_, theta0_deg_;

  // CSV参照エラーモード用
  bool use_csv_errors_ = true;
  std::string human_csv_path_;
  std::vector<double> ref_e2_;
  std::vector<double> ref_e3_;
  std::vector<double> ref_vr_;
  std::vector<double> ref_v_xi_;

  int step_count_ = 0;
  bool errors_received_ = false;
  bool arduino_received_ = false;
  double observed_vr_ = 0.0;
  double unity_e2_ = 0.0;
  double unity_e3_ = 0.0;
  double latest_potA_ = 512.0;
  double latest_potB_ = 512.0;

  double center_B_ = 512.0;
  double max_deviation_B_ = 100.0;
  double max_roll_rad_ = 0.5;

  double robot_x_, robot_y_, robot_theta_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IocCalculatorNode>());
  rclcpp::shutdown();
  return 0;
}