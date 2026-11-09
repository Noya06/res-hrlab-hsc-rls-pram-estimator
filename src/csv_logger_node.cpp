#include <chrono>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class CsvLoggerNode : public rclcpp::Node
{
public:
  CsvLoggerNode() : Node("csv_logger_node")
  {
    // パラメータの宣言（dt=0.01s の場合、10秒間で1000ステップ）
    this->declare_parameter<int>("steps", 10000);
    this->declare_parameter<std::string>("output_dir", "/home/hurolab/Programs/colcon_ws/data/datafiles");

    // パラメータの取得
    this->get_parameter("steps", max_steps_);
    this->get_parameter("output_dir", output_dir_);

    // 計算結果トピックの購読
    calc_result_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/joystick/calc_results", 10,
      std::bind(&CsvLoggerNode::calc_results_callback, this, std::placeholders::_1));

    state_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/real_robot_hum/state", 10,
      std::bind(&CsvLoggerNode::state_callback, this, std::placeholders::_1));

    // CSV保存トリガーの購読
    save_trigger_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/optimization/save_csv_trigger", 10,
      std::bind(&CsvLoggerNode::save_trigger_callback, this, std::placeholders::_1));

    // シミュレーション1周完了通知
    simulation_done_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "/optimization/simulation_done", 10);

    // CSV保存完了通知
    csv_saved_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "/optimization/csv_saved", 10);

    // 新試行開始時(/optimization/rho_cmd)の記録バッファリセット用購読
    rho_cmd_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/optimization/rho_cmd", 10,
      std::bind(&CsvLoggerNode::rho_cmd_callback, this, std::placeholders::_1));

    // シミュレーション完了/リセット通知の購読
    sim_done_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/optimization/simulation_done", 10,
      std::bind(&CsvLoggerNode::sim_done_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "CSV記録ノードが起動しました。(max_steps = %d, 時間 = %.1f秒)", max_steps_, max_steps_ * 0.01);
  }

  ~CsvLoggerNode()
  {
    save_to_csv();
  }

private:
  void rho_cmd_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    (void)msg;
    reset_recording_state();
    RCLCPP_INFO(this->get_logger(), "[RESET] 新しいパラメータを受信しました。記録バッファをクリアして計測を開始します。");
  }

  void sim_done_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg->data) {
      return;
    }

    pending_reset_ = true;
    RCLCPP_INFO(this->get_logger(), "[RESET] エピソード完了/リセット要求 (/optimization/simulation_done) を受け取りました。");
  }

  void calc_results_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (step_count_ >= max_steps_) {
      RCLCPP_INFO_ONCE(this->get_logger(), "指定記録ステップ数 (%d) に達しました。新規記録を一時停止します。", max_steps_);
      publish_simulation_done();
      return;
    }

    if (msg->data.size() < 7) return;

    CsvRow row;
    row.time      = msg->data[0];
    row.e_2       = msg->data[1];
    row.e_3       = msg->data[2];
    row.v_r       = msg->data[3];
    row.v_xi      = msg->data[4];
    row.phi_com_y = msg->data[5];
    row.phi_hum_y = msg->data[6];
    row.y_difference = row.phi_hum_y - row.phi_com_y;
    row.position_x = position_x_;
    row.position_z = position_z_;
    row.yaw = yaw_;

    records_.push_back(row);
    step_count_++;

    if (step_count_ >= max_steps_) {
      publish_simulation_done();
    }
  }

  void state_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 3) {
      RCLCPP_WARN_ONCE(
        this->get_logger(),
        "/real_robot_hum/stateには位置x、位置z、yawが必要です。");
      return;
    }

    position_x_ = msg->data[0];
    position_z_ = msg->data[1];
    yaw_ = msg->data[2];
    state_received_ = true;
  }

  void save_trigger_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data) {
      RCLCPP_INFO(this->get_logger(), "CSV保存トリガーを受信しました。書き出してバッファをリセットします。");
      const bool saved = save_to_csv();
      if (saved) {
        publish_csv_saved();
      }
      reset_recording_state();
      pending_reset_ = false;
    }
  }

  std::string get_current_timestamp()
  {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S");
    return ss.str();
  }

  bool save_to_csv()
  {
    if (records_.empty()) {
      RCLCPP_WARN(this->get_logger(), "記録されたデータがありません。CSVは生成されませんでした。");
      return false;
    }

    std::error_code ec;
    std::filesystem::create_directories(output_dir_, ec);
    if (ec) {
      RCLCPP_ERROR(this->get_logger(), "ディレクトリの作成に失敗しました: %s", output_dir_.c_str());
      return false;
    }

    std::string timestamp = get_current_timestamp();
    std::stringstream filename_ss;
    filename_ss << "result_" << timestamp 
                << "_steps" << max_steps_ 
                << ".csv";

    std::filesystem::path file_path = std::filesystem::path(output_dir_) / filename_ss.str();

    std::ofstream file(file_path);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "CSVファイルを開けませんでした: %s", file_path.string().c_str());
      return false;
    }

    file << "# --- Experiment Parameters ---\n";
    file << "# Timestamp: " << timestamp << "\n";
    file << "# steps: " << max_steps_ << "\n";
    file << "# output_dir: " << output_dir_ << "\n";
    file << "# -----------------------------\n";
    file << "time,e_2,e_3,v_r,v_xi,phi_com_y,phi_hum_y,y_difference,"
            "position_x,position_z,yaw\n";

    for (const auto& r : records_) {
      file << r.time << "," << r.e_2 << "," << r.e_3 << "," << r.v_r << "," << r.v_xi
           << "," << r.phi_com_y << "," << r.phi_hum_y
           << "," << r.y_difference << "," << r.position_x << "," << r.position_z
           << "," << r.yaw << "\n";
    }
    
    file.close();
    RCLCPP_INFO(this->get_logger(), "CSVファイルが正常に保存されました (%d 行): %s", step_count_, file_path.string().c_str());
    return true;
  }

  void publish_simulation_done()
  {
    if (simulation_done_published_) {
      return;
    }

    std_msgs::msg::Bool done_msg;
    done_msg.data = true;
    simulation_done_pub_->publish(done_msg);
    simulation_done_published_ = true;
    RCLCPP_INFO(this->get_logger(), "シミュレーション1周完了を通知しました (/optimization/simulation_done = true)");
  }

  void publish_csv_saved()
  {
    if (csv_saved_published_) {
      return;
    }

    std_msgs::msg::Bool saved_msg;
    saved_msg.data = true;
    csv_saved_pub_->publish(saved_msg);
    csv_saved_published_ = true;
    RCLCPP_INFO(this->get_logger(), "CSV保存完了を通知しました (/optimization/csv_saved = true)");
  }

  void reset_recording_state()
  {
    records_.clear();
    step_count_ = 0;
    simulation_done_published_ = false;
    csv_saved_published_ = false;
  }

  struct CsvRow {
    double time, e_2, e_3, v_r, v_xi, phi_com_y, phi_hum_y, y_difference;
    double position_x, position_z, yaw;
  };

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr calc_result_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr save_trigger_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr rho_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sim_done_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr simulation_done_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr csv_saved_pub_;
  
  int max_steps_;
  std::string output_dir_;
  int step_count_ = 0;
  bool simulation_done_published_ = false;
  bool csv_saved_published_ = false;
  bool pending_reset_ = false;
  bool state_received_ = false;
  double position_x_ = 0.0;
  double position_z_ = 0.0;
  double yaw_ = 0.0;
  std::vector<CsvRow> records_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CsvLoggerNode>());
  rclcpp::shutdown();
  return 0;
}