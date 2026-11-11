#include <algorithm>
#include <fstream>
#include <functional>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "rls_estimator.hpp"

class RlsEstimatorNode final : public rclcpp::Node
{
public:
  RlsEstimatorNode()
  : Node("rls_estimator_node")
  {
    declare_parameter<int>("parameter_count", 2);
    declare_parameter<double>("initial_covariance", 1.0e3);
    declare_parameter<std::vector<double>>("initial_parameters", std::vector<double>{});
    declare_parameter<std::vector<int64_t>>("feature_indices", {4, 5});
    declare_parameter<int>("phi_ref_index", 11);
    declare_parameter<int>("phi_src_index", 10);
    declare_parameter<std::string>("csv_path", "");
    declare_parameter<bool>("process_csv_on_start", false);
    declare_parameter<std::string>("csv_output_path", "");

    int parameter_count;
    double initial_covariance;
    std::vector<double> initial_parameters;
    std::vector<int64_t> feature_indices;
    get_parameter("parameter_count", parameter_count);
    get_parameter("initial_covariance", initial_covariance);
    get_parameter("initial_parameters", initial_parameters);
    get_parameter("feature_indices", feature_indices);
    get_parameter("phi_ref_index", phi_ref_index_);
    get_parameter("phi_src_index", phi_src_index_);
    get_parameter("csv_path", csv_path_);
    get_parameter("process_csv_on_start", process_csv_on_start_);
    get_parameter("csv_output_path", csv_output_path_);

    if (parameter_count <= 0 || feature_indices.size() != static_cast<std::size_t>(parameter_count)) {
      throw std::invalid_argument("parameter_count must match feature_indices");
    }
    std::vector<std::size_t> indices;
    for (const auto index : feature_indices) {
      if (index < 0) {
        throw std::invalid_argument("feature_indices must be non-negative");
      }
      indices.push_back(static_cast<std::size_t>(index));
    }

    estimator_ = std::make_unique<RLSEstimator>(
      static_cast<std::size_t>(parameter_count), initial_covariance, initial_parameters);
    feature_indices_ = std::move(indices);
    theta_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("rls/theta", 10);
    sample_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/joystick/calc_results", 100,
      std::bind(&RlsEstimatorNode::sample_callback, this, std::placeholders::_1));

    if (process_csv_on_start_ && !csv_path_.empty()) {
      process_csv();
    }
    RCLCPP_INFO(this->get_logger(), "RLS推定ノードを起動しました (次元=%d)", parameter_count);
  }

private:
  void sample_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (phi_ref_index_ < 0 || phi_src_index_ < 0 ||
      static_cast<std::size_t>(std::max(phi_ref_index_, phi_src_index_)) >= msg->data.size()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "RLS入力メッセージの列数が不足しています");
      return;
    }
    std::vector<double> features;
    for (const auto index : feature_indices_) {
      if (index >= msg->data.size()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "RLS特徴量の列数が不足しています");
        return;
      }
      features.push_back(msg->data[index]);
    }
    update_and_publish(msg->data[phi_ref_index_], msg->data[phi_src_index_], features);
  }

  void update_and_publish(double phi_ref, double phi_src, const std::vector<double> & features)
  {
    try {
      const auto theta = estimator_->update(phi_ref, phi_src, features);
      std_msgs::msg::Float64MultiArray msg;
      msg.data = theta;
      theta_pub_->publish(msg);
    } catch (const std::exception & error) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "RLS更新をスキップ: %s", error.what());
    }
  }

  static std::vector<std::string> split(const std::string & line)
  {
    std::vector<std::string> cells;
    std::stringstream stream(line);
    std::string cell;
    while (std::getline(stream, cell, ',')) {
      cells.push_back(cell);
    }
    return cells;
  }

  void process_csv()
  {
    std::ifstream file(csv_path_);
    if (!file) {
      RCLCPP_ERROR(get_logger(), "CSVを開けません: %s", csv_path_.c_str());
      return;
    }
    std::string line;
    std::vector<std::string> header;
    while (std::getline(file, line)) {
      if (line.empty() || line[0] == '#') {
        continue;
      }
      header = split(line);
      break;
    }
    auto column = [&header](const std::string & name) -> int {
        const auto iterator = std::find(header.begin(), header.end(), name);
        return iterator == header.end() ? -1 : static_cast<int>(iterator - header.begin());
      };
    const auto ref_index = column("phi_hum_y");
    const auto src_index = column("phi_com_y");
    if (ref_index < 0 || src_index < 0) {
      RCLCPP_ERROR(get_logger(), "CSVにphi_hum_yまたはphi_com_y列がありません");
      return;
    }
    std::ofstream output;
    if (!csv_output_path_.empty()) {
      output.open(csv_output_path_);
      if (!output) {
        RCLCPP_ERROR(get_logger(), "RLS結果CSVを開けません: %s", csv_output_path_.c_str());
        return;
      }
      output << "sample";
      for (std::size_t i = 0; i < feature_indices_.size(); ++i) output << ",theta_" << i;
      output << "\n";
    }
    std::size_t sample = 0;
    while (std::getline(file, line)) {
      if (line.empty() || line[0] == '#') continue;
      const auto row = split(line);
      if (static_cast<std::size_t>(std::max(ref_index, src_index)) >= row.size()) continue;
      std::vector<double> features;
      bool valid = true;
      for (const auto index : feature_indices_) {
        if (index >= row.size()) { valid = false; break; }
        try {
          features.push_back(std::stod(row[index]));
        } catch (const std::exception &) {
          valid = false;
          break;
        }
      }
      if (!valid) continue;
      try {
        const auto theta = estimator_->update(
          std::stod(row[ref_index]), std::stod(row[src_index]), features);
        if (output) {
          output << sample++;
          for (const auto value : theta) output << "," << value;
          output << "\n";
        }
      } catch (const std::exception & error) {
        RCLCPP_ERROR(get_logger(), "CSV RLS更新に失敗: %s", error.what());
        return;
      }
    }
    RCLCPP_INFO(get_logger(), "CSVからRLS推定を完了しました: %s", csv_path_.c_str());
  }

  std::unique_ptr<RLSEstimator> estimator_;
  std::vector<std::size_t> feature_indices_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr theta_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sample_sub_;
  int phi_ref_index_{11};
  int phi_src_index_{10};
  std::string csv_path_;
  std::string csv_output_path_;
  bool process_csv_on_start_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RlsEstimatorNode>());
  rclcpp::shutdown();
  return 0;
}
