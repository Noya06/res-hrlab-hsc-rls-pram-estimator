#include <algorithm>
#include <cmath>
#include <fstream>
#include <functional>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class RLSEstimator
{
public:
  RLSEstimator(
    std::size_t parameter_count,
    double initial_covariance,
    const std::vector<double> & initial_parameters,
    double forgetting_factor)
  : theta_(parameter_count, 0.0),
    covariance_(parameter_count, std::vector<double>(parameter_count, 0.0)),
    forgetting_factor_(forgetting_factor)
  {
    if (parameter_count == 0 || !std::isfinite(initial_covariance) || initial_covariance <= 0.0) {
      throw std::invalid_argument("RLS requires a positive parameter count and covariance");
    }
    if (!std::isfinite(forgetting_factor_) || forgetting_factor_ <= 0.0 ||
      forgetting_factor_ > 1.0)
    {
      throw std::invalid_argument("RLS forgetting_factor must be in (0, 1]");
    }
    if (!initial_parameters.empty() && initial_parameters.size() != parameter_count) {
      throw std::invalid_argument("Initial parameter dimension does not match RLS dimension");
    }
    if (!initial_parameters.empty()) {
      theta_ = initial_parameters;
    }
    for (std::size_t i = 0; i < parameter_count; ++i) {
      covariance_[i][i] = initial_covariance;
    }
  }

  std::vector<double> update(
    double phi_ref_val,
    double phi_src_val,
    const std::vector<double> & feature_vector)
  {
    const std::size_t n = theta_.size();
    if (feature_vector.size() != n) {
      throw std::invalid_argument("Feature vector dimension does not match RLS dimension");
    }
    const double y = phi_ref_val - phi_src_val;

    std::vector<double> p_phi(n, 0.0);
    double denominator = forgetting_factor_;
    for (std::size_t i = 0; i < n; ++i) {
      for (std::size_t j = 0; j < n; ++j) {
        p_phi[i] += covariance_[i][j] * feature_vector[j];
      }
      denominator += feature_vector[i] * p_phi[i];
    }
    if (!std::isfinite(denominator) || denominator <= 1.0e-12) {
      throw std::runtime_error("RLS covariance denominator is not positive and finite");
    }

    std::vector<double> gain(n, 0.0);
    double prediction = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
      gain[i] = p_phi[i] / denominator;
      prediction += feature_vector[i] * theta_[i];
    }
    const double innovation = y - prediction;
    for (std::size_t i = 0; i < n; ++i) {
      theta_[i] += gain[i] * innovation;
    }

    std::vector<std::vector<double>> updated = covariance_;
    for (std::size_t i = 0; i < n; ++i) {
      for (std::size_t j = 0; j < n; ++j) {
        updated[i][j] =
          (updated[i][j] - gain[i] * p_phi[j]) / forgetting_factor_;
      }
    }
    covariance_ = updated;
    for (std::size_t i = 0; i < n; ++i) {
      for (std::size_t j = i + 1; j < n; ++j) {
        const double symmetric = 0.5 * (covariance_[i][j] + covariance_[j][i]);
        covariance_[i][j] = symmetric;
        covariance_[j][i] = symmetric;
      }
      if (!std::isfinite(covariance_[i][i]) || covariance_[i][i] < 0.0) {
        covariance_[i][i] = 0.0;
      }
    }
    return theta_;
  }

  const std::vector<double> & parameters() const noexcept { return theta_; }

private:
  std::vector<double> theta_;
  std::vector<std::vector<double>> covariance_;
  double forgetting_factor_;
};

class RlsEstimatorNode final : public rclcpp::Node
{
public:
  RlsEstimatorNode()
  : Node("rls_estimator_node")
  {
    declare_parameter<int>("parameter_count", 10);
    declare_parameter<double>("initial_covariance", 1.0e3);
    declare_parameter<double>("forgetting_factor", 0.99);
    declare_parameter<bool>("update_enabled", false);
    declare_parameter<std::vector<double>>("initial_parameters", std::vector<double>{});
    declare_parameter<std::vector<int64_t>>("feature_indices", {4, 5, 12, 13, 14, 15, 16, 17, 18, 19});
    declare_parameter<std::vector<double>>("feature_scales", std::vector<double>{});
    declare_parameter<std::vector<std::string>>("parameter_names", std::vector<std::string>{});
    declare_parameter<int>("phi_ref_index", 11);
    declare_parameter<int>("phi_src_index", 10);
    declare_parameter<std::string>("csv_path", "");
    declare_parameter<bool>("process_csv_on_start", false);
    declare_parameter<std::string>("csv_output_path", "");

    int parameter_count;
    double initial_covariance;
    double forgetting_factor;
    get_parameter("update_enabled", update_enabled_);
    std::vector<double> initial_parameters;
    std::vector<int64_t> feature_indices;
    std::vector<double> feature_scales;
    std::vector<std::string> parameter_names;
    get_parameter("parameter_count", parameter_count);
    get_parameter("initial_covariance", initial_covariance);
    get_parameter("forgetting_factor", forgetting_factor);
    get_parameter("initial_parameters", initial_parameters);
    get_parameter("feature_indices", feature_indices);
    get_parameter("feature_scales", feature_scales);
    get_parameter("parameter_names", parameter_names);
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

    // 特徴量の正規化スケール設定
    if (feature_scales.empty()) {
      feature_scales_.assign(indices.size(), 1.0);
    } else if (feature_scales.size() != indices.size()) {
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

    if (parameter_names.empty()) {
      for (const auto idx : indices) {
        if (idx == 4) parameter_names_.push_back("theta_e2");
        else if (idx == 5) parameter_names_.push_back("theta_e3");
        else if (idx == 12) parameter_names_.push_back("theta_e2_abs_e2");
        else if (idx == 13) parameter_names_.push_back("theta_e3_abs_e3");
        else if (idx == 14) parameter_names_.push_back("theta_e2_abs_e3");
        else if (idx == 15) parameter_names_.push_back("theta_abs_e2_e3");
        else if (idx == 16) parameter_names_.push_back("theta_bias");
        else if (idx == 17) parameter_names_.push_back("theta_rho");
        else if (idx == 18) parameter_names_.push_back("theta_e2_delay");
        else if (idx == 19) parameter_names_.push_back("theta_e3_delay");
        else parameter_names_.push_back("theta_idx_" + std::to_string(idx));
      }
    } else if (parameter_names.size() != indices.size()) {
      throw std::invalid_argument("parameter_names size must match feature_indices size");
    } else {
      parameter_names_ = parameter_names;
    }

    estimator_ = std::make_unique<RLSEstimator>(
      static_cast<std::size_t>(parameter_count), initial_covariance, initial_parameters,
      forgetting_factor);
    feature_indices_ = std::move(indices);
    theta_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("rls/theta", 10);
    sample_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/joystick/calc_results", 100,
      std::bind(&RlsEstimatorNode::sample_callback, this, std::placeholders::_1));
    update_enabled_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/rls/update_enabled", rclcpp::QoS(1).transient_local(),
      std::bind(&RlsEstimatorNode::update_enabled_callback, this, std::placeholders::_1));

    if (process_csv_on_start_ && !csv_path_.empty()) {
      process_csv();
    }
    RCLCPP_INFO(
      this->get_logger(), "RLS推定ノードを起動しました (次元=%d, 忘却係数=%.6f)",
      parameter_count, forgetting_factor);
    {
      std::ostringstream oss;
      oss << "特徴量スケール: [";
      for (std::size_t i = 0; i < feature_scales_.size(); ++i) {
        if (i > 0) oss << ", ";
        oss << feature_scales_[i];
      }
      oss << "]";
      RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
    }
    RCLCPP_INFO(
      this->get_logger(), "RLSパラメータ更新: 待機中（トリガーを引いている間のみ更新します）");
  }

private:
  void update_enabled_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (update_enabled_ == msg->data) {
      return;
    }
    update_enabled_ = msg->data;
    RCLCPP_INFO(
      get_logger(), "トリガー入力によりRLSパラメータ更新を%sにしました。",
      update_enabled_ ? "有効（更新中）" : "無効（固定）");
  }

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
    // 特徴量の正規化
    for (std::size_t i = 0; i < features.size(); ++i) {
      features[i] /= feature_scales_[i];
    }
    update_and_publish(msg->data[phi_ref_index_], msg->data[phi_src_index_], features);
  }

  void update_and_publish(double phi_ref, double phi_src, const std::vector<double> & features)
  {
    try {
      const auto theta = update_enabled_
        ? estimator_->update(phi_ref, phi_src, features)
        : estimator_->parameters();
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
      for (const auto & name : parameter_names_) output << "," << name;
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
      // CSV特徴量の正規化
      for (std::size_t i = 0; i < features.size(); ++i) {
        features[i] /= feature_scales_[i];
      }
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
  std::vector<double> feature_scales_;
  std::vector<std::string> parameter_names_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr theta_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sample_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr update_enabled_sub_;
  int phi_ref_index_{11};
  int phi_src_index_{10};
  std::string csv_path_;
  std::string csv_output_path_;
  bool process_csv_on_start_{false};
  bool update_enabled_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RlsEstimatorNode>());
  rclcpp::shutdown();
  return 0;
}