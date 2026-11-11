#pragma once

#include <cmath>
#include <stdexcept>
#include <vector>

class RLSEstimator
{
public:
  RLSEstimator(
    std::size_t parameter_count,
    double initial_covariance = 1.0e3,
    const std::vector<double> & initial_parameters = {})
  : theta_(parameter_count, 0.0),
    covariance_(parameter_count, std::vector<double>(parameter_count, 0.0))
  {
    if (parameter_count == 0 || !std::isfinite(initial_covariance) || initial_covariance <= 0.0) {
      throw std::invalid_argument("RLS requires a positive parameter count and covariance");
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
    double denominator = 1.0;
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
        updated[i][j] -= gain[i] * p_phi[j];
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
  const std::vector<std::vector<double>> & covariance() const noexcept { return covariance_; }

private:
  std::vector<double> theta_;
  std::vector<std::vector<double>> covariance_;
};
