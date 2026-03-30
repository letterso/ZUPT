#include "zupt.h"

namespace {
constexpr double kEpsilon = 1e-12;
}

ZUPT::ZUPT()
    : image_enabled_(false), feature_thd_(0.005), imu_enabled_(false),
      g_norm_(9.81007), sigma_acc_2_(0.1 * 0.1),
      sigma_gyo_2_((0.1 / 180.0 * M_PI) * (0.1 / 180.0 * M_PI)),
      zupt_thd_(100.0), score_logging_enabled_(true),
      out_file_("output.txt", std::ios::out | std::ios::trunc) {}

ZUPT::~ZUPT() = default;

bool ZUPT::is_finite_vector(const Eigen::Vector3d &vec) {
  return vec.allFinite();
}

bool ZUPT::has_valid_samples(const std::vector<Eigen::Vector3d> &acc_measures,
                             const std::vector<Eigen::Vector3d> &gyo_measures) {
  if (acc_measures.size() < 3 || acc_measures.size() != gyo_measures.size()) {
    return false;
  }

  for (size_t i = 0; i < acc_measures.size(); ++i) {
    if (!is_finite_vector(acc_measures[i]) || !is_finite_vector(gyo_measures[i])) {
      return false;
    }
  }

  return true;
}

void ZUPT::log_score(double score) {
  if (!score_logging_enabled_ || !out_file_.is_open()) {
    return;
  }
  out_file_ << score << '\n';
}

void ZUPT::set_score_output_path(const std::string &output_path) {
  out_file_.close();
  if (output_path.empty()) {
    return;
  }
  out_file_.open(output_path, std::ios::out | std::ios::trunc);
}

bool ZUPT::detect_zupt(const std::vector<Eigen::Vector3d> &acc_measures,
                       const std::vector<Eigen::Vector3d> &gyo_measures,
                       IMUDetectorType imu_detector_type) {
  const DetectionResult result =
      detect_zupt_result(acc_measures, gyo_measures, imu_detector_type);
  return result.valid && result.zupt_detected;
}

ZUPT::DetectionResult ZUPT::detect_zupt_result(
    const std::vector<Eigen::Vector3d> &acc_measures,
    const std::vector<Eigen::Vector3d> &gyo_measures,
    IMUDetectorType imu_detector_type) {
  DetectionResult result;
  if (!imu_enabled_) {
    return result;
  }

  if (!has_valid_samples(acc_measures, gyo_measures)) {
    return result;
  }

  const bool acc_var_valid =
      std::isfinite(sigma_acc_2_) && std::abs(sigma_acc_2_) > kEpsilon;
  const bool gyo_var_valid =
      std::isfinite(sigma_gyo_2_) && std::abs(sigma_gyo_2_) > kEpsilon;
  if (!std::isfinite(g_norm_)) {
    return result;
  }

  double score = std::numeric_limits<double>::quiet_NaN();
  switch (imu_detector_type) {
  case IMUDetectorType::GLRT:
    if (!acc_var_valid || !gyo_var_valid) {
      return result;
    }
    score = glrt_score(acc_measures, gyo_measures);
    break;
  case IMUDetectorType::MV:
    if (!acc_var_valid) {
      return result;
    }
    score = mv_score(acc_measures, gyo_measures);
    break;
  case IMUDetectorType::MAG:
    if (!acc_var_valid) {
      return result;
    }
    score = mag_score(acc_measures, gyo_measures);
    break;
  case IMUDetectorType::ARE:
    if (!gyo_var_valid) {
      return result;
    }
    score = are_score(acc_measures, gyo_measures);
    break;
  default:
    if (!acc_var_valid || !gyo_var_valid) {
      return result;
    }
    score = glrt_score(acc_measures, gyo_measures);
    break;
  }

  if (!std::isfinite(score)) {
    return result;
  }

  result.score = score;
  result.valid = true;
  result.zupt_detected = score <= zupt_thd_;
  log_score(score);
  return result;
}

double ZUPT::glrt_score(const std::vector<Eigen::Vector3d> &acc_measures,
                        const std::vector<Eigen::Vector3d> &gyo_measures) {
  const double size = static_cast<double>(acc_measures.size());
  const double size_invert = 1.0 / size;

  Eigen::Vector3d acc_avg = Eigen::Vector3d::Zero();
  for (size_t i = 0; i < acc_measures.size(); ++i) {
    acc_avg += acc_measures[i];
  }
  acc_avg *= size_invert;

  const double acc_norm = acc_avg.norm();
  if (acc_norm <= kEpsilon) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const Eigen::Vector3d a_norm = g_norm_ * acc_avg / acc_norm;

  double sum = 0;
  for (size_t i = 0; i < acc_measures.size(); ++i) {
    const Eigen::Vector3d acc_tmp = acc_measures[i] - a_norm;
    sum += gyo_measures[i].squaredNorm() / sigma_gyo_2_ +
           acc_tmp.squaredNorm() / sigma_acc_2_;
  }
  return sum / size;
}

double ZUPT::mv_score(const std::vector<Eigen::Vector3d> &acc_measures,
                      const std::vector<Eigen::Vector3d> &gyo_measures) {
  (void)gyo_measures;

  const double size = static_cast<double>(acc_measures.size());
  const double size_invert = 1.0 / size;

  Eigen::Vector3d acc_avg = Eigen::Vector3d::Zero();
  for (size_t i = 0; i < acc_measures.size(); ++i) {
    acc_avg += acc_measures[i];
  }
  acc_avg *= size_invert;

  double sum = 0;
  for (size_t i = 0; i < acc_measures.size(); ++i) {
    const Eigen::Vector3d acc_tmp = acc_measures[i] - acc_avg;
    sum += acc_tmp.squaredNorm();
  }
  return sum / (sigma_acc_2_ * size);
}

double ZUPT::mag_score(const std::vector<Eigen::Vector3d> &acc_measures,
                       const std::vector<Eigen::Vector3d> &gyo_measures) {
  (void)gyo_measures;

  const double size = static_cast<double>(acc_measures.size());

  double sum = 0;
  for (size_t i = 0; i < acc_measures.size(); ++i) {
    const double acc_tmp = acc_measures[i].norm() - g_norm_;
    sum += acc_tmp * acc_tmp;
  }
  return sum / (sigma_acc_2_ * size);
}

double ZUPT::are_score(const std::vector<Eigen::Vector3d> &acc_measures,
                       const std::vector<Eigen::Vector3d> &gyo_measures) {
  (void)acc_measures;

  const double size = static_cast<double>(gyo_measures.size());

  double sum = 0;
  for (size_t i = 0; i < gyo_measures.size(); ++i) {
    sum += gyo_measures[i].squaredNorm();
  }
  return sum / (sigma_gyo_2_ * size);
}