#include "zupt.h"

ZUPT::ZUPT() {
  image_enabled_ = false;
  feature_thd_ = 0.005;

  imu_enabled_ = false;
  g_norm_ = 9.81007;
  sigma_gyo_2_ = (0.1 / 180.0 * M_PI) * (0.1 / 180.0 * M_PI);
  sigma_acc_2_ = 0.1 * 0.1;
  zupt_thd_ = 100.0;

  out_file_ = std::ofstream("output.txt");
}

ZUPT::~ZUPT() {}

bool ZUPT::detect_zupt(const std::vector<Eigen::Vector3d> &acc_measures,
                       const std::vector<Eigen::Vector3d> gyo_measures,
                       IMUDetectorType imu_detector_type) {
  bool zupt_detected = false;

  if (imu_enabled_) {
    if (acc_measures.size() < 3 || acc_measures.size() != gyo_measures.size())
      return false;

    switch (imu_detector_type) {
    case IMUDetectorType::GLRT: {
      zupt_detected = glrt_handle(acc_measures, gyo_measures);
      break;
    }
    case IMUDetectorType::MV: {
      zupt_detected = mv_handle(acc_measures, gyo_measures);
      break;
    }
    case IMUDetectorType::MAG: {
      zupt_detected = mag_handle(acc_measures, gyo_measures);
      break;
    }
    case IMUDetectorType::ARE: {
      zupt_detected = are_handle(acc_measures, gyo_measures);
      break;
    }
    default: {
      zupt_detected = glrt_handle(acc_measures, gyo_measures);
      break;
    }
    }
  }

  return zupt_detected;
}

bool ZUPT::glrt_handle(const std::vector<Eigen::Vector3d> &acc_measures,
                       const std::vector<Eigen::Vector3d> &gyo_measures) {
  auto size = static_cast<double>(acc_measures.size());
  double size_invert = 1.0 / size;

  Eigen::Vector3d acc_avg = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyo_avg = Eigen::Vector3d::Zero();
  for (size_t i = 0; i < acc_measures.size(); i++) {
    acc_avg = acc_avg + acc_measures[i];
    gyo_avg = gyo_avg + gyo_measures[i];
  }
  acc_avg = acc_avg * size_invert;
  gyo_avg = gyo_avg * size_invert;

  Eigen::Vector3d a_norm = g_norm_ * acc_avg / acc_avg.norm();
  double sum = 0;
  for (size_t i = 0; i < acc_measures.size(); i++) {
    Eigen::Vector3d acc_tmp = acc_measures[i] - a_norm;
    sum += (gyo_measures[i][0] * gyo_measures[i][0] +
            gyo_measures[i][1] * gyo_measures[i][1] +
            gyo_measures[i][2] * gyo_measures[i][2]) /
               sigma_gyo_2_ +
           (acc_tmp[0] * acc_tmp[0] + acc_tmp[1] * acc_tmp[1] +
            acc_tmp[2] * acc_tmp[2]) /
               sigma_acc_2_;
  }
  double zupt_score = sum / size;
  out_file_ << zupt_score << "\n";
  if (zupt_score > zupt_thd_)
    return false;

  return true;
}

bool ZUPT::mv_handle(const std::vector<Eigen::Vector3d> &acc_measures,
                     const std::vector<Eigen::Vector3d> &gyo_measures) {

  auto size = static_cast<double>(acc_measures.size());
  double size_invert = 1.0 / size;

  Eigen::Vector3d acc_avg = Eigen::Vector3d::Zero();
  for (size_t i = 0; i < acc_measures.size(); i++) {
    acc_avg = acc_avg + acc_measures[i];
  }
  acc_avg = acc_avg * size_invert;

  double sum = 0;
  for (size_t i = 0; i < acc_measures.size(); i++) {
    Eigen::Vector3d acc_tmp = acc_measures[i] - acc_avg;
    sum += (acc_tmp[0] * acc_tmp[0] + acc_tmp[1] * acc_tmp[1] +
            acc_tmp[2] * acc_tmp[2]);
  }
  double zupt_score = sum / (sigma_acc_2_ * size);
  out_file_ << zupt_score << "\n";
  if (zupt_score > zupt_thd_)
    return false;

  return true;
}

bool ZUPT::mag_handle(const std::vector<Eigen::Vector3d> &acc_measures,
                      const std::vector<Eigen::Vector3d> &gyo_measures) {
  auto size = static_cast<double>(acc_measures.size());

  double sum = 0;
  for (size_t i = 0; i < acc_measures.size(); i++) {
    double acc_tmp = acc_measures[i].norm() - g_norm_;
    sum += acc_tmp * acc_tmp;
  }
  double zupt_score = sum / (sigma_acc_2_ * size);
  out_file_ << zupt_score << "\n";
  if (zupt_score > zupt_thd_)
    return false;

  return true;
}

bool ZUPT::are_handle(const std::vector<Eigen::Vector3d> &acc_measures,
                      const std::vector<Eigen::Vector3d> &gyo_measures) {
  auto size = static_cast<double>(acc_measures.size());

  double sum = 0;
  for (size_t i = 0; i < gyo_measures.size(); i++) {
    double gyo_tmp = gyo_measures[i].norm();
    sum += gyo_tmp * gyo_tmp;
  }
  double zupt_score = sum / (sigma_gyo_2_ * size);
  out_file_ << zupt_score << "\n";
  if (zupt_score > zupt_thd_)
    return false;

  return true;
}