#include <Eigen/Core>
#include <Eigen/Dense>

#include <fstream>
#include <iostream>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

struct ImuData {
  double timestamp = 0;
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  Eigen::Matrix3d orientation_covariance = Eigen::Matrix3d::Zero();
  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
  Eigen::Matrix3d angular_velocity_covariance = Eigen::Matrix3d::Zero();
  Eigen::Vector3d linear_acceleration = Eigen::Vector3d::Zero();
  Eigen::Matrix3d linear_acceleration_covariance = Eigen::Matrix3d::Zero();

  ImuData() = default;
  ImuData(double t, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr)
      : timestamp(t), linear_acceleration(acc), angular_velocity(gyr) {}

  using Ptr = std::shared_ptr<ImuData>;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class ZUPT {
private:
  // IMAGE
  bool image_enabled_;
  double feature_thd_;

  // IMU
  bool imu_enabled_;
  double g_norm_;
  double sigma_acc_2_, sigma_gyo_2_;
  double zupt_thd_;

  // debug
  bool score_logging_enabled_;
  std::ofstream out_file_;

  // https://github.com/hcarlsso/ZUPT-aided-INS
  static bool is_finite_vector(const Eigen::Vector3d &vec);
  static bool has_valid_samples(
      const std::vector<Eigen::Vector3d> &acc_measures,
      const std::vector<Eigen::Vector3d> &gyo_measures);
  void log_score(double score);
  double glrt_score(const std::vector<Eigen::Vector3d> &acc_measures,
                    const std::vector<Eigen::Vector3d> &gyo_measures);
  double mv_score(const std::vector<Eigen::Vector3d> &acc_measures,
                  const std::vector<Eigen::Vector3d> &gyo_measures);
  double mag_score(const std::vector<Eigen::Vector3d> &acc_measures,
                   const std::vector<Eigen::Vector3d> &gyo_measures);
  double are_score(const std::vector<Eigen::Vector3d> &acc_measures,
                   const std::vector<Eigen::Vector3d> &gyo_measures);

public:
  typedef std::shared_ptr<ZUPT> Ptr;

  ZUPT();
  ~ZUPT();

  enum class IMUDetectorType { GLRT = 0, MV = 1, MAG = 2, ARE = 3 };

  struct DetectionResult {
    bool zupt_detected = false;
    bool valid = false;
    double score = std::numeric_limits<double>::quiet_NaN();
  };

  void set_imu_param(double g_norm, double sigma_acc, double sigma_gyo) {
    g_norm_ = g_norm;
    sigma_acc_2_ = sigma_acc * sigma_acc;
    sigma_gyo_2_ = sigma_gyo * sigma_gyo;
    imu_enabled_ = true;
  }

  void set_image_param(double feature_thd) {
    feature_thd_ = feature_thd;
    image_enabled_ = true;
  }

  void set_score_output_path(const std::string &output_path);

  void enable_score_logging(bool enabled) { score_logging_enabled_ = enabled; }

  bool detect_zupt(const std::vector<Eigen::Vector3d> &acc_measures,
                   const std::vector<Eigen::Vector3d> &gyo_measures,
                   IMUDetectorType imu_detector_type = IMUDetectorType::GLRT);

  DetectionResult
  detect_zupt_result(const std::vector<Eigen::Vector3d> &acc_measures,
                     const std::vector<Eigen::Vector3d> &gyo_measures,
                   IMUDetectorType imu_detector_type = IMUDetectorType::GLRT);
};