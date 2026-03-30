#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "zupt.h"

#include <cmath>
#include <deque>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

struct ImuSample {
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyr = Eigen::Vector3d::Zero();
};

std::vector<ImuSample> load_imu_csv(const std::string &path) {
  std::ifstream file(path);
  if (!file.is_open()) {
    throw std::runtime_error("cannot open file: " + path);
  }

  std::vector<ImuSample> samples;
  std::string line;
  while (std::getline(file, line)) {
    if (line.empty()) {
      continue;
    }

    std::stringstream ss(line);
    std::string item;
    std::vector<double> values;
    values.reserve(6);

    while (std::getline(ss, item, ',')) {
      values.push_back(std::stod(item));
    }

    if (values.size() < 6) {
      continue;
    }

    ImuSample sample;
    sample.acc << values[0], values[1], values[2];
    sample.gyr << values[3], values[4], values[5];
    samples.push_back(sample);
  }

  return samples;
}

std::vector<double> load_scores(const std::string &path) {
  std::ifstream file(path);
  if (!file.is_open()) {
    throw std::runtime_error("cannot open score file: " + path);
  }

  std::vector<double> scores;
  std::string line;
  while (std::getline(file, line)) {
    if (line.empty()) {
      continue;
    }
    scores.push_back(std::stod(line));
  }
  return scores;
}

std::vector<double> run_detector_and_save_scores(
    const std::vector<ImuSample> &samples, ZUPT::IMUDetectorType mode,
    const std::string &output_path) {
  const std::filesystem::path output_file_path(output_path);
  if (output_file_path.has_parent_path()) {
    std::filesystem::create_directories(output_file_path.parent_path());
  }

  {
    ZUPT detector;
    detector.set_imu_param(9.8173, 0.01, 0.1 / 180.0 * M_PI);
    detector.enable_score_logging(true);
    detector.set_score_output_path(output_path);

    std::deque<Eigen::Vector3d> acc_buffer;
    std::deque<Eigen::Vector3d> gyr_buffer;

    for (const auto &sample : samples) {
      acc_buffer.push_back(sample.acc);
      gyr_buffer.push_back(sample.gyr);

      if (acc_buffer.size() > 2) {
        detector.detect_zupt({acc_buffer.begin(), acc_buffer.end()},
                             {gyr_buffer.begin(), gyr_buffer.end()}, mode);
        acc_buffer.pop_front();
        gyr_buffer.pop_front();
      }
    }
  }

  return load_scores(output_path);
}

void assert_scores_close(const std::vector<double> &current,
                         const std::vector<double> &baseline,
                         const std::string &mode_name,
                         double tolerance = 1e-9) {
  REQUIRE(current.size() == baseline.size());

  double max_abs_error = 0.0;
  size_t max_error_index = 0;
  for (size_t i = 0; i < current.size(); ++i) {
    const double abs_error = std::abs(current[i] - baseline[i]);
    if (abs_error > max_abs_error) {
      max_abs_error = abs_error;
      max_error_index = i;
    }
  }

  CHECK_MESSAGE(max_abs_error <= tolerance,
                mode_name << " max abs error=" << max_abs_error
                          << " at index=" << max_error_index
                          << ", tolerance=" << tolerance);
}

} // namespace

TEST_CASE("Baseline regression for all detector modes") {
  const auto samples = load_imu_csv("data/imu_data.csv");
  REQUIRE(samples.size() > 3);

  SUBCASE("GLRT") {
    const auto current_scores = run_detector_and_save_scores(
        samples, ZUPT::IMUDetectorType::GLRT,
        "build/test_outputs/glrt_current_scores.csv");
    const auto baseline_scores = load_scores("data/baseline/glrt_scores.csv");
    assert_scores_close(current_scores, baseline_scores, "GLRT");
  }

  SUBCASE("MV") {
    const auto current_scores = run_detector_and_save_scores(
        samples, ZUPT::IMUDetectorType::MV,
        "build/test_outputs/mv_current_scores.csv");
    const auto baseline_scores = load_scores("data/baseline/mv_scores.csv");
    assert_scores_close(current_scores, baseline_scores, "MV");
  }

  SUBCASE("MAG") {
    const auto current_scores = run_detector_and_save_scores(
        samples, ZUPT::IMUDetectorType::MAG,
        "build/test_outputs/mag_current_scores.csv");
    const auto baseline_scores = load_scores("data/baseline/mag_scores.csv");
    assert_scores_close(current_scores, baseline_scores, "MAG");
  }

  SUBCASE("ARE") {
    const auto current_scores = run_detector_and_save_scores(
        samples, ZUPT::IMUDetectorType::ARE,
        "build/test_outputs/are_current_scores.csv");
    const auto baseline_scores = load_scores("data/baseline/are_scores.csv");
    assert_scores_close(current_scores, baseline_scores, "ARE");
  }
}

TEST_CASE("Detector output is stable across repeated runs") {
  const auto samples = load_imu_csv("data/imu_data.csv");
  REQUIRE(samples.size() > 500);

  const std::vector<ImuSample> subset(samples.begin(), samples.begin() + 500);

  const auto scores_run_1 = run_detector_and_save_scores(
      subset, ZUPT::IMUDetectorType::GLRT,
      "build/test_outputs/stability_glrt_run1.csv");
  const auto scores_run_2 = run_detector_and_save_scores(
      subset, ZUPT::IMUDetectorType::GLRT,
      "build/test_outputs/stability_glrt_run2.csv");

  assert_scores_close(scores_run_1, scores_run_2, "GLRT stability", 0.0);
}

TEST_CASE("Detector handles invalid input robustly") {
  ZUPT detector;
  detector.set_imu_param(9.8173, 0.01, 0.1 / 180.0 * M_PI);
  detector.enable_score_logging(false);

  const std::vector<Eigen::Vector3d> acc_short = {
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
  const std::vector<Eigen::Vector3d> gyr_short = {
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
  CHECK_FALSE(
      detector.detect_zupt_result(acc_short, gyr_short, ZUPT::IMUDetectorType::MV)
          .valid);

  const std::vector<Eigen::Vector3d> acc_mismatch = {
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
  CHECK_FALSE(detector
                  .detect_zupt_result(acc_mismatch, gyr_short,
                                      ZUPT::IMUDetectorType::ARE)
                  .valid);

  std::vector<Eigen::Vector3d> acc_nan = {
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
  acc_nan[1](0) = std::numeric_limits<double>::quiet_NaN();
  const std::vector<Eigen::Vector3d> gyr_nan = {
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
  CHECK_FALSE(
      detector.detect_zupt_result(acc_nan, gyr_nan, ZUPT::IMUDetectorType::MAG)
          .valid);

  const std::vector<Eigen::Vector3d> acc_zero_avg = {
      Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Vector3d(-1.0, 0.0, 0.0),
      Eigen::Vector3d::Zero()};
  const std::vector<Eigen::Vector3d> gyr_zero_avg = {
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
  CHECK_FALSE(detector
                  .detect_zupt_result(acc_zero_avg, gyr_zero_avg,
                                      ZUPT::IMUDetectorType::GLRT)
                  .valid);
}

TEST_CASE("Detector returns valid zero score for ideal static data") {
  ZUPT detector;
  detector.set_imu_param(9.8173, 0.01, 0.1 / 180.0 * M_PI);
  detector.enable_score_logging(false);

  const std::vector<Eigen::Vector3d> acc = {
      Eigen::Vector3d(0.0, 0.0, -9.8173), Eigen::Vector3d(0.0, 0.0, -9.8173),
      Eigen::Vector3d(0.0, 0.0, -9.8173)};
  const std::vector<Eigen::Vector3d> gyr = {
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};

  const auto glrt = detector.detect_zupt_result(acc, gyr, ZUPT::IMUDetectorType::GLRT);
  const auto mv = detector.detect_zupt_result(acc, gyr, ZUPT::IMUDetectorType::MV);
  const auto mag = detector.detect_zupt_result(acc, gyr, ZUPT::IMUDetectorType::MAG);
  const auto are = detector.detect_zupt_result(acc, gyr, ZUPT::IMUDetectorType::ARE);

  REQUIRE(glrt.valid);
  REQUIRE(mv.valid);
  REQUIRE(mag.valid);
  REQUIRE(are.valid);

  CHECK(glrt.zupt_detected);
  CHECK(mv.zupt_detected);
  CHECK(mag.zupt_detected);
  CHECK(are.zupt_detected);

  CHECK(glrt.score == doctest::Approx(0.0));
  CHECK(mv.score == doctest::Approx(0.0));
  CHECK(mag.score == doctest::Approx(0.0));
  CHECK(are.score == doctest::Approx(0.0));
}
