#include "zupt.h"

#include <array>
#include <deque>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

int main(int argc, char **argv) {
  if (argc < 2) {
    std::cerr << "usage: dump_baseline <mode>\n";
    return 1;
  }

  const std::string mode = argv[1];
  ZUPT::IMUDetectorType detector = ZUPT::IMUDetectorType::GLRT;
  if (mode == "glrt") {
    detector = ZUPT::IMUDetectorType::GLRT;
  } else if (mode == "mv") {
    detector = ZUPT::IMUDetectorType::MV;
  } else if (mode == "mag") {
    detector = ZUPT::IMUDetectorType::MAG;
  } else if (mode == "are") {
    detector = ZUPT::IMUDetectorType::ARE;
  } else {
    std::cerr << "unknown mode\n";
    return 1;
  }

  std::ifstream file("data/imu_data.csv");
  if (!file.is_open()) {
    std::cerr << "cannot open data/imu_data.csv\n";
    return 1;
  }

  std::vector<ImuData::Ptr> data;
  std::string line;
  while (std::getline(file, line)) {
    std::stringstream ss(line);
    std::string item;
    std::array<double, 6> row{};
    int idx = 0;
    while (std::getline(ss, item, ',')) {
      if (idx < 6) {
        row[idx] = std::stod(item);
      }
      ++idx;
    }
    if (idx < 6) {
      continue;
    }

    Eigen::Vector3d acc(row[0], row[1], row[2]);
    Eigen::Vector3d gyr(row[3], row[4], row[5]);
    data.push_back(std::make_shared<ImuData>(0.0, acc, gyr));
  }

  ZUPT zupt;
  zupt.set_imu_param(9.8173, 0.01, 0.1 / 180.0 * M_PI);

  std::deque<Eigen::Vector3d> acc_buf;
  std::deque<Eigen::Vector3d> gyr_buf;
  for (const auto &d : data) {
    acc_buf.push_back(d->linear_acceleration);
    gyr_buf.push_back(d->angular_velocity);
    if (acc_buf.size() > 2) {
      zupt.detect_zupt({acc_buf.begin(), acc_buf.end()},
                       {gyr_buf.begin(), gyr_buf.end()}, detector);
      acc_buf.pop_front();
      gyr_buf.pop_front();
    }
  }

  return 0;
}
