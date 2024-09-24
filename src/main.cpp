#include "zupt.h"

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>

#include <fstream>
#include <iostream>
#include <signal.h>

static bool end_this_program = false;
void signal_handler(int s) {
  std::cout << "exit" << std::endl;
  end_this_program = true;
}

void readCSV(const std::string &filename, std::vector<ImuData::Ptr> &data) {
  std::ifstream file(filename);
  if (!file.is_open()) {
    std::cerr << "无法打开文件: " << filename << std::endl;
    return;
  }

  std::string line;
  while (std::getline(file, line)) {
    std::stringstream ss(line);
    std::string item;
    std::array<double, 6> row;
    int index = 0;
    while (std::getline(ss, item, ',')) {
      row[index++] = std::stod(item);
    }
    Eigen::Vector3d acc, gyr;
    acc << row[0], row[1], row[2];
    gyr << row[3], row[4], row[5];
    ImuData::Ptr imu_data_ptr = std::make_shared<ImuData>(0, acc, gyr);
    data.push_back(imu_data_ptr);
  }

  file.close();
}

void readBag(const std::string &filename, std::vector<ImuData::Ptr> &data) {
  // read data
  rosbag::Bag bag;
  bag.open(filename, rosbag::bagmode::Read);
  std::vector<std::string> topics;
  topics.push_back("/livox/imu");
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  for (const auto &message : view) {
    if (message.getDataType() == "sensor_msgs/Imu") {
      sensor_msgs::ImuConstPtr msg = message.instantiate<sensor_msgs::Imu>();
      Eigen::Vector3d acc, gyr;
      acc << msg->linear_acceleration.x, msg->linear_acceleration.y,
          msg->linear_acceleration.z;
      gyr << msg->angular_velocity.x, msg->angular_velocity.y,
          msg->angular_velocity.z;
      ImuData::Ptr imu_data_ptr = std::make_shared<ImuData>(0, acc, gyr);
      data.push_back(imu_data_ptr);
    }
  }
}

int main(int argc, char **argv) {
  // zupt initialization
  ZUPT::Ptr zupt_ptr = std::make_shared<ZUPT>();
  zupt_ptr->set_imu_param(9.8173, 0.01, 0.1 / 180.0 * M_PI);

  // read data
  std::vector<ImuData::Ptr> datas;
  readCSV("../data/imu_data.csv", datas);

  // handle
  std::deque<Eigen::Vector3d> acc_bufs;
  std::deque<Eigen::Vector3d> gyo_bufs;
  for (ImuData::Ptr data : datas) {
    acc_bufs.push_back(data->linear_acceleration);
    gyo_bufs.push_back(data->angular_velocity);
    if (acc_bufs.size() > 2) {
      zupt_ptr->detect_zupt({acc_bufs.begin(), acc_bufs.end()},
                            {gyo_bufs.begin(), gyo_bufs.end()},
                            ZUPT::IMUDetectorType::MAG);
      acc_bufs.pop_front();
      gyo_bufs.pop_front();
    }
  }
  return 0;
}