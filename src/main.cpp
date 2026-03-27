#include "zupt.h"

#include "cxxopts.hpp"

#include <array>
#include <deque>
#include <fstream>
#include <iostream>
#include <sstream>
#include <signal.h>
#include <stdexcept>
#include <string>
#include <vector>

#ifdef ENABLE_ROS2_BAG
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/imu.hpp>
#endif

static bool end_this_program = false;
void signal_handler(int s) {
  std::cout << "exit" << std::endl;
  end_this_program = true;
}

enum class DataType { CSV = 0, BAG = 1 };

bool parseDataType(const std::string &data_type, DataType &parsed_type) {
  if (data_type == "csv") {
    parsed_type = DataType::CSV;
    return true;
  }
  if (data_type == "bag") {
    parsed_type = DataType::BAG;
    return true;
  }
  return false;
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

#ifdef ENABLE_ROS2_BAG
void readBag(const std::string &bag_uri, const std::string &topic,
             const std::string &storage_id,
             std::vector<ImuData::Ptr> &data) {
  rosbag2_cpp::readers::SequentialReader reader;
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = bag_uri;
  storage_options.storage_id = storage_id;

  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";

  try {
    reader.open(storage_options, converter_options);

    rclcpp::Serialization<sensor_msgs::msg::Imu> serializer;
    while (reader.has_next()) {
      auto bag_message = reader.read_next();
      if (bag_message->topic_name != topic) {
        continue;
      }

      rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
      sensor_msgs::msg::Imu msg;
      serializer.deserialize_message(&serialized_msg, &msg);

      Eigen::Vector3d acc, gyr;
      acc << msg.linear_acceleration.x, msg.linear_acceleration.y,
          msg.linear_acceleration.z;
      gyr << msg.angular_velocity.x, msg.angular_velocity.y,
          msg.angular_velocity.z;
      const double timestamp = static_cast<double>(msg.header.stamp.sec) +
                               static_cast<double>(msg.header.stamp.nanosec) *
                                   1e-9;

      ImuData::Ptr imu_data_ptr = std::make_shared<ImuData>(timestamp, acc, gyr);
      data.push_back(imu_data_ptr);
    }
  } catch (const std::exception &e) {
    std::cerr << "读取 ROS2 bag 失败: " << e.what() << std::endl;
  }
}
#endif

int main(int argc, char **argv) {
  cxxopts::Options options(
      "main",
      "ZUPT demo. Support CSV and optional ROS2 bag input when compiled with "
      "ENABLE_ROS2_BAG=ON.");
  options.add_options()("h,help", "Show help")(
      "data_type",
      "Input data type: csv or bag",
      cxxopts::value<std::string>()->default_value("csv"))("input",
                                                              "Input file/bag path",
                                                              cxxopts::value<std::string>()->default_value("../data/imu_data.csv"))("topic",
                                                                                                                                             "ROS2 bag topic name",
                                                                                                                                             cxxopts::value<std::string>()->default_value("/livox/imu"))("storage_id",
                                                                                                                                                                                                          "ROS2 bag storage id",
                                                                                                                                                                                                          cxxopts::value<std::string>()->default_value("sqlite3"));

  cxxopts::ParseResult result;
  try {
    result = options.parse(argc, argv);
  } catch (const cxxopts::exceptions::exception &e) {
    std::cerr << "命令行参数解析失败: " << e.what() << std::endl;
    std::cout << options.help() << std::endl;
    std::cout << "示例: ./bin/main --data_type csv --input ../data/imu_data.csv"
              << std::endl;
    std::cout
        << "示例: ./bin/main --data_type bag --input /tmp/imu_bag --topic /livox/imu --storage_id sqlite3"
        << std::endl;
    return 1;
  }

  if (result.count("help") > 0) {
    std::cout << options.help() << std::endl;
    std::cout << "示例: ./bin/main --data_type csv --input ../data/imu_data.csv"
              << std::endl;
    std::cout
        << "示例: ./bin/main --data_type bag --input /tmp/imu_bag --topic /livox/imu --storage_id sqlite3"
        << std::endl;
    return 0;
  }

  const std::string data_type_str = result["data_type"].as<std::string>();
  const std::string input = result["input"].as<std::string>();
  const std::string topic = result["topic"].as<std::string>();
  const std::string storage_id = result["storage_id"].as<std::string>();

  DataType data_type;
  if (!parseDataType(data_type_str, data_type)) {
    std::cerr << "参数 --data_type 非法: " << data_type_str
              << "，仅支持 csv 或 bag" << std::endl;
    std::cout << options.help() << std::endl;
    return 1;
  }

  if (input.empty()) {
    std::cerr << "参数 --input 不能为空" << std::endl;
    std::cout << options.help() << std::endl;
    return 1;
  }

  if (data_type == DataType::BAG && topic.empty()) {
    std::cerr << "当 --data_type=bag 时，参数 --topic 不能为空" << std::endl;
    std::cout << options.help() << std::endl;
    return 1;
  }

  if (data_type == DataType::BAG && storage_id.empty()) {
    std::cerr << "当 --data_type=bag 时，参数 --storage_id 不能为空"
              << std::endl;
    std::cout << options.help() << std::endl;
    return 1;
  }

  // zupt initialization
  ZUPT::Ptr zupt_ptr = std::make_shared<ZUPT>();
  zupt_ptr->set_imu_param(9.8173, 0.01, 0.1 / 180.0 * M_PI);

  // read data
  std::vector<ImuData::Ptr> datas;
#ifdef ENABLE_ROS2_BAG
  if (data_type == DataType::BAG) {
    readBag(input, topic, storage_id, datas);
  } else {
    readCSV(input, datas);
  }
#else
  if (data_type == DataType::BAG) {
    std::cerr
        << "当前可执行文件未启用 ROS2 bag 支持，请在 CMake 中打开 ENABLE_ROS2_BAG"
        << std::endl;
    return 1;
  }
  readCSV(input, datas);
#endif

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