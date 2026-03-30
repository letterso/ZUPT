#include "zupt.h"

#include "cxxopts.hpp"

#include <algorithm>
#include <array>
#include <csignal>
#include <cctype>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
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

namespace {

volatile std::sig_atomic_t g_stop_requested = 0;

void signal_handler(int signal) {
  if (signal == SIGINT || signal == SIGTERM) {
    g_stop_requested = 1;
  }
}

enum class DataType { CSV = 0, BAG = 1 };
enum class DetectorSelection { GLRT = 0, MV = 1, MAG = 2, ARE = 3, ALL = 4 };

std::string to_lower_copy(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
  return value;
}

bool parseDataType(const std::string &data_type, DataType &parsed_type) {
  const std::string data_type_normalized = to_lower_copy(data_type);
  if (data_type_normalized == "csv") {
    parsed_type = DataType::CSV;
    return true;
  }
  if (data_type_normalized == "bag") {
    parsed_type = DataType::BAG;
    return true;
  }
  return false;
}

bool parseDetectorSelection(const std::string &detector,
                            DetectorSelection &selection) {
  const std::string detector_normalized = to_lower_copy(detector);
  if (detector_normalized == "glrt") {
    selection = DetectorSelection::GLRT;
    return true;
  }
  if (detector_normalized == "mv") {
    selection = DetectorSelection::MV;
    return true;
  }
  if (detector_normalized == "mag") {
    selection = DetectorSelection::MAG;
    return true;
  }
  if (detector_normalized == "are") {
    selection = DetectorSelection::ARE;
    return true;
  }
  if (detector_normalized == "all") {
    selection = DetectorSelection::ALL;
    return true;
  }
  return false;
}

ZUPT::IMUDetectorType to_detector_type(DetectorSelection selection) {
  switch (selection) {
  case DetectorSelection::GLRT:
    return ZUPT::IMUDetectorType::GLRT;
  case DetectorSelection::MV:
    return ZUPT::IMUDetectorType::MV;
  case DetectorSelection::MAG:
    return ZUPT::IMUDetectorType::MAG;
  case DetectorSelection::ARE:
    return ZUPT::IMUDetectorType::ARE;
  default:
    return ZUPT::IMUDetectorType::GLRT;
  }
}

std::string detector_name(DetectorSelection selection) {
  switch (selection) {
  case DetectorSelection::GLRT:
    return "glrt";
  case DetectorSelection::MV:
    return "mv";
  case DetectorSelection::MAG:
    return "mag";
  case DetectorSelection::ARE:
    return "are";
  case DetectorSelection::ALL:
    return "all";
  default:
    return "unknown";
  }
}

std::string mode_output_path(const std::string &base_output,
                             const std::string &mode,
                             bool append_mode_suffix) {
  if (!append_mode_suffix) {
    return base_output;
  }

  std::filesystem::path output_path(base_output);
  const std::string stem = output_path.stem().string();
  std::string extension = output_path.extension().string();
  if (extension.empty()) {
    extension = ".txt";
  }

  const std::string stem_non_empty = stem.empty() ? "output" : stem;
  const std::filesystem::path generated_name =
      stem_non_empty + "_" + mode + extension;

  if (!output_path.parent_path().empty()) {
    return (output_path.parent_path() / generated_name).string();
  }
  return generated_name.string();
}

bool parse_csv_row(const std::string &line, std::array<double, 6> &row) {
  std::stringstream ss(line);
  std::string item;
  size_t index = 0;
  while (std::getline(ss, item, ',')) {
    if (index >= row.size()) {
      return false;
    }
    try {
      row[index] = std::stod(item);
    } catch (const std::exception &) {
      return false;
    }
    ++index;
  }
  return index == row.size();
}

bool readCSV(const std::string &filename, std::vector<ImuData::Ptr> &data) {
  std::ifstream file(filename);
  if (!file.is_open()) {
    std::cerr << "无法打开文件: " << filename << std::endl;
    return false;
  }

  size_t line_number = 0;
  size_t skipped_lines = 0;
  std::string line;
  while (std::getline(file, line)) {
    ++line_number;
    if (line.empty()) {
      ++skipped_lines;
      continue;
    }

    std::array<double, 6> row{};
    if (!parse_csv_row(line, row)) {
      ++skipped_lines;
      std::cerr << "警告: 跳过非法 CSV 行 " << line_number << std::endl;
      continue;
    }

    Eigen::Vector3d acc, gyr;
    acc << row[0], row[1], row[2];
    gyr << row[3], row[4], row[5];
    ImuData::Ptr imu_data_ptr = std::make_shared<ImuData>(0.0, acc, gyr);
    data.push_back(imu_data_ptr);
  }

  if (skipped_lines > 0) {
    std::cerr << "提示: CSV 读取完成，跳过 " << skipped_lines << " 行异常数据"
              << std::endl;
  }

  return !data.empty();
}

#ifdef ENABLE_ROS2_BAG
bool readBag(const std::string &bag_uri, const std::string &topic,
             const std::string &storage_id, std::vector<ImuData::Ptr> &data) {
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
      if (g_stop_requested != 0) {
        break;
      }

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
    return false;
  }

  return !data.empty();
}
#endif

bool run_detector(const std::vector<ImuData::Ptr> &datas,
                  DetectorSelection selection, size_t window_size,
                  const std::string &score_output) {
  if (selection == DetectorSelection::ALL) {
    return false;
  }

  ZUPT::Ptr zupt_ptr = std::make_shared<ZUPT>();
  zupt_ptr->set_imu_param(9.8173, 0.01, 0.1 / 180.0 * M_PI);
  zupt_ptr->set_score_output_path(score_output);
  zupt_ptr->enable_score_logging(true);

  std::deque<Eigen::Vector3d> acc_bufs;
  std::deque<Eigen::Vector3d> gyo_bufs;

  size_t windows_count = 0;
  size_t valid_windows_count = 0;
  size_t zupt_detected_count = 0;

  for (const ImuData::Ptr &data : datas) {
    if (g_stop_requested != 0) {
      break;
    }

    acc_bufs.push_back(data->linear_acceleration);
    gyo_bufs.push_back(data->angular_velocity);
    if (acc_bufs.size() >= window_size) {
      ++windows_count;

      const auto result = zupt_ptr->detect_zupt_result(
          {acc_bufs.begin(), acc_bufs.end()}, {gyo_bufs.begin(), gyo_bufs.end()},
          to_detector_type(selection));

      if (result.valid) {
        ++valid_windows_count;
      }
      if (result.zupt_detected) {
        ++zupt_detected_count;
      }

      acc_bufs.pop_front();
      gyo_bufs.pop_front();
    }
  }

  std::cout << "detector=" << detector_name(selection)
            << ", windows=" << windows_count
            << ", valid_windows=" << valid_windows_count
            << ", zupt_detected=" << zupt_detected_count
            << ", output=" << score_output << std::endl;

  if (g_stop_requested != 0) {
    std::cerr << "提示: 检测流程被中断" << std::endl;
  }

  return windows_count > 0;
}

} // namespace

int main(int argc, char **argv) {
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  cxxopts::Options options(
      "main",
      "ZUPT demo. Support CSV and optional ROS2 bag input when compiled with "
      "ENABLE_ROS2_BAG=ON.");
  options.add_options()("h,help", "Show help")(
      "data_type",
      "Input data type: csv or bag",
      cxxopts::value<std::string>()->default_value("csv"))("input",
                                                             "Input file/bag path",
                                                             cxxopts::value<std::string>()->default_value("../data/imu_data.csv"))("detector",
                                                                                                                                        "Detector mode: glrt/mv/mag/are/all",
                                                                                                                                        cxxopts::value<std::string>()->default_value("mag"))("window_size",
                                                                                                                                                                                             "Sliding window size (>=3)",
                                                                                                                                                                                             cxxopts::value<size_t>()->default_value("3"))("score_output",
                                                                                                                                                                                                                                            "Score output path (all mode auto adds suffix)",
                                                                                                                                                                                                                                            cxxopts::value<std::string>()->default_value("output.txt"))("topic",
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
  const std::string detector_str = result["detector"].as<std::string>();
  const size_t window_size = result["window_size"].as<size_t>();
  const std::string score_output = result["score_output"].as<std::string>();
  const std::string topic = result["topic"].as<std::string>();
  const std::string storage_id = result["storage_id"].as<std::string>();

  DataType data_type;
  if (!parseDataType(data_type_str, data_type)) {
    std::cerr << "参数 --data_type 非法: " << data_type_str
              << "，仅支持 csv 或 bag" << std::endl;
    std::cout << options.help() << std::endl;
    return 1;
  }

  DetectorSelection detector_selection;
  if (!parseDetectorSelection(detector_str, detector_selection)) {
    std::cerr << "参数 --detector 非法: " << detector_str
              << "，仅支持 glrt/mv/mag/are/all" << std::endl;
    std::cout << options.help() << std::endl;
    return 1;
  }

  if (window_size < 3) {
    std::cerr << "参数 --window_size 必须 >= 3" << std::endl;
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

  if (score_output.empty()) {
    std::cerr << "参数 --score_output 不能为空" << std::endl;
    return 1;
  }

  // read data
  std::vector<ImuData::Ptr> datas;
#ifdef ENABLE_ROS2_BAG
  bool loaded_ok = false;
  if (data_type == DataType::BAG) {
    loaded_ok = readBag(input, topic, storage_id, datas);
  } else {
    loaded_ok = readCSV(input, datas);
  }
#else
  if (data_type == DataType::BAG) {
    std::cerr
        << "当前可执行文件未启用 ROS2 bag 支持，请在 CMake 中打开 ENABLE_ROS2_BAG"
        << std::endl;
    return 1;
  }
  const bool loaded_ok = readCSV(input, datas);
#endif

  if (!loaded_ok || datas.empty()) {
    std::cerr << "输入数据为空或读取失败，无法执行检测" << std::endl;
    return 1;
  }

  bool run_ok = true;
  if (detector_selection == DetectorSelection::ALL) {
    const std::array<DetectorSelection, 4> detectors = {
        DetectorSelection::GLRT, DetectorSelection::MV,
        DetectorSelection::MAG, DetectorSelection::ARE};
    for (const auto detector : detectors) {
      const std::string output_path =
          mode_output_path(score_output, detector_name(detector), true);
      if (!run_detector(datas, detector, window_size, output_path)) {
        run_ok = false;
      }
      if (g_stop_requested != 0) {
        break;
      }
    }
  } else {
    run_ok = run_detector(datas, detector_selection, window_size,
                          mode_output_path(score_output,
                                           detector_name(detector_selection),
                                           false));
  }

  if (!run_ok) {
    std::cerr << "检测过程未成功完成，请检查输入数据与参数" << std::endl;
    return 1;
  }

  if (g_stop_requested != 0) {
    std::cerr << "程序已按信号请求提前退出" << std::endl;
    return 130;
  }

  return 0;
}