CMAKE_MINIMUM_REQUIRED(VERSION 3.10)

PROJECT(ZUPTPorject VERSION 1.0.0
                  DESCRIPTION "zupt project"
                  LANGUAGES CXX)

MESSAGE("Identifying the OS...")
if(WIN32)
  MESSAGE("This is Windows.")
elseif(APPLE)
  MESSAGE("This is MacOS.")
elseif(UNIX)
  MESSAGE("This is Linux.")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb -DDEBUG")   # debug模式下 gdb相关选项
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")                   # release模式下 gdb相关选项
SET(CMAKE_BUILD_TYPE "RelWithDebInfo")  # 选项: Release Debug MinSizeRel RelWithDebInfo

find_package(Eigen3 REQUIRED)
find_package(rosbag REQUIRED)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib) # 静态库
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib) # 动态库
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin) # 可执行文件

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${rosbag_INCLUDE_DIRS}
  include)

add_executable(main
   ./src/main.cpp
   ./src/zupt.cpp)
target_link_libraries(main ${rosbag_LIBRARIES})

