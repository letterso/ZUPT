# Repository Guidelines

## Project Structure & Module Organization

This C++17 project implements zero-velocity detection with GLRT, MV, MAG, and ARE modes.

- `include/zupt.h` defines IMU data and the detector API; `src/zupt.cpp` implements the `zupt_core` library.
- `src/main.cpp` provides the CSV and optional ROS2 bag command-line interface.
- `tests/test_zupt.cpp` contains doctest tests; `data/imu_data.csv` and `data/baseline/` provide regression fixtures.
- `tools/dump_baseline.cpp` implements the `zupt_dump` score-generation utility.
- `docs/zupt-detector.md` explains detector formulas and behavior.
- `include/doctest.h` and `include/cxxopts.hpp` are bundled dependencies; avoid unrelated edits to them.

## Build, Test, and Development Commands

Install a C++17 compiler, CMake 3.10+, and Eigen3. Run commands from the repository root:

```bash
cmake -S . -B build -DBUILD_TESTING=ON  # Configure with tests
cmake --build build -j                # Build library, CLI, tools, tests
ctest --test-dir build --output-on-failure  # Run registered tests
./build/bin/main --data_type csv --input data/imu_data.csv --detector all --window_size 3 --score_output build/scores.txt
```

The CLI example writes separate score files for each mode. Window sizes must be at least three. For bag input, configure with `-DENABLE_ROS2_BAG=ON` in an environment providing `rclcpp`, `sensor_msgs`, and `rosbag2_cpp`; support is disabled by default.

## Coding Style & Naming Conventions

Follow surrounding C++ code: two-space indentation, braces on the declaration line, and descriptive names. Prefer `snake_case` for new functions and variables, trailing underscores for private members, and existing type conventions such as `DetectionResult` and `IMUDetectorType`. Preserve local naming when editing older mixed-style code. No formatter or linter configuration is checked in.

## Testing Guidelines

Add descriptive doctest `TEST_CASE` entries in `tests/test_zupt.cpp`; use `SUBCASE` for detector variants. Cover numerical behavior, invalid inputs, and repeatability when relevant. Regression tests compare all four modes against baseline scores with absolute tolerance `1e-9`. Explain intentional baseline changes. Generated test scores belong in `build/test_outputs/`. No coverage percentage is configured.

## Commit & Pull Request Guidelines

History uses short, descriptive English or Chinese subjects, such as `add doctest`; no mandatory prefix convention is evident. Keep commits focused. PRs should explain the change, link relevant issues, report build/test results, and describe effects on detector formulas, thresholds, CLI behavior, or baselines. Update documentation when behavior changes. Exclude generated scores and build artifacts.
