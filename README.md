# ZUPTaidedINS

参考[ZUPTaidedINS](https://github.com/hcarlsso/ZUPT-aided-INS)使用CPP重新实现ZUPT判断

## 编译与运行

```bash
cmake -S . -B build
# cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/bin/main
```

指定检测模式与输出文件（支持 `glrt/mv/mag/are/all`）：

```bash
./build/bin/main --data_type csv --input data/imu_data.csv --detector mag --window_size 3 --score_output build/test_outputs/main_mag_scores.txt
```

一次运行四种模式并分别保存输出：

```bash
./build/bin/main --data_type csv --input data/imu_data.csv --detector all --window_size 3 --score_output build/test_outputs/main_all.txt
```

`all` 模式会自动生成以下文件：

- `build/test_outputs/main_all_glrt.txt`
- `build/test_outputs/main_all_mv.txt`
- `build/test_outputs/main_all_mag.txt`
- `build/test_outputs/main_all_are.txt`

## 附录

[zupt-detector](./docs/zupt-detector.md)