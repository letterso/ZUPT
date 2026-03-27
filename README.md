# ZUPTaidedINS

参考[ZUPTaidedINS](https://github.com/hcarlsso/ZUPT-aided-INS)使用CPP重新实现ZUPT判断

## 编译与运行

```bash
cd /home/robot/develop/cpp/ZUPT
cmake -S . -B build
cmake --build build -j
./build/bin/main
```

如需指定编译类型（例如 Release）：

```bash
cd /home/robot/develop/cpp/ZUPT
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/bin/main
```

## 附录

[zupt-detector](./docs/zupt-detector.md)