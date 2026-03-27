
## 检测器说明

### 统一判决逻辑

- 先计算统计量 zupt_score
- 再与阈值 zupt_thd_ 比较
- 当 score <= 阈值时判定为静止（true），否则为运动（false）

### 原理与实现

| 方案 | 统计量定义（窗口长度 W） | 使用传感器 | 典型特点 |
|---|---|---|---|
| MAG（加速度幅值检测） | $T=\frac{1}{W\sigma_a^2}\sum(\|a\|-g)^2$ | 加速度 | 载体处于静止状态时，惯性测量器件中加速度计的三轴输出矢量和应稳定在当地重力加速度值附近，通过设定比力阈值检测零速状态。（实现最简单，但动态加速度或姿态变化时易误判） |
| MV（加速度滑动方差检测） | $T=\frac{1}{W\sigma_a^2}\sum\|a-\bar a\|^2$ | 加速度 | 载体处于静止状态时，惯性测量器件中加速度计三个轴的输出值的方差应该近似于0，设置一定大小的滑动窗口，通过比力方差阈值对加速度计三个轴分别检测零速状态。（计算简单，能抑制短时抖动，但对持续震动敏感） |
| ARE（角速度幅值检测） | $T=\frac{1}{W\sigma_g^2}\sum\|\omega\|^2$ | 角速度 | 同加速度幅值检测，载体处于静止状态时，惯性测量器件中陀螺仪的三轴输出矢量和应近似为零，通过设定角速率阈值检测零速状态。（对旋转敏感，线性振动场景较稳，但无法直接感知平移加速度） |
| GLRT（SHOE） | $T=\frac{1}{W}\sum\left(\frac{\|\omega\|^2}{\sigma_g^2}+\frac{\|a-g\hat{a}\|^2}{\sigma_a^2}\right)$ | 加速度 + 角速度 |信息最全，鲁棒性通常最好，但参数更敏感 |

### MATLAB 与当前 C++ 的对比

上游参考实现：

- GLRT: https://github.com/hcarlsso/zupt-aided-ins/blob/master/src/GLRT.m
- MV: https://github.com/hcarlsso/zupt-aided-ins/blob/master/src/MV.m
- MAG: https://github.com/hcarlsso/zupt-aided-ins/blob/master/src/MAG.m
- ARE: https://github.com/hcarlsso/zupt-aided-ins/blob/master/src/ARE.m
- 检测器包装与阈值判决: https://github.com/hcarlsso/zupt-aided-ins/blob/master/src/zero_velocity_detector.m

| 对比项 | 上游 MATLAB | 当前 C++ |
|---|---|---|
| 检测器集合 | GLRT / MV / MAG / ARE | GLRT / MV / MAG / ARE |
| 判决方式 | 在 wrapper 中按阈值 gamma 判决，并将窗口内样本标记为 false | 每次调用对一个窗口返回一次布尔判决（score 与 zupt_thd_ 比较） |
| 输入组织 | 全序列矩阵 u，滑窗遍历输出整段 T | 由外部传入当前窗口的 acc_measures 与 gyo_measures |
| 输出形式 | zupt 序列 + 统计量序列 T | 单次 bool 判决；同时将 score 写入 output.txt |
| 参数命名 | sigma_a, sigma_g, Window_size, gamma, g | sigma_acc_2_, sigma_gyo_2_, zupt_thd_, g_norm_ |

### 选型建议

| 场景 | 建议方案 | 原因 |
|---|---|---|
| 通用足式导航、追求稳健性 | GLRT | 同时利用加速度与角速度信息，综合判别能力更强 |
| 计算资源紧、实现要简单 | MV 或 MAG | 仅依赖加速度，公式简单 |
| 旋转状态变化明显、角速度质量高 | ARE | 对角速度能量变化最直接 |

注：四种方法的效果高度依赖噪声参数与阈值，跨数据集通常需要重新标定。