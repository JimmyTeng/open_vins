# OpenVINS 中两种 CPI 实现对比文档

## 一、概述

本项目存在两套 Continuous Preintegration (CPI，连续预积分) 实现：

| 模块 | 路径 | 命名空间 |
|------|------|----------|
| **OpenVINS 原生 CPI** | `ov_core/src/cpi/` | `ov_core` |
| **数据预处理 CPI** | `ov_core/src/data_preprocessing/` | `data_preprocessing` |

两套实现均基于同一理论文献：Eckenhoff, Geneva, Huang 的 IJRR 2019 与 WAFR 2016，详见 http://udel.edu/~ghuang/papers/tr_cpi.pdf。

---

## 二、模块结构对比

### 2.1 OpenVINS 原生 CPI (`ov_core/src/cpi/`)

```
CpiBase.h          # 抽象基类，定义公共接口与成员
    ├── CpiV1.h/cpp   # Model 1：分段常数测量假设
    └── CpiV2.h/cpp   # Model 2：分段常数局部加速度假设
```

- **继承关系**：`CpiBase` → `CpiV1` / `CpiV2`
- **接口**：`setLinearizationPoints()`, `feed_IMU()`
- **使用方式**：预先设定线性化点，再按时间顺序逐对喂入 IMU 测量

### 2.2 数据预处理 CPI (`ov_core/src/data_preprocessing/`)

```
cpi_model.h        # 分段常数测量下的预积分核心模型
cpi_sequence.h/cpp # 序列管理 + 任意时间段预积分查询
```

- **组合关系**：`CpiSequence` 内部使用 `CpiModel` 计算
- **接口**：`CpiModel`: `InitBias()`, `FeedImu()`；`CpiSequence`: `SetBias()`, `AddImu()`, `GetPreintegration()`
- **使用方式**：按时间顺序添加 IMU，再对任意 `[t_start, t_end]` 请求预积分结果

---

## 三、理论模型对比

### 3.1 CpiV1 / CpiModel（对应 Model 1）

| 特性 | CpiV1 | CpiModel |
|------|-------|----------|
| 假设 | 分段常数**测量** (piecewise constant measurement) | 同左 |
| 线性化变量 | `b_w`, `b_a` | `b_w`, `b_a` |
| 重力/姿态 | 不参与线性化 | 同左 |
| 量纲 | t: 秒, w: rad/s, a: m/s² | 一致 |

两者在模型假设和输出量纲上一致。

### 3.2 CpiV2（Model 2）

| 特性 | 说明 |
|------|------|
| 假设 | 分段常数**局部加速度** (piecewise constant local acceleration) |
| 线性化变量 | `b_w`, `b_a`, `q_k_lin`, `grav` |
| 重力/姿态 | 使用 `q_k_lin`、`grav` 参与线性化 |
| 额外雅可比 | `O_a`, `O_b`（对姿态线性化点的雅可比） |
| 协方差 | 21×21 扩展状态，含 clone/marginalization |

数据预处理模块当前仅实现与 Model 1 等价的 `CpiModel`，未实现 Model 2。

---

## 四、接口与调用方式对比

### 4.1 OpenVINS CPI (CpiV1 示例)

```cpp
#include "cpi/CpiV1.h"

// 1. 构造并设置线性化点
auto cpi = std::make_shared<ov_core::CpiV1>(sigma_w, sigma_wb, sigma_a, sigma_ab, true);
cpi->setLinearizationPoints(b_w, b_a, q_k_lin, gravity);  // CpiV2 需要 q_k, grav

// 2. 按时间顺序逐对喂入 IMU
for (size_t i = 0; i + 1 < imu_times.size(); ++i) {
  cpi->feed_IMU(t_0, t_1, w_0, a_0, w_1, a_1);
}

// 3. 直接读取结果
double DT = cpi->DT;
Eigen::Vector3d alpha = cpi->alpha_tau;
Eigen::Vector3d beta = cpi->beta_tau;
// ...
```

### 4.2 数据预处理 CPI (CpiSequence)

```cpp
#include "data_preprocessing/cpi_sequence.h"

// 1. 构造并设置 bias
data_preprocessing::CpiSequence seq(sigma_w, sigma_wb, sigma_a, sigma_ab, true);
seq.SetBias(b_w, b_a);

// 2. 按时间顺序追加整条 IMU 序列
for (const auto& imu : imu_buffer) {
  seq.AddImu(imu.t, imu.w, imu.a);
}

// 3. 对任意 [t_start, t_end] 查询预积分
data_preprocessing::CpiResult result;
if (seq.GetPreintegration(t_start, t_end, &result)) {
  double DT = result.DT;
  auto alpha = result.alpha_tau;
  // ...
}
```

---

## 五、输出量对比

### 5.1 公共输出量（两者一致）

| 量 | 符号 | 含义 |
|----|------|------|
| DT | 积分时长 | 秒 |
| alpha_tau | 位移增量 | 起始体坐标系下 |
| beta_tau | 速度增量 | 起始体坐标系下 |
| q_k2tau, R_k2tau | 相对姿态 | 四元数 / 旋转矩阵 |
| J_q, J_a, J_b | 对陀螺 bias 的雅可比 | 姿态、alpha、beta |
| H_a, H_b | 对加表 bias 的雅可比 | alpha、beta |
| P_meas | 测量协方差 | 15×15 |

### 5.2 CpiV2 独有

- `O_a`, `O_b`：对 `q_k_lin` 的雅可比
- `setLinearizationPoints()` 需要 `q_k_lin`, `grav`

---

## 六、实现细节对比

### 6.1 协方差积分

| 实现 | 方法 | 状态维度 |
|------|------|----------|
| CpiV1 | RK4 数值积分 | 15×15 |
| CpiV2 | RK4 数值积分 | 21×21（含 clone 状态） |
| CpiModel | 依赖具体实现（cpi_model 仅有声明） | 预期 15×15 |

### 6.2 IMU 平均选项

两者均支持 `imu_avg`：

- **CpiV1**：对两端 `(w_m, a_m)` 取平均
- **CpiV2**：对 `w` 取平均；对**局部加速度**（经旋转与重力修正后）取平均
- **CpiModel**：与 CpiV1 相同逻辑（`imu_avg=false` 时第二组可传零）

### 6.3 小角速度处理

CpiV1/CpiV2 在 `mag_w < 0.008726646` 时使用 Taylor 展开避免数值不稳定，CpiModel 设计上应沿用类似策略。

---

## 七、集成与依赖

### 7.1 构建集成

| 模块 | CMake 集成 | 实际使用位置 |
|------|------------|--------------|
| ov_core CPI | `src/cpi/*.cpp` 已纳入主库 | `DynamicInitializer`, `test_dynamic_mle`, `Factor_ImuCPIv1` |
| data_preprocessing | **未**纳入主 CMakeLists.txt | 暂无上层引用 |

### 7.2 依赖关系

- **ov_core CPI**：依赖 `utils/quat_ops.h`，用于 `rot_2_quat`, `skew_x` 等
- **data_preprocessing**：依赖 `cpi_model.h`、`utils/quat_ops.h`（与 CpiV1 一致，无 Sophus 依赖）

---

## 八、适用场景与选择建议

| 场景 | 推荐 |
|------|------|
| 固定起止时间、顺序 feed IMU | CpiV1 / CpiV2 |
| 需要任意时间段预积分、多次查询 | CpiSequence + CpiModel |
| 需要局部加速度模型、姿态线性化 | CpiV2 |
| 仅需分段常数测量模型 | CpiV1 或 CpiModel |

---

## 九、总结

| 维度 | OpenVINS CPI | 数据预处理 CPI |
|------|--------------|----------------|
| 模型 | Model 1 (CpiV1) + Model 2 (CpiV2) | 仅 Model 1 (CpiModel) |
| 架构 | 继承（基类 + 多实现） | 组合（序列 + 模型） |
| 查询模式 | 固定区间、一次性顺序积分 | 任意 [t_start, t_end] 按需查询 |
| 工程状态 | 已集成、有 Ceres 因子 | 未完全接入构建、CpiModel 实现待补全 |
| 接口风格 | `setLinearizationPoints`, `feed_IMU` | `InitBias`/`SetBias`, `FeedImu`/`AddImu`, `GetPreintegration` |

两者在 Model 1 的假设和量纲上对齐，数据预处理 CPI 的优势在于「序列 + 任意时间窗查询」的接口设计，适合数据预处理或离线分析；当前工程中实际使用仍以 OpenVINS 原生 CPI 为主。
