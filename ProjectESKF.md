# 项目 ESKF 技术文档

> 基于 `src/my_nav2_robot/src/data_handle.cpp` 的实现分析，含 IMU 外参预处理方案

---

## 一、为什么选择 ESKF

### 1.1 项目需求

本系统是一个 RoboMaster 麦轮底盘的里程计系统，搭载两类传感器：

| 传感器 | 数据 | 频率 | 特性 |
|--------|------|------|------|
| IMU（陀螺仪+加速度计） | 角速度 `ω`、线加速度 `a` | 200 Hz | 高频、短期精确、长期漂移（零偏） |
| 轮速编码器 | 四轮角速度 `w_fl/w_fr/w_rl/w_rr` | 200 Hz | 直接测量、受打滑/地面影响 |

核心任务：**融合 IMU 与轮速数据，输出高频、低漂移的里程计（位姿 + 速度）**。

### 1.2 为什么不用 EKF

经典 EKF 直接估计状态本身（如位置、速度、姿态角），存在以下问题：

1. **姿态参数化问题**：姿态角（欧拉角）有奇异性（万向锁），四元数有过约束（4 参数描述 3 自由度），EKF 状态向量与自由度不匹配
2. **线性化精度差**：EKF 在**后验状态**附近线性化，当状态变化大时（快速转弯），泰勒展开的高阶截断误差显著
3. **协方差矩阵不对称**：旋转空间不是向量空间，直接在欧拉角/四元数上做加减会导致协方差矩阵物理意义混乱

### 1.3 ESKF 的优势

ESKF（Error-State Kalman Filter）的核心思想是：**不直接估计状态本身，而是估计状态的误差**。

| 对比项 | EKF | ESKF |
|--------|-----|------|
| 估计对象 | 全状态 $\boldsymbol{x}$ | 误差状态 $\delta\boldsymbol{x}$ |
| 线性化点 | 后验状态（变化大） | 误差状态（始终很小） |
| 线性化精度 | 低（大信号处展开） | 高（零点附近展开） |
| 姿态表示 | 欧拉角/四元数（3或4参数） | 旋转向量 $\delta\boldsymbol{\theta}$（3参数，无约束） |
| 运算空间 | 非线性流形 | 线性向量空间 $\mathbb{R}^{15}$ |
| 数值稳定性 | 差 | 好（误差小，协方差矩阵行为良好） |

**关键结论**：误差状态 $\delta\boldsymbol{x}$ 始终很小，在零点附近泰勒展开的精度远高于在大信号处展开。这使得 ESKF 的线性化模型比 EKF 更精确。

---

## 二、状态量定义与物理意义

### 2.1 IMU 姿态安装误差与预处理方案

IMU 安装时未严格对齐车体坐标系，存在 Pitch、Roll（以及 Yaw）方向的安装角度误差。这导致 IMU 坐标系相对车体坐标系有一个小角度旋转 $\boldsymbol{R}_{imu}^{body}$（IMU 系→车体系）。

**影响**：
1. **加速度投影偏移**：安装角误差导致重力分量泄漏到水平轴，静止时 $a_z \approx 9.8$ m/s² 泄漏到 x/y 轴，造成虚假加速度
2. **角速度投影偏移**：转弯时 $\omega_z$ 经安装角误差投影到 x/y 轴，造成 pitch/roll 漂移

**处理方式：预处理旋转而非 ESKF 状态扩展**

将 IMU 外参纳入 ESKF 状态向量会带来以下问题：
- **计算量增加**：状态维度从 15 增至 17，F/H/P/Q 矩阵全部增大
- **收敛风险**：外参为常量，可观测量依赖特定运动激励，若激励不足可能导致协方差发散或不收敛
- **复杂度增加**：F 矩阵新增 $\boldsymbol{F}_{\boldsymbol{v}\boldsymbol{\phi}}$ 和 $\boldsymbol{F}_{\boldsymbol{\theta}\boldsymbol{\phi}}$ 耦合项，增加线性化误差

因此，采用**预处理方案**：在数据进入 ESKF 之前，使用已标定的旋转矩阵 $\boldsymbol{R}_{imu}^{body}$ 直接将 IMU 原始数据旋转到车体系。ESKF 本身保持 15 维不变，完全不感知 IMU 安装误差。

$$\boldsymbol{a}_{body} = \boldsymbol{R}_{imu}^{body} \cdot \boldsymbol{a}_{imu}$$

$$\boldsymbol{\omega}_{body} = \boldsymbol{R}_{imu}^{body} \cdot \boldsymbol{\omega}_{imu}$$

其中 $\boldsymbol{R}_{imu}^{body}$ 为离线标定所得的 3×3 旋转矩阵：

$$\boldsymbol{R}_{imu}^{body} = \begin{bmatrix} 0.999892758 & -0.000146531 & 0.014644162 \\ -0.000146531 & 0.999799785 & 0.020009186 \\ -0.014644162 & -0.020009188 & 0.999692543 \end{bmatrix}$$

> 从矩阵可读出安装角：$\phi_x \approx -0.020$ rad ≈ -1.15°（Roll），$\phi_y \approx 0.015$ rad ≈ 0.84°（Pitch），$\phi_z \approx -0.00015$ rad ≈ -0.009°（Yaw）。Yaw 误差极小可忽略，Pitch/Roll 误差约 1°，足以造成显著的重力泄漏。

### 2.2 状态向量

ESKF 将真实状态 $\boldsymbol{x}_t$ 分解为**名义状态** $\boldsymbol{x}$ 和**误差状态** $\delta\boldsymbol{x}$：

$$\boldsymbol{x}_t = \boldsymbol{x} \oplus \delta\boldsymbol{x}$$

其中 $\oplus$ 对位置/速度/零偏是加法，对姿态是四元数乘法。

**名义状态**（5 组，共 16 参数，其中姿态 4 参数为四元数）：

| 符号 | 维度 | 物理意义 | 代码变量 |
|------|------|---------|---------|
| $\boldsymbol{p}$ | 3 | 世界系位置（车体中心） | `p_` |
| $\boldsymbol{v}$ | 3 | 世界系速度（车体中心） | `v_` |
| $\boldsymbol{q}$ | 4 | 姿态四元数（车体系→世界系） | `q_` |
| $\boldsymbol{b}_a$ | 3 | 加速度计零偏 | `b_a_` |
| $\boldsymbol{b}_g$ | 3 | 陀螺仪零偏 | `b_g_` |

**误差状态**（5 组，共 15 维——姿态用 3 维旋转向量）：

$$\delta\boldsymbol{x} = \begin{bmatrix} \delta\boldsymbol{p} \\ \delta\boldsymbol{v} \\ \delta\boldsymbol{\theta} \\ \delta\boldsymbol{b}_a \\ \delta\boldsymbol{b}_g \end{bmatrix} \in \mathbb{R}^{15}$$

| 分量 | 维度 | 索引 | 物理意义 |
|------|------|------|---------|
| $\delta\boldsymbol{p}$ | 3 | 0–2 | 位置误差（世界系，米） |
| $\delta\boldsymbol{v}$ | 3 | 3–5 | 速度误差（世界系，m/s） |
| $\delta\boldsymbol{\theta}$ | 3 | 6–8 | 姿态误差角（车体系，弧度） |
| $\delta\boldsymbol{b}_a$ | 3 | 9–11 | 加速度计零偏误差（m/s²） |
| $\delta\boldsymbol{b}_g$ | 3 | 12–14 | 陀螺仪零偏误差（rad/s） |

### 2.3 姿态误差的参数化

姿态误差使用**旋转向量** $\delta\boldsymbol{\theta} \in \mathbb{R}^3$ 而非四元数，原因：

1. **最小参数化**：3 参数描述 3 自由度，无过约束
2. **无奇异性**：小角度下 $\delta\boldsymbol{\theta}$ 是良好定义的
3. **线性空间**：$\delta\boldsymbol{\theta}$ 属于 $\mathbb{R}^3$，可直接进行协方差运算

误差四元数与旋转向量的关系：

$$\delta q \approx \begin{bmatrix} \frac{1}{2}\delta\boldsymbol{\theta} \\ 1 \end{bmatrix} \quad (\|\delta\boldsymbol{\theta}\| \ll 1)$$

真实姿态与名义姿态的关系：

$$\boldsymbol{q}_t = \boldsymbol{q} \otimes \delta\boldsymbol{q}$$

> **注意**：本实现中 $\delta\boldsymbol{\theta}$ 定义在**车体系**（右乘扰动），对应代码中的 `q_ * dq`。

### 2.4 IMU 测量模型（预处理后）

IMU 的实际测量值包含零偏和白噪声：

$$\boldsymbol{a}_m = \boldsymbol{a}_{imu} + \boldsymbol{b}_a + \boldsymbol{n}_a$$

$$\boldsymbol{\omega}_m = \boldsymbol{\omega}_{imu} + \boldsymbol{b}_g + \boldsymbol{n}_g$$

预处理阶段，先用标定矩阵将 IMU 原始数据旋转到车体系：

$$\boldsymbol{a}_m^{body} = \boldsymbol{R}_{imu}^{body} \cdot \boldsymbol{a}_m$$

$$\boldsymbol{\omega}_m^{body} = \boldsymbol{R}_{imu}^{body} \cdot \boldsymbol{\omega}_m$$

注意：零偏 $\boldsymbol{b}_a$, $\boldsymbol{b}_g$ 是 IMU 传感器固有的，**在 IMU 坐标系下标定**。因此补偿零偏应在旋转之前进行：

$$\boldsymbol{a}_{clean}^{body} = \boldsymbol{R}_{imu}^{body} \cdot (\boldsymbol{a}_m - \boldsymbol{b}_a)$$

$$\boldsymbol{\omega}_{clean}^{body} = \boldsymbol{R}_{imu}^{body} \cdot (\boldsymbol{\omega}_m - \boldsymbol{b}_g)$$

> **IMU 特性**：静止时加速度计 z 轴读数 ≈ +9.8 m/s²（重力反作用力），并非 0。经预处理旋转后，此重力分量被正确投影到车体系 z 轴，不会泄漏到水平轴。

### 2.5 姿态关系

车体姿态 $\boldsymbol{q}$ 描述的是车体系→世界系的旋转。经预处理后，IMU 数据已在车体系下表达，ESKF 不再感知 IMU 安装误差。

---

## 三、名义状态传播（predict）

### 3.1 连续时间运动方程

IMU 补偿零偏并经预处理旋转后输出车体系的"干净"加速度和角速度：

$$\boldsymbol{a}_{clean}^{body} = \boldsymbol{R}_{imu}^{body} \cdot (\boldsymbol{a}_m - \boldsymbol{b}_a)$$

$$\boldsymbol{\omega}_{clean}^{body} = \boldsymbol{R}_{imu}^{body} \cdot (\boldsymbol{\omega}_m - \boldsymbol{b}_g)$$

名义状态的连续时间微分方程：

$$\dot{\boldsymbol{p}} = \boldsymbol{v}$$

$$\dot{\boldsymbol{v}} = \boldsymbol{R}(\boldsymbol{q}) \cdot \boldsymbol{a}_{clean}^{body} + \boldsymbol{g}$$

$$\dot{\boldsymbol{q}} = \frac{1}{2}\boldsymbol{q} \otimes \boldsymbol{\omega}_{clean}^{body}$$

$$\dot{\boldsymbol{b}}_a = \boldsymbol{0}, \quad \dot{\boldsymbol{b}}_g = \boldsymbol{0} \quad (\text{零偏为慢时变常量})$$

其中：
- $\boldsymbol{R}(\boldsymbol{q})$：四元数对应的旋转矩阵（车体系→世界系）
- $\boldsymbol{g} = [0, 0, -9.8]^T$：世界系重力加速度

### 3.2 离散时间积分

采用一阶欧拉法（IMU 频率 200Hz，dt≈5ms，精度足够）：

$$\boldsymbol{p}_{k+1} = \boldsymbol{p}_k + \boldsymbol{v}_k \Delta t + \frac{1}{2}(\boldsymbol{R}_k \boldsymbol{a}_{clean}^{body} + \boldsymbol{g})\Delta t^2$$

$$\boldsymbol{v}_{k+1} = \boldsymbol{v}_k + (\boldsymbol{R}_k \boldsymbol{a}_{clean}^{body} + \boldsymbol{g})\Delta t$$

$$\boldsymbol{q}_{k+1} = \boldsymbol{q}_k \otimes \Delta\boldsymbol{q}(\boldsymbol{\omega}_{clean}^{body} \Delta t)$$

其中角增量四元数：

$$\Delta\boldsymbol{q} = \begin{bmatrix} \sin(\frac{\|\Delta\boldsymbol{\theta}\|}{2}) \frac{\Delta\boldsymbol{\theta}}{\|\Delta\boldsymbol{\theta}\|} \\ \cos(\frac{\|\Delta\boldsymbol{\theta}\|}{2}) \end{bmatrix}, \quad \Delta\boldsymbol{\theta} = \boldsymbol{\omega}_{clean}^{body} \Delta t$$

> **代码实现**（`data_handle.cpp`）：
> ```cpp
> // 补偿零偏 + 预处理旋转（IMU 系 → 车体系）
> Eigen::Vector3d acc_imu = acc_filtered_ - b_a_;
> Eigen::Vector3d w_imu   = gyro_filtered_ - b_g_;
> Eigen::Vector3d acc = R_imu_to_body_ * acc_imu;
> Eigen::Vector3d w   = R_imu_to_body_ * w_imu;
> // 名义状态积分
> p_ = p_ + v_ * dt + 0.5 * (q_ * acc + G_VEC_) * dt * dt;
> v_ = v_ + (q_ * acc + G_VEC_) * dt;
> ```

### 3.3 误差状态传播方程

对误差状态在名义状态附近做一阶泰勒展开：

$$\delta\dot{\boldsymbol{x}} = \boldsymbol{F} \delta\boldsymbol{x} + \boldsymbol{G}\boldsymbol{w}$$

其中系统矩阵 $\boldsymbol{F} \in \mathbb{R}^{15 \times 15}$：

$$\boldsymbol{F} = \begin{bmatrix} \boldsymbol{0} & \boldsymbol{I} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} \\ \boldsymbol{0} & \boldsymbol{0} & -\boldsymbol{R}[\boldsymbol{a}_c]_\times & -\boldsymbol{R} & \boldsymbol{0} \\ \boldsymbol{0} & \boldsymbol{0} & -[\boldsymbol{\omega}_c]_\times & \boldsymbol{0} & -\boldsymbol{I} \\ \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} \\ \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} \end{bmatrix}$$

其中 $\boldsymbol{a}_c = \boldsymbol{a}_{clean}^{body}$，$\boldsymbol{\omega}_c = \boldsymbol{\omega}_{clean}^{body}$。

**各分块的物理意义推导**：

#### $\delta\dot{\boldsymbol{v}}$ 对 $\delta\boldsymbol{\theta}$ 的偏导：$\boldsymbol{F}_{\boldsymbol{v}\boldsymbol{\theta}}$

速度传播方程：$\dot{\boldsymbol{v}} = \boldsymbol{R}(\boldsymbol{q})\boldsymbol{a}_c + \boldsymbol{g}$

当姿态有扰动 $\delta\boldsymbol{\theta}$（车体系，右乘）时：

$$\boldsymbol{R}_t = \boldsymbol{R}(\boldsymbol{I} - [\delta\boldsymbol{\theta}]_\times)$$

$$\delta\dot{\boldsymbol{v}} = -\boldsymbol{R}[\delta\boldsymbol{\theta}]_\times \boldsymbol{a}_c = -\boldsymbol{R}[\boldsymbol{a}_c]_\times \delta\boldsymbol{\theta}$$

$$\boxed{\boldsymbol{F}_{\boldsymbol{v}\boldsymbol{\theta}} = -\boldsymbol{R}[\boldsymbol{a}_c]_\times}$$

#### $\delta\dot{\boldsymbol{v}}$ 对 $\delta\boldsymbol{b}_a$ 的偏导：$-\boldsymbol{R}$

$$\delta\dot{\boldsymbol{v}} = -\boldsymbol{R}\delta\boldsymbol{b}_a$$

#### $\delta\dot{\boldsymbol{\theta}}$ 对 $\delta\boldsymbol{\theta}$ 和 $\delta\boldsymbol{b}_g$ 的偏导

$$\frac{\partial \delta\dot{\boldsymbol{\theta}}}{\partial \delta\boldsymbol{\theta}} = -[\boldsymbol{\omega}_c]_\times, \quad \frac{\partial \delta\dot{\boldsymbol{\theta}}}{\partial \delta\boldsymbol{b}_g} = -\boldsymbol{I}$$

### 3.4 离散化状态转移矩阵

一阶近似：$\boldsymbol{\Phi} \approx \boldsymbol{I} + \boldsymbol{F}\Delta t$

$$\boldsymbol{\Phi} = \begin{bmatrix} \boldsymbol{I} & \boldsymbol{I}\Delta t & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} \\ \boldsymbol{0} & \boldsymbol{I} & -\boldsymbol{R}[\boldsymbol{a}_c]_\times\Delta t & -\boldsymbol{R}\Delta t & \boldsymbol{0} \\ \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{I} - [\boldsymbol{\omega}_c]_\times\Delta t & \boldsymbol{0} & -\boldsymbol{I}\Delta t \\ \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{I} & \boldsymbol{0} \\ \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{I} \end{bmatrix}$$

> **代码实现**：
> ```cpp
> Eigen::Matrix<double, 15, 15> Fx = Eigen::Matrix<double, 15, 15>::Identity();
> Eigen::Matrix3d R = q_.toRotationMatrix();
> Fx.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
> Fx.block<3, 3>(3, 6) = -R * skew_symmetric(acc) * dt;
> Fx.block<3, 3>(3, 9) = -R * dt;
> Fx.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() - skew_symmetric(w * dt);
> Fx.block<3, 3>(6, 12) = -Eigen::Matrix3d::Identity() * dt;
> ```

### 3.5 协方差预测

$$\boldsymbol{P}_{k|k-1} = \boldsymbol{\Phi}\boldsymbol{P}_{k-1|k-1}\boldsymbol{\Phi}^T + \boldsymbol{Q}_d$$

其中 $\boldsymbol{Q}_d \in \mathbb{R}^{15 \times 15}$ 是离散化过程噪声矩阵：

```cpp
P_ = Fx * P_ * Fx.transpose() + Q_;
```

---

## 四、观测模型

本系统有**两个独立的观测更新**：`observeWheel()` 和 `observeZeroTilt()`。

### 4.1 轮速观测 `observeWheel()`

#### 4.1.1 观测量选取

观测量共 4 维，均在**车体系**下表示：

$$\boldsymbol{y}_{wheel} = \begin{bmatrix} v_x^{body} \\ v_y^{body} \\ v_z^{body} \\ \omega_z^{body} \end{bmatrix}$$

| 分量 | 来源 | 物理意义 |
|------|------|---------|
| $v_x^{body}$ | 麦轮运动学 | 车体系前进速度 |
| $v_y^{body}$ | 麦轮运动学 | 车体系横向速度 |
| $v_z^{body}$ | 约束为 0 | 地面机器人 z 速度≈0（零速观测） |
| $\omega_z^{body}$ | 麦轮运动学 | 车体系偏航角速度 |

#### 4.1.2 麦轮运动学

四轮麦克纳姆轮的运动学关系：

$$v_x = \frac{r}{4}(w_{fl} + w_{fr} + w_{rl} + w_{rr})$$

$$v_y = \frac{r}{4}(-w_{fl} + w_{fr} + w_{rl} - w_{rr})$$

$$\omega_z = \frac{r}{4(l_x + l_y)}(-w_{fl} + w_{fr} - w_{rl} + w_{rr})$$

参数：
- $r = 0.0815$ m（轮半径）
- $l_x + l_y \approx 0.3005$ m（轮对角线半距离之和）

#### 4.1.3 观测预测函数 $h(\boldsymbol{x})$

**速度部分**（前 3 维）：

ESKF 的速度状态 $\boldsymbol{v}$ 是车体中心的**世界系**速度，需要转到车体系：

$$h_v(\boldsymbol{x}) = \boldsymbol{R}^T \boldsymbol{v}$$

**角速度部分**（第 4 维）：

轮速反算的 $\omega_z$ 与车体系下已补偿零偏的陀螺仪 z 轴输出应该一致：

$$h_w(\boldsymbol{x}) = \omega_z^{body,clean}$$

经预处理旋转后，$\omega_z^{body,clean} = [\boldsymbol{R}_{imu}^{body}(\boldsymbol{\omega}_m - \boldsymbol{b}_g)]_z$。

综合观测预测值：

$$\boldsymbol{h}(\boldsymbol{x}) = \begin{bmatrix} \boldsymbol{R}^T\boldsymbol{v} \\ \omega_z^{body,clean} \end{bmatrix}$$

#### 4.1.4 雅可比矩阵 $\boldsymbol{H}$ 的推导

$\boldsymbol{H} = \frac{\partial \boldsymbol{h}}{\partial \delta\boldsymbol{x}} \in \mathbb{R}^{4 \times 15}$

**速度部分** $h_v = \boldsymbol{R}^T\boldsymbol{v}$ 对各误差状态的偏导：

**(a) 对 $\delta\boldsymbol{v}$ 的偏导**：$\boldsymbol{R}^T$

**(b) 对 $\delta\boldsymbol{\theta}$ 的偏导**：$-[\boldsymbol{v}_{body}]_\times$

**角速度部分** $h_w$ 对各误差状态的偏导：

**(c) 对 $\delta\boldsymbol{b}_g$ 的偏导**：$[0, 0, -1]$

**综合雅可比矩阵**：

$$\boldsymbol{H} = \begin{bmatrix} \boldsymbol{0}_{3\times3} & \boldsymbol{R}^T & -[\boldsymbol{v}_{body}]_\times & \boldsymbol{0}_{3\times3} & \boldsymbol{0}_{3\times3} \\ \boldsymbol{0}_{1\times3} & \boldsymbol{0}_{1\times3} & \boldsymbol{0}_{1\times3} & \boldsymbol{0}_{1\times3} & [0\;0\;{-1}] \end{bmatrix}$$

---

### 4.2 零倾斜观测 `observeZeroTilt()`

#### 4.2.1 设计动机

地面机器人在平地上运动时，pitch 和 roll 始终接近 0。但 ESKF 的 predict 步骤中，IMU 陀螺仪 x/y 轴噪声会导致 pitch/roll 累积漂移。此观测将 pitch/roll 拉回 0，防止旋转抖动和 z 轴漂移。

#### 4.2.2 观测量

$$\boldsymbol{y}_{tilt} = \begin{bmatrix} 0 \\ 0 \end{bmatrix}$$

物理含义：地面机器人的 pitch = 0, roll = 0。

#### 4.2.3 观测预测值

从当前名义四元数 $\boldsymbol{q}$ 提取 pitch/roll：

$$\boldsymbol{R} = \boldsymbol{R}(\boldsymbol{q})$$

$$pitch = -\arcsin(R_{31})$$

$$roll = \arctan2(R_{32}, R_{33})$$

$$\boldsymbol{h}_{tilt}(\boldsymbol{x}) = \begin{bmatrix} pitch \\ roll \end{bmatrix}$$

#### 4.2.4 雅可比矩阵推导

$$\boldsymbol{H}_{tilt} = \begin{bmatrix} \boldsymbol{0}_{1\times6} & 0 & 1 & 0 & \boldsymbol{0}_{1\times6} \\ \boldsymbol{0}_{1\times6} & 1 & 0 & 0 & \boldsymbol{0}_{1\times6} \end{bmatrix} \in \mathbb{R}^{2 \times 15}$$

---

## 五、卡尔曼滤波更新

### 5.1 卡尔曼增益

$$\boldsymbol{K}_k = \boldsymbol{P}_{k|k-1}\boldsymbol{H}^T(\boldsymbol{H}\boldsymbol{P}_{k|k-1}\boldsymbol{H}^T + \boldsymbol{R})^{-1}$$

其中 $\boldsymbol{R}$ 是观测噪声协方差矩阵：

- `observeWheel()`: $\boldsymbol{R} \in \mathbb{R}^{4\times4}$，对角阵 `Identity * 0.005`
- `observeZeroTilt()`: $\boldsymbol{R}_{tilt} \in \mathbb{R}^{2\times2}$，对角阵 `Identity * 0.005`

### 5.2 误差状态更新

$$\delta\boldsymbol{x} = \delta\boldsymbol{x} + \boldsymbol{K}_k(\boldsymbol{y} - \boldsymbol{h}(\boldsymbol{x}))$$

> 注意：使用 `+=` 而非 `=`，因为两个观测更新依次执行，误差状态逐步累积。

### 5.3 协方差更新（Joseph 形式）

标准形式 $\boldsymbol{P} = (\boldsymbol{I} - \boldsymbol{K}\boldsymbol{H})\boldsymbol{P}$ 在数值上可能产生非对称/非正定问题。Joseph 形式保证对称正定：

$$\boldsymbol{P}_{k|k} = (\boldsymbol{I} - \boldsymbol{K}\boldsymbol{H})\boldsymbol{P}_{k|k-1}(\boldsymbol{I} - \boldsymbol{K}\boldsymbol{H})^T + \boldsymbol{K}\boldsymbol{R}\boldsymbol{K}^T$$

---

## 六、状态注入与误差重置（injectAndReset）

### 6.1 误差注入

将估计的误差状态注入名义状态，得到"最优估计"：

$$\boldsymbol{p} \leftarrow \boldsymbol{p} + \delta\boldsymbol{p}$$

$$\boldsymbol{v} \leftarrow \boldsymbol{v} + \delta\boldsymbol{v}$$

$$\boldsymbol{q} \leftarrow \boldsymbol{q} \otimes \Delta\boldsymbol{q}(\delta\boldsymbol{\theta})$$

$$\boldsymbol{b}_a \leftarrow \boldsymbol{b}_a + \delta\boldsymbol{b}_a$$

$$\boldsymbol{b}_g \leftarrow \boldsymbol{b}_g + \delta\boldsymbol{b}_g$$

其中角度注入使用轴角→四元数：

$$\Delta\boldsymbol{q}(\delta\boldsymbol{\theta}) = \begin{bmatrix} \sin(\frac{\|\delta\boldsymbol{\theta}\|}{2})\frac{\delta\boldsymbol{\theta}}{\|\delta\boldsymbol{\theta}\|} \\ \cos(\frac{\|\delta\boldsymbol{\theta}\|}{2}) \end{bmatrix}$$

> **代码实现**：
> ```cpp
> p_ += delta_x_.segment<3>(0);
> v_ += delta_x_.segment<3>(3);
> b_a_ += delta_x_.segment<3>(9);
> b_g_ += delta_x_.segment<3>(12);
> Eigen::Vector3d dtheta = delta_x_.segment<3>(6);
> if (dtheta.norm() > 1e-10) {
>     Eigen::Quaterniond dq(Eigen::AngleAxisd(dtheta.norm(), dtheta.normalized()));
>     q_ = (q_ * dq).normalized();
> }
> ```

### 6.2 误差重置

注入完成后，误差状态归零：

$$\delta\boldsymbol{x} \leftarrow \boldsymbol{0}$$

---

## 七、完整 ESKF 流程

```
每帧 IMU+轮速数据到达（~200Hz）：
│
├── 0. 预处理（IMU 数据旋转到车体系）
│   ├── acc_body = R_imu_to_body * (acc_raw - b_a_)
│   └── w_body   = R_imu_to_body * (gyro_raw - b_g_)
│
├── 1. predict(dt)
│   ├── 名义状态积分：p_, v_, q_（使用预处理后的 acc_body, w_body）
│   ├── 计算状态转移矩阵 Fx (15×15)
│   └── 协方差预测：P = Fx·P·Fx^T + Q
│
├── 2. observeWheel()
│   ├── 麦轮运动学计算观测量 y = [vx, vy, 0, wz]
│   ├── 计算观测预测 h(x) = [R^T·v, w_body_z]
│   ├── 计算雅可比矩阵 H (4×15)
│   ├── 卡尔曼增益 K = P·H^T·(H·P·H^T + R)^{-1}
│   ├── 更新误差状态 δx += K·(y - h)
│   └── 更新协方差 P（Joseph 形式）
│
├── 3. observeZeroTilt()
│   ├── 提取 pitch, roll
│   ├── 观测量 y = [0, 0]，预测 h = [pitch, roll]
│   ├── 雅可比 H (2×15)
│   ├── K = P·H^T·(H·P·H^T + R_tilt)^{-1}
│   ├── δx += K·(y - h)
│   └── 更新协方差 P（Joseph 形式）
│
├── 4. injectAndReset()
│   ├── 将 δx 注入名义状态：p_, v_, q_, b_a_, b_g_
│   └── δx ← 0
│
└── 5. publishOdometry()
    ├── 发布 TF: odom → base_footprint
    └── 发布 /odom 消息
```

---

## 八、噪声参数说明

| 参数 | 代码值 | 维度 | 含义 | 调参指导 |
|------|--------|------|------|---------|
| $\boldsymbol{P}_0$ | `Identity * 0.01` | 15×15 | 初始状态不确定性 | 值越大→初始收敛越快 |
| $\boldsymbol{Q}$ | `Identity * 0.005` | 15×15 | 过程噪声 | IMU 精度越低应越大 |
| $\boldsymbol{R}$ | `4×4 Identity * 0.005` | 4×4 | 轮速观测噪声 | 轮速打滑越多应越大 |
| $\boldsymbol{R}_{tilt}$ | `2×2 Identity * 0.005` | 2×2 | 零倾斜观测噪声 | 值越小→约束越强→pitch/roll越稳定 |

**调参原则**：

- $\boldsymbol{Q}$ ↑ → 更信任观测（轮速），IMU 漂移被更快修正，但里程计更抖
- $\boldsymbol{R}$ ↑ → 更信任 IMU，轮速修正弱，直线性好但可能漂移
- $\boldsymbol{R}_{tilt}$ ↓ → 零倾斜约束强，pitch/roll 被锁死，但上坡时不灵活

---

## 九、坐标系约定

```
世界系 (odom)：          车体系 (base_footprint)：
  ↑ z                      ↑ z (上)
  |                        |
  +---→ y                  +---→ y (左)
 /                        /
↙ x                      ↙ x (前)

IMU 坐标系 → 车体系 转换（预处理旋转）：
  v_body = R_imu_to_body * v_imu
  R_imu_to_body 为离线标定的 3×3 旋转矩阵

IMU 输出在 IMU 坐标系（经 R_imu_to_body 预处理旋转到车体系）：
  gyro_x: 绕 IMU x 轴角速度 → 旋转后为车体 x 轴角速度
  gyro_y: 绕 IMU y 轴角速度 → 旋转后为车体 y 轴角速度
  gyro_z: 绕 IMU z 轴角速度 → 旋转后为车体 z 轴角速度
  acc_x: 沿 IMU x 轴加速度 → 旋转后为车体 x 轴加速度
  acc_y: 沿 IMU y 轴加速度 → 旋转后为车体 y 轴加速度
  acc_z: 沿 IMU z 轴加速度（静止时 ≈ +9.8 m/s²）

四元数 q_：车体系 → 世界系
  v_world = q_ * v_body
  v_body  = q_.inverse() * v_world = R^T * v_world
```

---

## 十、IMU 外参处理方案对比

### 10.1 两种方案

| 方案 | 状态维度 | 外参处理 | 优点 | 缺点 |
|------|---------|---------|------|------|
| **A：ESKF 状态扩展** | 17 维 | 将 $\delta\boldsymbol{\phi}$ 纳入状态向量 | 可在线标定外参 | 计算量增加；可观测量依赖运动激励，可能不收敛；F 矩阵新增耦合项 |
| **B：预处理旋转**（采用） | 15 维 | 离线标定 $\boldsymbol{R}_{imu}^{body}$，数据预处理时旋转 | 零额外计算；ESKF 完全不变；无收敛问题 | 需要离线标定；外参变化需重新标定 |

### 10.2 选择预处理方案的理由

1. **外参为常量**：IMU 固连在车体上，安装角不会随时间变化，无需在线估计
2. **已有标定数据**：通过 Allan 方差等离线方法可精确标定 $\boldsymbol{R}_{imu}^{body}$
3. **计算效率**：15×15 矩阵运算 vs 17×17，每帧节省约 28% 计算量
4. **收敛稳定性**：ESKF 状态扩展方案中，$\delta\boldsymbol{\phi}$ 的可观测量依赖重力泄漏和角速度耦合，若运动激励不足（如长时间静止），协方差可能发散
5. **代码简洁性**：ESKF 核心逻辑不变，仅在预处理阶段多一步矩阵乘法

### 10.3 预处理旋转对零偏标定的影响

零偏标定阶段，IMU 原始数据直接累积（不经过预处理旋转），标定得到的 $\boldsymbol{b}_a$, $\boldsymbol{b}_g$ 是 IMU 坐标系下的零偏。后续运行阶段，先减零偏再旋转到车体系：

$$\boldsymbol{a}_{clean}^{body} = \boldsymbol{R}_{imu}^{body} \cdot (\boldsymbol{a}_m - \boldsymbol{b}_a)$$

> 注意：$\boldsymbol{b}_a$ 是 IMU 系下的零偏，不应先旋转再减。等价验证：$\boldsymbol{R}(\boldsymbol{a}_m - \boldsymbol{b}_a) = \boldsymbol{R}\boldsymbol{a}_m - \boldsymbol{R}\boldsymbol{b}_a$，而 $\boldsymbol{R}\boldsymbol{b}_a$ 就是零偏在车体系下的表达。两种顺序等价，但先减零偏更符合物理直觉。

---

## 十一、关键设计决策总结

| 决策 | 理由 |
|------|------|
| 使用 ESKF 而非 EKF | 误差状态始终很小，线性化精度高；姿态用 3 维旋转向量表示，无过约束 |
| $\delta\boldsymbol{\theta}$ 定义在车体系（右乘） | 与 IMU 在车体系测量一致，雅可比矩阵推导更自然 |
| IMU 外参采用预处理旋转而非 ESKF 状态扩展 | 外参为常量，离线标定即可；避免状态扩展带来的计算增加和收敛风险 |
| 轮速观测在车体系 | 麦轮运动学直接输出车体系速度，无需旋转，避免坐标系混淆 |
| wz 纳入轮速观测 | 为 $\delta b_{g,z}$ 提供观测通路，帮助陀螺仪 z 轴零偏收敛 |
| 零倾斜观测 | 地面机器人 pitch/roll≈0 是强先验，直接约束角度抖动和 z 轴漂移 |
| Joseph 形式更新协方差 | 数值稳定，保证 $\boldsymbol{P}$ 对称正定 |
| 用 MCU 时间戳 `t_ms` 算 dt | 比 ROS `now()` 更精确（无 USB 传输延迟抖动） |
| 15 维状态向量 | IMU 姿态安装误差通过预处理旋转消除，不纳入 ESKF 状态 |
