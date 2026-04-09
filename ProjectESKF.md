# 项目 ESKF 技术文档

> 基于 `src/my_nav2_robot/src/data_handle.cpp` 的完整实现分析

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

### 2.1 状态向量

ESKF 将真实状态 $\boldsymbol{x}_t$ 分解为**名义状态** $\boldsymbol{x}$ 和**误差状态** $\delta\boldsymbol{x}$：

$$\boldsymbol{x}_t = \boldsymbol{x} \oplus \delta\boldsymbol{x}$$

其中 $\oplus$ 对位置/速度/零偏是加法，对姿态是四元数乘法。

**名义状态**（5 组，共 17 参数，其中姿态 4 参数为四元数）：

| 符号 | 维度 | 物理意义 | 代码变量 |
|------|------|---------|---------|
| $\boldsymbol{p}$ | 3 | 世界系位置 | `p_` |
| $\boldsymbol{v}$ | 3 | 世界系速度 | `v_` |
| $\boldsymbol{q}$ | 4 | 姿态四元数（车体系→世界系） | `q_` |
| $\boldsymbol{b}_a$ | 3 | 加速度计零偏 | `b_a_` |
| $\boldsymbol{b}_g$ | 3 | 陀螺仪零偏 | `b_g_` |

**误差状态**（5 组，共 15 维——姿态用 3 维旋转向量表示）：

$$\delta\boldsymbol{x} = \begin{bmatrix} \delta\boldsymbol{p} \\ \delta\boldsymbol{v} \\ \delta\boldsymbol{\theta} \\ \delta\boldsymbol{b}_a \\ \delta\boldsymbol{b}_g \end{bmatrix} \in \mathbb{R}^{15}$$

| 分量 | 维度 | 索引 | 物理意义 |
|------|------|------|---------|
| $\delta\boldsymbol{p}$ | 3 | 0–2 | 位置误差（世界系，米） |
| $\delta\boldsymbol{v}$ | 3 | 3–5 | 速度误差（世界系，m/s） |
| $\delta\boldsymbol{\theta}$ | 3 | 6–8 | 姿态误差角（车体系，弧度） |
| $\delta\boldsymbol{b}_a$ | 3 | 9–11 | 加速度计零偏误差（m/s²） |
| $\delta\boldsymbol{b}_g$ | 3 | 12–14 | 陀螺仪零偏误差（rad/s） |

### 2.2 姿态误差的参数化

姿态误差使用**旋转向量** $\delta\boldsymbol{\theta} \in \mathbb{R}^3$ 而非四元数，原因：

1. **最小参数化**：3 参数描述 3 自由度，无过约束
2. **无奇异性**：小角度下 $\delta\boldsymbol{\theta}$ 是良好定义的
3. **线性空间**：$\delta\boldsymbol{\theta}$ 属于 $\mathbb{R}^3$，可直接进行协方差运算

误差四元数与旋转向量的关系：

$$\delta q \approx \begin{bmatrix} \frac{1}{2}\delta\boldsymbol{\theta} \\ 1 \end{bmatrix} \quad (\|\delta\boldsymbol{\theta}\| \ll 1)$$

真实姿态与名义姿态的关系：

$$\boldsymbol{q}_t = \boldsymbol{q} \otimes \delta\boldsymbol{q}$$

> **注意**：本实现中 $\delta\boldsymbol{\theta}$ 定义在**车体系**（右乘扰动），对应代码中的 `q_ * dq`。

### 2.3 IMU 测量模型

IMU 测量值包含真实值、零偏和白噪声：

$$\boldsymbol{a}_m = \boldsymbol{a}_{true} + \boldsymbol{b}_a + \boldsymbol{n}_a$$

$$\boldsymbol{\omega}_m = \boldsymbol{\omega}_{true} + \boldsymbol{b}_g + \boldsymbol{n}_g$$

- $\boldsymbol{a}_m$：加速度计测量值（车体系，含重力反作用力）
- $\boldsymbol{\omega}_m$：陀螺仪测量值（车体系）
- $\boldsymbol{b}_a, \boldsymbol{b}_g$：零偏（慢时变，由 ESKF 在线估计）
- $\boldsymbol{n}_a, \boldsymbol{n}_g$：高斯白噪声

> **IMU 特性**：静止时加速度计 z 轴读数 ≈ +9.8 m/s²（重力反作用力），并非 0。

---

## 三、名义状态传播（predict）

### 3.1 连续时间运动方程

IMU 补偿零偏后输出"干净"的加速度和角速度：

$$\boldsymbol{a}_{clean} = \boldsymbol{a}_m - \boldsymbol{b}_a, \quad \boldsymbol{\omega}_{clean} = \boldsymbol{\omega}_m - \boldsymbol{b}_g$$

名义状态的连续时间微分方程：

$$\dot{\boldsymbol{p}} = \boldsymbol{v}$$

$$\dot{\boldsymbol{v}} = \boldsymbol{R}(\boldsymbol{q}) \cdot \boldsymbol{a}_{clean} + \boldsymbol{g}$$

$$\dot{\boldsymbol{q}} = \frac{1}{2}\boldsymbol{q} \otimes \boldsymbol{\omega}_{clean}$$

其中：
- $\boldsymbol{R}(\boldsymbol{q})$：四元数对应的旋转矩阵（车体系→世界系）
- $\boldsymbol{g} = [0, 0, -9.8]^T$：世界系重力加速度
- $\boldsymbol{R} \cdot \boldsymbol{a}_{clean} + \boldsymbol{g}$：将车体系加速度转到世界系后加重力，静止时 = $\boldsymbol{R} \cdot [0,0,+9.8]^T + [0,0,-9.8]^T = \boldsymbol{0}$

### 3.2 离散时间积分

采用一阶欧拉法（IMU 频率 200Hz，dt≈5ms，精度足够）：

$$\boldsymbol{p}_{k+1} = \boldsymbol{p}_k + \boldsymbol{v}_k \Delta t + \frac{1}{2}(\boldsymbol{R}_k \boldsymbol{a}_{clean} + \boldsymbol{g})\Delta t^2$$

$$\boldsymbol{v}_{k+1} = \boldsymbol{v}_k + (\boldsymbol{R}_k \boldsymbol{a}_{clean} + \boldsymbol{g})\Delta t$$

$$\boldsymbol{q}_{k+1} = \boldsymbol{q}_k \otimes \Delta\boldsymbol{q}(\boldsymbol{\omega}_{clean} \Delta t)$$

其中角增量四元数：

$$\Delta\boldsymbol{q} = \begin{bmatrix} \sin(\frac{\|\Delta\boldsymbol{\theta}\|}{2}) \frac{\Delta\boldsymbol{\theta}}{\|\Delta\boldsymbol{\theta}\|} \\ \cos(\frac{\|\Delta\boldsymbol{\theta}\|}{2}) \end{bmatrix}, \quad \Delta\boldsymbol{\theta} = \boldsymbol{\omega}_{clean} \Delta t$$

> 代码实现（`data_handle.cpp:109-115`）：
> ```cpp
> p_ = p_ + v_ * dt + 0.5 * (q_ * acc + G_VEC_) * dt * dt;
> v_ = v_ + (q_ * acc + G_VEC_) * dt;
> Eigen::Vector3d dtheta = w * dt;
> q_ = q_ * Eigen::Quaterniond(Eigen::AngleAxisd(dtheta.norm(), dtheta.normalized()));
> ```

### 3.3 误差状态传播方程

对误差状态在名义状态附近做一阶泰勒展开：

$$\delta\dot{\boldsymbol{x}} = \boldsymbol{F} \delta\boldsymbol{x} + \boldsymbol{G}\boldsymbol{w}$$

其中系统矩阵 $\boldsymbol{F}$：

$$\boldsymbol{F} = \begin{bmatrix} \boldsymbol{0} & \boldsymbol{I} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} \\ \boldsymbol{0} & \boldsymbol{0} & -\boldsymbol{R}[\boldsymbol{a}_{clean}]_\times & -\boldsymbol{R} & \boldsymbol{0} \\ \boldsymbol{0} & \boldsymbol{0} & -[\boldsymbol{\omega}_{clean}]_\times & \boldsymbol{0} & -\boldsymbol{I} \\ \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} \\ \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} \end{bmatrix}_{15 \times 15}$$

**各分块的物理意义推导**：

#### $\delta\dot{\boldsymbol{v}}$ 对 $\delta\boldsymbol{\theta}$ 的偏导：$-\boldsymbol{R}[\boldsymbol{a}_{clean}]_\times$

速度传播方程：$\dot{\boldsymbol{v}} = \boldsymbol{R}(\boldsymbol{q})\boldsymbol{a}_{clean} + \boldsymbol{g}$

当姿态有扰动 $\boldsymbol{q}_t = \boldsymbol{q} \otimes \delta\boldsymbol{q}$ 时（车体系扰动）：

$$\boldsymbol{R}_t = \boldsymbol{R}(\boldsymbol{I} - [\delta\boldsymbol{\theta}]_\times) = \boldsymbol{R} - \boldsymbol{R}[\delta\boldsymbol{\theta}]_\times$$

$$\dot{\boldsymbol{v}}_t = (\boldsymbol{R} - \boldsymbol{R}[\delta\boldsymbol{\theta}]_\times)\boldsymbol{a}_{clean} + \boldsymbol{g}$$

$$\delta\dot{\boldsymbol{v}} = \dot{\boldsymbol{v}}_t - \dot{\boldsymbol{v}} = -\boldsymbol{R}[\delta\boldsymbol{\theta}]_\times \boldsymbol{a}_{clean} = -\boldsymbol{R}[\boldsymbol{a}_{clean}]_\times \delta\boldsymbol{\theta}$$

> 利用了恒等式 $[\boldsymbol{a}]_\times \boldsymbol{b} = -[\boldsymbol{b}]_\times \boldsymbol{a}$

#### $\delta\dot{\boldsymbol{v}}$ 对 $\delta\boldsymbol{b}_a$ 的偏导：$-\boldsymbol{R}$

加速度计零偏有误差 $\delta\boldsymbol{b}_a$ 时：

$$\boldsymbol{a}_{clean}^* = \boldsymbol{a}_m - (\boldsymbol{b}_a + \delta\boldsymbol{b}_a) = \boldsymbol{a}_{clean} - \delta\boldsymbol{b}_a$$

$$\delta\dot{\boldsymbol{v}} = \boldsymbol{R}(\boldsymbol{a}_{clean} - \delta\boldsymbol{b}_a) + \boldsymbol{g} - (\boldsymbol{R}\boldsymbol{a}_{clean} + \boldsymbol{g}) = -\boldsymbol{R}\delta\boldsymbol{b}_a$$

#### $\delta\dot{\boldsymbol{\theta}}$ 对 $\delta\boldsymbol{\theta}$ 的偏导：$-[\boldsymbol{\omega}_{clean}]_\times$

角速度传播中，当姿态有扰动时，旋转向量的变化率为：

$$\delta\dot{\boldsymbol{\theta}} = -[\boldsymbol{\omega}_{clean}]_\times \delta\boldsymbol{\theta} + \delta\boldsymbol{\omega}$$

这来自旋转向量的运动学方程，其中 $\delta\boldsymbol{\omega} = -\delta\boldsymbol{b}_g$ 是角速度误差。

#### $\delta\dot{\boldsymbol{\theta}}$ 对 $\delta\boldsymbol{b}_g$ 的偏导：$-\boldsymbol{I}$

陀螺仪零偏有误差时：$\boldsymbol{\omega}_{clean}^* = \boldsymbol{\omega}_{clean} - \delta\boldsymbol{b}_g$

$$\delta\dot{\boldsymbol{\theta}} = -\delta\boldsymbol{b}_g$$

### 3.4 离散化状态转移矩阵

一阶近似：$\boldsymbol{\Phi} \approx \boldsymbol{I} + \boldsymbol{F}\Delta t$

$$\boldsymbol{\Phi} = \begin{bmatrix} \boldsymbol{I} & \boldsymbol{I}\Delta t & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} \\ \boldsymbol{0} & \boldsymbol{I} & -\boldsymbol{R}[\boldsymbol{a}_{clean}]_\times\Delta t & -\boldsymbol{R}\Delta t & \boldsymbol{0} \\ \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{I} - [\boldsymbol{\omega}_{clean}]_\times\Delta t & \boldsymbol{0} & -\boldsymbol{I}\Delta t \\ \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{I} & \boldsymbol{0} \\ \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{I} \end{bmatrix}$$

> 代码实现（`data_handle.cpp:118-127`）：
> ```cpp
> Eigen::Matrix<double, 15, 15> Fx = Eigen::Matrix<double, 15, 15>::Identity();
> Fx.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;            // ∂δp/∂δv
> Fx.block<3, 3>(3, 6) = -R * skew_symmetric(acc) * dt;               // ∂δv/∂δθ
> Fx.block<3, 3>(3, 9) = -R * dt;                                      // ∂δv/∂δb_a
> Fx.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() - skew_symmetric(w * dt); // ∂δθ/∂δθ
> Fx.block<3, 3>(6, 12) = -Eigen::Matrix3d::Identity() * dt;          // ∂δθ/∂δb_g
> ```

### 3.5 协方差预测

$$\boldsymbol{P}_{k|k-1} = \boldsymbol{\Phi}\boldsymbol{P}_{k-1|k-1}\boldsymbol{\Phi}^T + \boldsymbol{Q}_d$$

其中 $\boldsymbol{Q}_d$ 是离散化过程噪声矩阵。代码中使用常数 $\boldsymbol{Q}$：

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

> 代码实现（`data_handle.cpp:151-159`）：
> ```cpp
> constexpr double kWheel = 0.0815 / 4.0;
> constexpr double kWz    = 0.0815 / (4.0 * 0.3005);
> y(0) = kWheel * ( wheel_v[0] + wheel_v[1] + wheel_v[2] + wheel_v[3]);  // vx
> y(1) = kWheel * (-wheel_v[0] + wheel_v[1] + wheel_v[2] - wheel_v[3]);  // vy
> y(2) = 0.0;                                                               // vz=0
> y(3) = kWz   * (-wheel_v[0] + wheel_v[1] - wheel_v[2] + wheel_v[3]);    // wz
> ```

#### 4.1.3 观测预测函数 $h(\boldsymbol{x})$

**速度部分**（前 3 维）：

将世界系速度 $\boldsymbol{v}$ 转到车体系：

$$h_v(\boldsymbol{x}) = \boldsymbol{R}^T \boldsymbol{v}$$

其中 $\boldsymbol{R} = \boldsymbol{R}(\boldsymbol{q})$ 是车体系→世界系的旋转矩阵，$\boldsymbol{R}^T$ 是世界系→车体系。

**角速度部分**（第 4 维）：

轮速反算的 $\omega_z$ 与 IMU 测量（已补偿零偏）的陀螺仪 z 轴输出应该一致：

$$h_w(\boldsymbol{x}) = \omega_{z,gyro} - b_{g,z}$$

综合观测预测值：

$$\boldsymbol{h}(\boldsymbol{x}) = \begin{bmatrix} \boldsymbol{R}^T\boldsymbol{v} \\ \omega_{z,gyro} - b_{g,z} \end{bmatrix}$$

> 代码实现（`data_handle.cpp:167-177`）：
> ```cpp
> Eigen::Matrix3d R_T = R.transpose();
> Eigen::Vector3d v_body = R_T * v_;
> h_x.head<3>() = v_body;
> h_x(3) = msg->angular_velocity.z - b_g_.z();
> ```

#### 4.1.4 雅可比矩阵 $\boldsymbol{H}$ 的推导

$\boldsymbol{H} = \frac{\partial \boldsymbol{h}}{\partial \delta\boldsymbol{x}} \in \mathbb{R}^{4 \times 15}$

**速度部分** $h_v = \boldsymbol{R}^T\boldsymbol{v}$ 对各误差状态的偏导：

**(a) 对 $\delta\boldsymbol{v}$ 的偏导**：

$\boldsymbol{v}_t = \boldsymbol{v} + \delta\boldsymbol{v}$，$\boldsymbol{R}$ 不受 $\delta\boldsymbol{v}$ 影响：

$$\frac{\partial h_v}{\partial \delta\boldsymbol{v}} = \boldsymbol{R}^T$$

**(b) 对 $\delta\boldsymbol{\theta}$ 的偏导**：

当姿态有扰动 $\delta\boldsymbol{\theta}$（车体系，右乘）时：

$$\boldsymbol{R}_t = \boldsymbol{R}(\boldsymbol{I} - [\delta\boldsymbol{\theta}]_\times)$$

$$\boldsymbol{R}_t^T = (\boldsymbol{I} - [\delta\boldsymbol{\theta}]_\times)^T \boldsymbol{R}^T = (\boldsymbol{I} + [\delta\boldsymbol{\theta}]_\times)\boldsymbol{R}^T$$

$$h_v(\boldsymbol{x}_t) = (\boldsymbol{I} + [\delta\boldsymbol{\theta}]_\times)\boldsymbol{R}^T\boldsymbol{v} = \boldsymbol{R}^T\boldsymbol{v} + [\delta\boldsymbol{\theta}]_\times \boldsymbol{R}^T\boldsymbol{v}$$

$$\frac{\partial h_v}{\partial \delta\boldsymbol{\theta}} = -[\boldsymbol{R}^T\boldsymbol{v}]_\times = -[\boldsymbol{v}_{body}]_\times$$

> 利用了 $[\delta\boldsymbol{\theta}]_\times \boldsymbol{a} = -[\boldsymbol{a}]_\times \delta\boldsymbol{\theta}$

**角速度部分** $h_w = \omega_{z,gyro} - b_{g,z}$ 对各误差状态的偏导：

**(c) 对 $\delta\boldsymbol{\theta}$ 的偏导**：

角速度在车体系直接测量，旋转误差 $\delta\boldsymbol{\theta}$ 对其影响为二阶小量，忽略：

$$\frac{\partial h_w}{\partial \delta\boldsymbol{\theta}} \approx \boldsymbol{0}$$

**(d) 对 $\delta\boldsymbol{b}_g$ 的偏导**：

$b_{g,z}^t = b_{g,z} + \delta b_{g,z}$，所以：

$$\frac{\partial h_w}{\partial \delta b_{g,z}} = -1$$

**综合雅可比矩阵**：

$$\boldsymbol{H} = \begin{bmatrix} \boldsymbol{0}_{3\times3} & \boldsymbol{R}^T & -[\boldsymbol{v}_{body}]_\times & \boldsymbol{0}_{3\times3} & \boldsymbol{0}_{3\times3} \\ \boldsymbol{0}_{1\times3} & \boldsymbol{0}_{1\times3} & \boldsymbol{0}_{1\times3} & \boldsymbol{0}_{1\times3} & [0\;0\;{-1}] \end{bmatrix}$$

> 代码实现（`data_handle.cpp:180-200`）：
> ```cpp
> H.block<3, 3>(0, 3) = R_T;         // ∂h_v/∂δv
> H.block<3, 3>(0, 6) = v_anti;      // ∂h_v/∂δθ = -[v_body]×
> H(3, 14) = -1.0;                    // ∂h_w/∂δb_g_z
> ```

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

> 代码实现（`data_handle.cpp:229-236`）：
> ```cpp
> double pitch = -std::asin(std::clamp(R(2, 0), -1.0, 1.0));
> double roll  =  std::atan2(R(2, 1), R(2, 2));
> Eigen::Vector2d y = Eigen::Vector2d::Zero();    // 约束目标
> Eigen::Vector2d h_x(pitch, roll);                // 当前名义值
> ```

#### 4.2.4 雅可比矩阵推导

旋转矩阵使用 ZYX 欧拉角分解：

$$\boldsymbol{R} = \boldsymbol{R}_z(yaw) \cdot \boldsymbol{R}_y(-pitch) \cdot \boldsymbol{R}_x(roll)$$

当存在车体系扰动 $\delta\boldsymbol{\theta}$ 时：

$$\boldsymbol{R}_t = \boldsymbol{R}(\boldsymbol{I} - [\delta\boldsymbol{\theta}]_\times)$$

对 pitch（= $-\arcsin(R_{31})$）：

$$R_{31}^t = R_{31} - R_{33}\delta\theta_y + R_{32}\delta\theta_x$$

当 pitch≈0, roll≈0 时，$R_{33} \approx 1, R_{32} \approx 0$：

$$\delta(pitch) = -\frac{\partial(-\arcsin(R_{31}))}{\partial R_{31}} \cdot \delta R_{31} \approx \delta\theta_y$$

对 roll（= $\arctan2(R_{32}, R_{33})$）：

当 pitch≈0 时：

$$\delta(roll) \approx \delta\theta_x$$

因此雅可比矩阵为：

$$\boldsymbol{H}_{tilt} = \begin{bmatrix} \boldsymbol{0}_{1\times6} & 0 & 1 & 0 & \boldsymbol{0}_{1\times6} \\ \boldsymbol{0}_{1\times6} & 1 & 0 & 0 & \boldsymbol{0}_{1\times6} \end{bmatrix}$$

> 代码实现（`data_handle.cpp:244-251`）：
> ```cpp
> H(0, 7) = 1.0;  // ∂pitch/∂δθ_y
> H(1, 6) = 1.0;  // ∂roll/∂δθ_x
> ```

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

> 代码实现（`data_handle.cpp:211-215`, `261-265`）

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

> 代码实现（`data_handle.cpp:269-284`）：
> ```cpp
> p_ += delta_x_.segment<3>(0);
> v_ += delta_x_.segment<3>(3);
> b_a_ += delta_x_.segment<3>(9);
> b_g_ += delta_x_.segment<3>(12);
> // 角度注入：q_ = q_ * dq
> Eigen::Quaterniond dq(Eigen::AngleAxisd(dtheta.norm(), dtheta.normalized()));
> q_ = (q_ * dq).normalized();
> ```

### 6.2 误差重置

注入完成后，误差状态归零：

$$\delta\boldsymbol{x} \leftarrow \boldsymbol{0}$$

> **理论细节**：严格来说，注入后协方差矩阵需要做一步重置变换：
> $$\boldsymbol{P}_{new} = \boldsymbol{G}_{reset}\boldsymbol{P}\boldsymbol{G}_{reset}^T$$
> 其中 $\boldsymbol{G}_{reset} = \frac{\partial \delta\boldsymbol{x}_{after}}{\partial \delta\boldsymbol{x}_{before}}$。
> 由于误差状态很小，$\boldsymbol{G}_{reset} \approx \boldsymbol{I}$，实践中通常省略。

---

## 七、完整 ESKF 流程

```
每帧 IMU+轮速数据到达（~200Hz）：
│
├── 1. predict(msg, dt)
│   ├── 补偿零偏：acc = a_raw - b_a_, w = w_raw - b_g_
│   ├── 名义状态积分：p_, v_, q_
│   ├── 计算状态转移矩阵 Fx = I + F·dt
│   └── 协方差预测：P = Fx·P·Fx^T + Q
│
├── 2. observeWheel(msg)
│   ├── 麦轮运动学计算观测量 y = [vx, vy, 0, wz]
│   ├── 计算观测预测 h(x) = [R^T·v, gyro_z - b_g_z]
│   ├── 计算雅可比矩阵 H (4×15)
│   ├── 卡尔曼增益 K = P·H^T·(H·P·H^T + R)^{-1}
│   ├── 更新误差状态 δx += K·(y - h)
│   └── 更新协方差 P（Joseph 形式）
│
├── 3. observeZeroTilt()
│   ├── 提取 pitch, roll
│   ├── 观测量 y = [0, 0]，预测 h = [pitch, roll]
│   ├── 雅可比 H：∂pitch/∂δθ_y = 1, ∂roll/∂δθ_x = 1
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

| 参数 | 代码值 | 含义 | 调参指导 |
|------|--------|------|---------|
| $\boldsymbol{P}_0$ | `Identity * 0.01` | 初始状态不确定性 | 初始位姿越不确定应越大 |
| $\boldsymbol{Q}$ | `Identity * 0.005` | 过程噪声 | IMU 精度越低应越大 |
| $\boldsymbol{R}$ | `4×4 Identity * 0.005` | 轮速观测噪声 | 轮速打滑越多应越大 |
| $\boldsymbol{R}_{tilt}$ | `2×2 Identity * 0.005` | 零倾斜观测噪声 | 值越小→约束越强→pitch/roll越稳定 |

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

IMU 输出已在车体系（电控已转换）：
  gyro_x: 绕车体 x 轴（前后方向）= 俯仰角速度
  gyro_y: 绕车体 y 轴（左右方向）= 横滚角速度
  gyro_z: 绕车体 z 轴（上下方向）= 偏航角速度
  acc_x: 沿车体前方加速度
  acc_y: 沿车体左方加速度
  acc_z: 沿车体上方加速度（静止时 ≈ +9.8 m/s²）

四元数 q_：车体系 → 世界系
  v_world = q_ * v_body
  v_body  = q_.inverse() * v_world = R^T * v_world
```

---

## 十、关键设计决策总结

| 决策 | 理由 |
|------|------|
| 使用 ESKF 而非 EKF | 误差状态始终很小，线性化精度高；姿态用 3 维旋转向量表示，无过约束 |
| $\delta\boldsymbol{\theta}$ 定义在车体系（右乘） | 与 IMU 在车体系测量一致，雅可比矩阵推导更自然 |
| 轮速观测在车体系 | 麦轮运动学直接输出车体系速度，无需旋转，避免坐标系混淆 |
| wz 纳入轮速观测 | 为 $\delta b_{g,z}$ 提供观测通路，帮助陀螺仪 z 轴零偏收敛 |
| 零倾斜观测 | 地面机器人 pitch/roll≈0 是强先验，直接约束角度抖动和 z 轴漂移 |
| Joseph 形式更新协方差 | 数值稳定，保证 $\boldsymbol{P}$ 对称正定 |
| 用 MCU 时间戳 `t_ms` 算 dt | 比 ROS `now()` 更精确（无 USB 传输延迟抖动） |
