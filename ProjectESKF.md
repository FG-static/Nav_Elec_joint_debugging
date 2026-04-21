# 项目 ESKF 技术文档

> 基于 `src/my_nav2_robot/src/data_handle.cpp` 的实现分析，含 IMU 外参扩展方案

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
| 运算空间 | 非线性流形 | 线性向量空间 $\mathbb{R}^{17}$ |
| 数值稳定性 | 差 | 好（误差小，协方差矩阵行为良好） |

**关键结论**：误差状态 $\delta\boldsymbol{x}$ 始终很小，在零点附近泰勒展开的精度远高于在大信号处展开。这使得 ESKF 的线性化模型比 EKF 更精确。

---

## 二、状态量定义与物理意义

### 2.1 问题描述：IMU 姿态安装误差

IMU 安装时未严格对齐车体坐标系，存在 Pitch 和 Roll 两个方向的安装角度误差 $\boldsymbol{\phi} = [\phi_x, \phi_y]^T$（定义在车体系下）。这导致 IMU 坐标系相对车体坐标系有一个小角度旋转：

$$\boldsymbol{R}_{body}^{imu} \approx \boldsymbol{I} + [\boldsymbol{\phi}_{3D}]_\times, \quad \boldsymbol{\phi}_{3D} = [\phi_x, \phi_y, 0]^T$$

影响如下：

1. **加速度投影偏移**：IMU 测到的加速度需旋转到车体系，安装角误差导致重力分量泄漏到水平轴：
   $$\boldsymbol{a}_{body} = (\boldsymbol{I} - [\boldsymbol{\phi}_{3D}]_\times) \boldsymbol{a}_{imu}$$
   静止时 $a_z^{imu} \approx 9.8$ m/s²，$\phi_y$ 使重力泄漏到 x 轴（前进方向），$\phi_x$ 使重力泄漏到 y 轴（横向），造成持续的虚假加速度。

2. **角速度投影偏移**：IMU 测到的角速度同样需旋转到车体系：
   $$\boldsymbol{\omega}_{body} = (\boldsymbol{I} - [\boldsymbol{\phi}_{3D}]_\times) \boldsymbol{\omega}_{imu}$$
   转弯时 $\omega_z$ 经安装角误差投影到 x/y 轴，造成 pitch/roll 漂移。

> **为什么只估计 Pitch/Roll 而不估计 Yaw？** Yaw 方向安装误差仅改变偏航参考方向，不影响里程计精度——加速度和重力投影与 yaw 安装角无关，且零倾斜观测也无法约束 yaw。因此 $\phi_z$ 不纳入状态向量。

### 2.2 状态向量

ESKF 将真实状态 $\boldsymbol{x}_t$ 分解为**名义状态** $\boldsymbol{x}$ 和**误差状态** $\delta\boldsymbol{x}$：

$$\boldsymbol{x}_t = \boldsymbol{x} \oplus \delta\boldsymbol{x}$$

其中 $\oplus$ 对位置/速度/零偏/外参是加法，对姿态是四元数乘法。

**名义状态**（6 组，共 19 参数，其中姿态 4 参数为四元数，外参 2 参数为欧拉角）：

| 符号 | 维度 | 物理意义 | 代码变量 |
|------|------|---------|---------|
| $\boldsymbol{p}$ | 3 | 世界系位置（车体中心） | `p_` |
| $\boldsymbol{v}$ | 3 | 世界系速度（车体中心） | `v_` |
| $\boldsymbol{q}$ | 4 | 姿态四元数（车体系→世界系） | `q_` |
| $\boldsymbol{b}_a$ | 3 | 加速度计零偏 | `b_a_` |
| $\boldsymbol{b}_g$ | 3 | 陀螺仪零偏 | `b_g_` |
| $\boldsymbol{\phi}$ | 2 | IMU 姿态安装误差（Pitch $\phi_y$，Roll $\phi_x$） | `phi_` |

**误差状态**（6 组，共 17 维——姿态用 3 维旋转向量，外参用 2 维角度误差）：

$$\delta\boldsymbol{x} = \begin{bmatrix} \delta\boldsymbol{p} \\ \delta\boldsymbol{v} \\ \delta\boldsymbol{\theta} \\ \delta\boldsymbol{b}_a \\ \delta\boldsymbol{b}_g \\ \delta\boldsymbol{\phi} \end{bmatrix} \in \mathbb{R}^{17}$$

| 分量 | 维度 | 索引 | 物理意义 |
|------|------|------|---------|
| $\delta\boldsymbol{p}$ | 3 | 0–2 | 位置误差（世界系，米） |
| $\delta\boldsymbol{v}$ | 3 | 3–5 | 速度误差（世界系，m/s） |
| $\delta\boldsymbol{\theta}$ | 3 | 6–8 | 姿态误差角（车体系，弧度） |
| $\delta\boldsymbol{b}_a$ | 3 | 9–11 | 加速度计零偏误差（m/s²） |
| $\delta\boldsymbol{b}_g$ | 3 | 12–14 | 陀螺仪零偏误差（rad/s） |
| $\delta\boldsymbol{\phi}$ | 2 | 15–16 | IMU 姿态安装误差（弧度） |

> **注意**：$\boldsymbol{\phi}$ 定义在车体系下。因为 IMU 固连在车体上，安装角度误差是车体系中的常量。仅估计 $\phi_x$（Roll）和 $\phi_y$（Pitch），不估计 $\phi_z$（Yaw）——如上节所述，Yaw 安装误差对里程计精度无影响。

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

### 2.4 IMU 测量模型（含姿态外参）

IMU 输出的角速度和加速度是在 IMU 自身坐标系下。由于安装角度误差 $\boldsymbol{\phi}$，IMU 坐标系相对车体坐标系有一个小角度旋转。需要将 IMU 读数旋转到车体系：

$$\boldsymbol{\omega}_{body} \approx (\boldsymbol{I} - [\boldsymbol{\phi}_{3D}]_\times) \boldsymbol{\omega}_{imu}$$

$$\boldsymbol{a}_{body} \approx (\boldsymbol{I} - [\boldsymbol{\phi}_{3D}]_\times) \boldsymbol{a}_{imu}$$

其中 $\boldsymbol{\phi}_{3D} = [\phi_x, \phi_y, 0]^T$，$[\boldsymbol{\phi}_{3D}]_\times$ 为反对称矩阵：

$$[\boldsymbol{\phi}_{3D}]_\times = \begin{bmatrix} 0 & 0 & \phi_y \\ 0 & 0 & -\phi_x \\ -\phi_y & \phi_x & 0 \end{bmatrix}$$

IMU 的实际测量值包含零偏和白噪声：

$$\boldsymbol{a}_m = \boldsymbol{a}_{imu} + \boldsymbol{b}_a + \boldsymbol{n}_a$$

$$\boldsymbol{\omega}_m = \boldsymbol{\omega}_{imu} + \boldsymbol{b}_g + \boldsymbol{n}_g$$

补偿零偏并转到车体系后：

$$\boldsymbol{a}_{clean}^{body} = (\boldsymbol{I} - [\boldsymbol{\phi}_{3D}]_\times)(\boldsymbol{a}_m - \boldsymbol{b}_a)$$

$$\boldsymbol{\omega}_{clean}^{body} = (\boldsymbol{I} - [\boldsymbol{\phi}_{3D}]_\times)(\boldsymbol{\omega}_m - \boldsymbol{b}_g)$$

> **IMU 特性**：静止时加速度计 z 轴读数 ≈ +9.8 m/s²（重力反作用力），并非 0。Pitch 安装误差 $\phi_y$ 使此重力泄漏到 x 轴，Roll 安装误差 $\phi_x$ 使重力泄漏到 y 轴——这是 IMU 姿态安装误差最显著的物理影响。

### 2.5 姿态关系（含外参）

车体姿态 $\boldsymbol{q}$ 描述的是车体系→世界系的旋转。IMU 安装角误差不影响这个定义——$\boldsymbol{q}$ 始终是车体的真实姿态。安装角误差仅影响如何将 IMU 读数正确解读为车体系下的物理量。

---

## 三、名义状态传播（predict）

### 3.1 连续时间运动方程

IMU 补偿零偏和安装角误差后输出车体系的"干净"加速度和角速度：

$$\boldsymbol{a}_{clean}^{body} = (\boldsymbol{I} - [\boldsymbol{\phi}_{3D}]_\times)(\boldsymbol{a}_m - \boldsymbol{b}_a)$$

$$\boldsymbol{\omega}_{clean}^{body} = (\boldsymbol{I} - [\boldsymbol{\phi}_{3D}]_\times)(\boldsymbol{\omega}_m - \boldsymbol{b}_g)$$

名义状态的连续时间微分方程：

$$\dot{\boldsymbol{p}} = \boldsymbol{v}$$

$$\dot{\boldsymbol{v}} = \boldsymbol{R}(\boldsymbol{q}) \cdot \boldsymbol{a}_{clean}^{body} + \boldsymbol{g}$$

$$\dot{\boldsymbol{q}} = \frac{1}{2}\boldsymbol{q} \otimes \boldsymbol{\omega}_{clean}^{body}$$

$$\dot{\boldsymbol{\phi}} = \boldsymbol{0} \quad (\text{安装角为常量})$$

$$\dot{\boldsymbol{b}}_a = \boldsymbol{0}, \quad \dot{\boldsymbol{b}}_g = \boldsymbol{0} \quad (\text{零偏为慢时变常量})$$

其中：
- $\boldsymbol{R}(\boldsymbol{q})$：四元数对应的旋转矩阵（车体系→世界系）
- $\boldsymbol{g} = [0, 0, -9.8]^T$：世界系重力加速度

### 3.2 离散时间积分

采用一阶欧拉法（IMU 频率 200Hz，dt≈5ms，精度足够）：

$$\boldsymbol{p}_{k+1} = \boldsymbol{p}_k + \boldsymbol{v}_k \Delta t + \frac{1}{2}(\boldsymbol{R}_k \boldsymbol{a}_{clean}^{body} + \boldsymbol{g})\Delta t^2$$

$$\boldsymbol{v}_{k+1} = \boldsymbol{v}_k + (\boldsymbol{R}_k \boldsymbol{a}_{clean}^{body} + \boldsymbol{g})\Delta t$$

$$\boldsymbol{q}_{k+1} = \boldsymbol{q}_k \otimes \Delta\boldsymbol{q}(\boldsymbol{\omega}_{clean}^{body} \Delta t)$$

$$\boldsymbol{\phi}_{k+1} = \boldsymbol{\phi}_k$$

其中角增量四元数：

$$\Delta\boldsymbol{q} = \begin{bmatrix} \sin(\frac{\|\Delta\boldsymbol{\theta}\|}{2}) \frac{\Delta\boldsymbol{\theta}}{\|\Delta\boldsymbol{\theta}\|} \\ \cos(\frac{\|\Delta\boldsymbol{\theta}\|}{2}) \end{bmatrix}, \quad \Delta\boldsymbol{\theta} = \boldsymbol{\omega}_{clean}^{body} \Delta t$$

> **当前代码实现**（`data_handle.cpp:210-232`，15维，φ=0 时 acc_body = acc_imu）：
> ```cpp
> Eigen::Vector3d acc = acc_filtered_ - b_a_;
> Eigen::Vector3d w   = gyro_filtered_ - b_g_;
> p_ = p_ + v_ * dt + 0.5 * (q_ * acc + G_VEC_) * dt * dt;
> v_ = v_ + (q_ * acc + G_VEC_) * dt;
> ```
>
> **扩展为 17 维后需修改**（新增姿态外参补偿）：
> ```cpp
> Eigen::Vector3d acc_imu = acc_filtered_ - b_a_;
> Eigen::Vector3d w_imu   = gyro_filtered_ - b_g_;
> Eigen::Matrix3d phi_cross = skew_symmetric(Eigen::Vector3d(phi_.x(), phi_.y(), 0.0));
> Eigen::Vector3d acc_body = (Eigen::Matrix3d::Identity() - phi_cross) * acc_imu;
> Eigen::Vector3d w_body   = (Eigen::Matrix3d::Identity() - phi_cross) * w_imu;
> p_ = p_ + v_ * dt + 0.5 * (q_ * acc_body + G_VEC_) * dt * dt;
> v_ = v_ + (q_ * acc_body + G_VEC_) * dt;
> ```
> 当 φ=0 时，`phi_cross = 0`，`acc_body = acc_imu`，与当前代码完全一致。

### 3.3 误差状态传播方程

对误差状态在名义状态附近做一阶泰勒展开：

$$\delta\dot{\boldsymbol{x}} = \boldsymbol{F} \delta\boldsymbol{x} + \boldsymbol{G}\boldsymbol{w}$$

其中系统矩阵 $\boldsymbol{F} \in \mathbb{R}^{17 \times 17}$：

$$\boldsymbol{F} = \begin{bmatrix} \boldsymbol{0} & \boldsymbol{I} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} \\ \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{F}_{\boldsymbol{v}\boldsymbol{\theta}} & -\boldsymbol{R} & \boldsymbol{0} & \boldsymbol{F}_{\boldsymbol{v}\boldsymbol{\phi}} \\ \boldsymbol{0} & \boldsymbol{0} & -[\boldsymbol{\omega}_{clean}^{body}]_\times & \boldsymbol{0} & -\boldsymbol{I} & \boldsymbol{F}_{\boldsymbol{\theta}\boldsymbol{\phi}} \\ \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} \\ \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} \\ \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} \end{bmatrix}$$

**各分块的物理意义推导**：

#### $\delta\dot{\boldsymbol{v}}$ 对 $\delta\boldsymbol{\theta}$ 的偏导：$\boldsymbol{F}_{\boldsymbol{v}\boldsymbol{\theta}}$

速度传播方程：$\dot{\boldsymbol{v}} = \boldsymbol{R}(\boldsymbol{q})\boldsymbol{a}_{clean}^{body} + \boldsymbol{g}$

当姿态有扰动 $\delta\boldsymbol{\theta}$（车体系，右乘）时：

$$\boldsymbol{R}_t = \boldsymbol{R}(\boldsymbol{I} - [\delta\boldsymbol{\theta}]_\times)$$

$$\delta\dot{\boldsymbol{v}} = -\boldsymbol{R}[\delta\boldsymbol{\theta}]_\times \boldsymbol{a}_{clean}^{body} = -\boldsymbol{R}[\boldsymbol{a}_{clean}^{body}]_\times \delta\boldsymbol{\theta}$$

$$\boxed{\boldsymbol{F}_{\boldsymbol{v}\boldsymbol{\theta}} = -\boldsymbol{R}[\boldsymbol{a}_{clean}^{body}]_\times}$$

> 与 15 维版本形式相同，只是 $\boldsymbol{a}_{clean}$ 替换为补偿安装角后的 $\boldsymbol{a}_{clean}^{body}$。

#### $\delta\dot{\boldsymbol{v}}$ 对 $\delta\boldsymbol{\phi}$ 的偏导：$\boldsymbol{F}_{\boldsymbol{v}\boldsymbol{\phi}}$

当安装角有误差 $\delta\boldsymbol{\phi}$ 时，加速度的旋转补偿不准确：

$$\boldsymbol{a}_{clean}^{body}(\boldsymbol{\phi}+\delta\boldsymbol{\phi}) \approx (\boldsymbol{I} - [\boldsymbol{\phi}_{3D}+\delta\boldsymbol{\phi}_{3D}]_\times)\boldsymbol{a}_{clean}^{imu} = \boldsymbol{a}_{clean}^{body} - [\delta\boldsymbol{\phi}_{3D}]_\times \boldsymbol{a}_{clean}^{imu}$$

其中 $\boldsymbol{a}_{clean}^{imu} = \boldsymbol{a}_m - \boldsymbol{b}_a$ 是 IMU 系下的干净加速度。

速度误差：

$$\delta\dot{\boldsymbol{v}} = \boldsymbol{R}(-[\delta\boldsymbol{\phi}_{3D}]_\times \boldsymbol{a}_{clean}^{imu}) = \boldsymbol{R}[\boldsymbol{a}_{clean}^{imu}]_\times \delta\boldsymbol{\phi}_{3D}$$

由于 $\delta\phi_z = 0$，只需取 $[\boldsymbol{a}_{clean}^{imu}]_\times$ 的前 2 列乘以 $\delta\boldsymbol{\phi} = [\delta\phi_x, \delta\phi_y]^T$：

$$[\boldsymbol{a}]_\times \begin{bmatrix} \delta\phi_x \\ \delta\phi_y \\ 0 \end{bmatrix} = \begin{bmatrix} 0 & -a_z & a_y \\ a_z & 0 & -a_x \\ -a_y & a_x & 0 \end{bmatrix} \begin{bmatrix} \delta\phi_x \\ \delta\phi_y \\ 0 \end{bmatrix} = \begin{bmatrix} -a_z\delta\phi_y \\ a_z\delta\phi_x \\ -a_y\delta\phi_x + a_x\delta\phi_y \end{bmatrix}$$

因此：

$$\boldsymbol{F}_{\boldsymbol{v}\boldsymbol{\phi}} = \boldsymbol{R} \begin{bmatrix} 0 & -a_z^{imu} \\ a_z^{imu} & 0 \\ -a_y^{imu} & a_x^{imu} \end{bmatrix}$$

其中 $\boldsymbol{a}^{imu} = \boldsymbol{a}_m - \boldsymbol{b}_a$。

$$\boxed{\boldsymbol{F}_{\boldsymbol{v}\boldsymbol{\phi}} = \boldsymbol{R} \begin{bmatrix} 0 & -a_z^{imu} \\ a_z^{imu} & 0 \\ -a_y^{imu} & a_x^{imu} \end{bmatrix}}$$

> **物理意义**：这是 IMU 姿态安装误差最核心的耦合项。静止时 $a_z^{imu} \approx 9.8$ m/s²，$a_x^{imu}, a_y^{imu} \approx 0$：
> $$\boldsymbol{F}_{\boldsymbol{v}\boldsymbol{\phi}} \approx \boldsymbol{R} \begin{bmatrix} 0 & -9.8 \\ 9.8 & 0 \\ 0 & 0 \end{bmatrix}$$
> $\phi_y$（Pitch 安装误差）使重力泄漏到前进方向，$\phi_x$（Roll 安装误差）使重力泄漏到横向。即使静止，这个耦合也始终存在——这是姿态安装误差与位置偏移的本质区别。

#### $\delta\dot{\boldsymbol{\theta}}$ 对 $\delta\boldsymbol{\phi}$ 的偏导：$\boldsymbol{F}_{\boldsymbol{\theta}\boldsymbol{\phi}}$

类似地，角速度的旋转补偿也有误差：

$$\boldsymbol{\omega}_{clean}^{body}(\boldsymbol{\phi}+\delta\boldsymbol{\phi}) \approx \boldsymbol{\omega}_{clean}^{body} - [\delta\boldsymbol{\phi}_{3D}]_\times \boldsymbol{\omega}_{clean}^{imu}$$

角速度误差影响姿态误差率：

$$\delta\dot{\boldsymbol{\theta}} = -[\boldsymbol{\omega}_{clean}^{body}]_\times \delta\boldsymbol{\theta} - \delta\boldsymbol{b}_g + [\boldsymbol{\omega}_{clean}^{imu}]_\times \delta\boldsymbol{\phi}_{3D}$$

取前 2 列：

$$\boldsymbol{F}_{\boldsymbol{\theta}\boldsymbol{\phi}} = [\boldsymbol{\omega}_{clean}^{imu}]_\times \begin{bmatrix} 1 & 0 \\ 0 & 1 \\ 0 & 0 \end{bmatrix} = \begin{bmatrix} 0 & -\omega_z^{imu} \\ \omega_z^{imu} & 0 \\ -\omega_y^{imu} & \omega_x^{imu} \end{bmatrix}$$

$$\boxed{\boldsymbol{F}_{\boldsymbol{\theta}\boldsymbol{\phi}} = \begin{bmatrix} 0 & -\omega_z^{imu} \\ \omega_z^{imu} & 0 \\ -\omega_y^{imu} & \omega_x^{imu} \end{bmatrix}}$$

> **物理意义**：对于地面机器人，$\omega_x^{imu}, \omega_y^{imu} \approx 0$，$\omega_z^{imu}$ 占主导：
> $$\boldsymbol{F}_{\boldsymbol{\theta}\boldsymbol{\phi}} \approx \begin{bmatrix} 0 & -\omega_z \\ \omega_z & 0 \\ 0 & 0 \end{bmatrix}$$
> 转弯时（$\omega_z \neq 0$），Pitch 安装误差 $\phi_y$ 导致 Roll 角漂移，Roll 安装误差 $\phi_x$ 导致 Pitch 角漂移。

#### $\delta\dot{\boldsymbol{v}}$ 对 $\delta\boldsymbol{b}_a$ 的偏导：$-\boldsymbol{R}$

与 15 维版本相同：

$$\boldsymbol{a}_{clean}^{*} = (\boldsymbol{I} - [\boldsymbol{\phi}]_\times)(\boldsymbol{a}_m - \boldsymbol{b}_a - \delta\boldsymbol{b}_a) = \boldsymbol{a}_{clean}^{body} - (\boldsymbol{I} - [\boldsymbol{\phi}]_\times)\delta\boldsymbol{b}_a$$

$$\delta\dot{\boldsymbol{v}} = -\boldsymbol{R}(\boldsymbol{I} - [\boldsymbol{\phi}]_\times)\delta\boldsymbol{b}_a \approx -\boldsymbol{R}\delta\boldsymbol{b}_a$$

> 小角度近似下 $[\boldsymbol{\phi}]_\times \delta\boldsymbol{b}_a$ 为二阶小量，可忽略。

#### $\delta\dot{\boldsymbol{\theta}}$ 对 $\delta\boldsymbol{\theta}$ 和 $\delta\boldsymbol{b}_g$ 的偏导

与 15 维版本相同：

$$\frac{\partial \delta\dot{\boldsymbol{\theta}}}{\partial \delta\boldsymbol{\theta}} = -[\boldsymbol{\omega}_{clean}^{body}]_\times, \quad \frac{\partial \delta\dot{\boldsymbol{\theta}}}{\partial \delta\boldsymbol{b}_g} = -\boldsymbol{I}$$

#### $\delta\dot{\boldsymbol{\phi}}$ 对各误差的偏导

$\boldsymbol{\phi}$ 为常量，$\delta\dot{\boldsymbol{\phi}} = \boldsymbol{0}$，所有偏导为零。

### 3.4 离散化状态转移矩阵

一阶近似：$\boldsymbol{\Phi} \approx \boldsymbol{I} + \boldsymbol{F}\Delta t$

$$\boldsymbol{\Phi} = \begin{bmatrix} \boldsymbol{I} & \boldsymbol{I}\Delta t & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} \\ \boldsymbol{0} & \boldsymbol{I} & -\boldsymbol{R}[\boldsymbol{a}_c^{body}]_\times\Delta t & -\boldsymbol{R}\Delta t & \boldsymbol{0} & \boldsymbol{F}_{\boldsymbol{v}\boldsymbol{\phi}}\Delta t \\ \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{I} - [\boldsymbol{\omega}_c^{body}]_\times\Delta t & \boldsymbol{0} & -\boldsymbol{I}\Delta t & \boldsymbol{F}_{\boldsymbol{\theta}\boldsymbol{\phi}}\Delta t \\ \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{I} & \boldsymbol{0} & \boldsymbol{0} \\ \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{I} & \boldsymbol{0} \\ \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{I} \end{bmatrix}$$

> **当前代码实现**（`data_handle.cpp:234-244`，15维，φ=0 时 acc_body = acc）：
> ```cpp
> Eigen::Matrix<double, 15, 15> Fx = Eigen::Matrix<double, 15, 15>::Identity();
> Eigen::Matrix3d R = q_.toRotationMatrix();
> Fx.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
> Fx.block<3, 3>(3, 6) = -R * skew_symmetric(acc) * dt;
> Fx.block<3, 3>(3, 9) = -R * dt;
> Fx.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() - skew_symmetric(w * dt);
> Fx.block<3, 3>(6, 12) = -Eigen::Matrix3d::Identity() * dt;
> ```
>
> **扩展为 17 维后需新增**（在上述 15 维代码基础上追加）：
> ```cpp
> // 维度扩展为 17×17，前 15×15 分块不变
> Eigen::Matrix<double, 3, 2> F_v_phi, F_theta_phi;
> F_v_phi << 0, -acc_imu.z(),
>            acc_imu.z(), 0,
>           -acc_imu.y(), acc_imu.x();
> F_v_phi = R * F_v_phi;
> F_theta_phi << 0,       -w_imu.z(),
>                 w_imu.z(), 0,
>                -w_imu.y(), w_imu.x();
> Fx.block<3, 2>(3, 15) = F_v_phi * dt;        // ∂δv/∂δφ ★新增
> Fx.block<3, 2>(6, 15) = F_theta_phi * dt;     // ∂δθ/∂δφ ★新增
> ```
> 当 φ=0 时，`acc_imu = acc`，`w_imu = w`，前 15×15 分块与当前代码完全一致。

### 3.5 协方差预测

$$\boldsymbol{P}_{k|k-1} = \boldsymbol{\Phi}\boldsymbol{P}_{k-1|k-1}\boldsymbol{\Phi}^T + \boldsymbol{Q}_d$$

其中 $\boldsymbol{Q}_d \in \mathbb{R}^{17 \times 17}$ 是离散化过程噪声矩阵：

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

> **注意**：$\boldsymbol{v}$ 已经是车体中心速度（在 predict 中已通过安装角补偿将 IMU 加速度转换为车体系加速度）。

**角速度部分**（第 4 维）：

轮速反算的 $\omega_z$ 与车体系下已补偿零偏和安装角的陀螺仪 z 轴输出应该一致：

$$h_w(\boldsymbol{x}) = \omega_z^{body,clean} = [0, 0, 1] \cdot (\boldsymbol{I} - [\boldsymbol{\phi}_{3D}]_\times)(\boldsymbol{\omega}_m - \boldsymbol{b}_g)$$

展开得：

$$h_w = (\omega_{z}^{imu} - b_{g,z}) + \phi_y(\omega_x^{imu} - b_{g,x}) - \phi_x(\omega_y^{imu} - b_{g,y})$$

对于地面机器人 $\omega_x^{imu}, \omega_y^{imu} \approx 0$，简化为：

$$h_w \approx \omega_{z}^{imu} - b_{g,z}$$

综合观测预测值：

$$\boldsymbol{h}(\boldsymbol{x}) = \begin{bmatrix} \boldsymbol{R}^T\boldsymbol{v} \\ \omega_{z}^{imu} - b_{g,z} \end{bmatrix}$$

#### 4.1.4 雅可比矩阵 $\boldsymbol{H}$ 的推导

$\boldsymbol{H} = \frac{\partial \boldsymbol{h}}{\partial \delta\boldsymbol{x}} \in \mathbb{R}^{4 \times 17}$

**速度部分** $h_v = \boldsymbol{R}^T\boldsymbol{v}$ 对各误差状态的偏导：

**(a) 对 $\delta\boldsymbol{v}$ 的偏导**：$\boldsymbol{R}^T$

**(b) 对 $\delta\boldsymbol{\theta}$ 的偏导**：$-[\boldsymbol{v}_{body}]_\times$

**(c) 对 $\delta\boldsymbol{\phi}$ 的偏导**：

$h_v$ 不显式依赖 $\boldsymbol{\phi}$（速度状态 $\boldsymbol{v}$ 已经是车体系积分结果），因此：

$$\frac{\partial h_v}{\partial \delta\boldsymbol{\phi}} = \boldsymbol{0}_{3 \times 2}$$

> $\delta\boldsymbol{\phi}$ 对 $h_v$ 的影响已在 predict 步骤的 $\boldsymbol{F}_{\boldsymbol{v}\boldsymbol{\phi}}$ 中体现。

**角速度部分** $h_w$ 对各误差状态的偏导：

**(d) 对 $\delta\boldsymbol{b}_g$ 的偏导**：$[0, 0, -1]$

**(e) 对 $\delta\boldsymbol{\phi}$ 的偏导**：

严格展开 $h_w$ 中的安装角项：

$$\frac{\partial h_w}{\partial \delta\phi_x} = -(\omega_y^{imu} - b_{g,y}) \approx 0$$

$$\frac{\partial h_w}{\partial \delta\phi_y} = (\omega_x^{imu} - b_{g,x}) \approx 0$$

对于地面机器人，此项可忽略，设为零。

**综合雅可比矩阵**：

$$\boldsymbol{H} = \begin{bmatrix} \boldsymbol{0}_{3\times3} & \boldsymbol{R}^T & -[\boldsymbol{v}_{body}]_\times & \boldsymbol{0}_{3\times3} & \boldsymbol{0}_{3\times3} & \boldsymbol{0}_{3\times2} \\ \boldsymbol{0}_{1\times3} & \boldsymbol{0}_{1\times3} & \boldsymbol{0}_{1\times3} & \boldsymbol{0}_{1\times3} & [0\;0\;{-1}] & \boldsymbol{0}_{1\times2} \end{bmatrix}$$

> 与 15 维版本相比，$\boldsymbol{H}$ 仅在右侧增加了 2 列零（对应 $\delta\boldsymbol{\phi}$）。$\delta\boldsymbol{\phi}$ 的可观测量完全通过 predict 步骤的 $\boldsymbol{F}_{\boldsymbol{v}\boldsymbol{\phi}}$ 和 $\boldsymbol{F}_{\boldsymbol{\theta}\boldsymbol{\phi}}$ 耦合到 $\delta\boldsymbol{v}$ 和 $\delta\boldsymbol{\theta}$，再由观测间接约束。

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

与 15 维版本推导完全相同（pitch/roll 只与 $\delta\boldsymbol{\theta}$ 有关，与 $\delta\boldsymbol{\phi}$ 无直接关系）：

$$\boldsymbol{H}_{tilt} = \begin{bmatrix} \boldsymbol{0}_{1\times6} & 0 & 1 & 0 & \boldsymbol{0}_{1\times8} \\ \boldsymbol{0}_{1\times6} & 1 & 0 & 0 & \boldsymbol{0}_{1\times8} \end{bmatrix} \in \mathbb{R}^{2 \times 17}$$

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

$$\boldsymbol{\phi} \leftarrow \boldsymbol{\phi} + \delta\boldsymbol{\phi}$$

其中角度注入使用轴角→四元数：

$$\Delta\boldsymbol{q}(\delta\boldsymbol{\theta}) = \begin{bmatrix} \sin(\frac{\|\delta\boldsymbol{\theta}\|}{2})\frac{\delta\boldsymbol{\theta}}{\|\delta\boldsymbol{\theta}\|} \\ \cos(\frac{\|\delta\boldsymbol{\theta}\|}{2}) \end{bmatrix}$$

> **当前代码实现**（`data_handle.cpp:386-402`，15维）：
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
>
> **扩展为 17 维后需新增**（在上述代码之后追加）：
> ```cpp
> phi_ += delta_x_.segment<2>(15);  // ★新增
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
│   ├── 补偿零偏：acc_imu = a_raw - b_a_, w_imu = w_raw - b_g_
│   ├── 补偿安装角：acc_body = (I - [φ]×) * acc_imu, w_body = (I - [φ]×) * w_imu  ★修改
│   ├── 名义状态积分：p_, v_, q_, φ_ 不变
│   ├── 计算状态转移矩阵 Fx (17×17)：
│   │   ├── Fx(3,6)  = -R * [acc_body]× * dt
│   │   ├── Fx(3,15) = R * F_v_phi * dt    ★新增（重力泄漏耦合）
│   │   └── Fx(6,15) = F_theta_phi * dt     ★新增（角速度耦合）
│   └── 协方差预测：P = Fx·P·Fx^T + Q (17×17)
│
├── 2. observeWheel(msg)
│   ├── 麦轮运动学计算观测量 y = [vx, vy, 0, wz]
│   ├── 计算观测预测 h(x) = [R^T·v, gyro_z - b_g_z]
│   ├── 计算雅可比矩阵 H (4×17)，δφ 列为零
│   ├── 卡尔曼增益 K = P·H^T·(H·P·H^T + R)^{-1}
│   ├── 更新误差状态 δx += K·(y - h)
│   └── 更新协方差 P（Joseph 形式）
│
├── 3. observeZeroTilt()
│   ├── 提取 pitch, roll
│   ├── 观测量 y = [0, 0]，预测 h = [pitch, roll]
│   ├── 雅可比 H (2×17)：∂pitch/∂δθ_y = 1, ∂roll/∂δθ_x = 1，δφ 列为零
│   ├── K = P·H^T·(H·P·H^T + R_tilt)^{-1}
│   ├── δx += K·(y - h)
│   └── 更新协方差 P（Joseph 形式）
│
├── 4. injectAndReset()
│   ├── 将 δx 注入名义状态：p_, v_, q_, b_a_, b_g_, φ_  ★新增 φ_
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
| $\boldsymbol{P}_0$ | `Identity * 0.01` | 17×17 | 初始状态不确定性 | $\delta\boldsymbol{\phi}$ 分量取决于 IMU 安装角度精度 |
| $\boldsymbol{Q}$ | `Identity * 0.005` | 17×17 | 过程噪声 | IMU 精度越低应越大；$\delta\boldsymbol{\phi}$ 对应行应设小值 |
| $\boldsymbol{R}$ | `4×4 Identity * 0.005` | 4×4 | 轮速观测噪声 | 轮速打滑越多应越大 |
| $\boldsymbol{R}_{tilt}$ | `2×2 Identity * 0.005` | 2×2 | 零倾斜观测噪声 | 值越小→约束越强→pitch/roll越稳定 |

### 外参相关参数调参建议

| 参数 | 建议值 | 说明 |
|------|--------|------|
| $P_0$ 的 $\delta\boldsymbol{\phi}$ 分量 | `0.001` ~ `0.01` | 初始安装角不确定性（rad²）。1°≈0.0175 rad，若安装精度约 3°，则 $P_{0,\phi} \approx 0.001$ |
| $Q$ 的 $\delta\boldsymbol{\phi}$ 分量 | `1e-8` ~ `1e-6` | 安装角为常量，过程噪声应极小。过大导致 $\boldsymbol{\phi}$ 震荡 |

**调参原则**：

- $\boldsymbol{Q}$ ↑ → 更信任观测（轮速），IMU 漂移被更快修正，但里程计更抖
- $\boldsymbol{R}$ ↑ → 更信任 IMU，轮速修正弱，直线性好但可能漂移
- $\boldsymbol{R}_{tilt}$ ↓ → 零倾斜约束强，pitch/roll 被锁死，但上坡时不灵活
- $Q_{\delta\boldsymbol{\phi}}$ ↑ → 安装角估计更灵活，但可能震荡；↓ → 安装角更稳定，但收敛慢

---

## 九、坐标系约定

```
世界系 (odom)：          车体系 (base_footprint)：
  ↑ z                      ↑ z (上)
  |                        |
  +---→ y                  +---→ y (左)
 /                        /
↙ x                      ↙ x (前)

IMU 姿态安装误差 φ = [φx, φy]^T（车体系下）：
  φx (Roll 安装误差)：IMU 绕车体 x 轴的旋转偏差
  φy (Pitch 安装误差)：IMU 绕车体 y 轴的旋转偏差
  （不估计 φz/Yaw 安装误差，因为不影响里程计精度）

IMU 坐标系 → 车体系 转换：
  v_body = (I - [φ3D]×) * v_imu
  其中 φ3D = [φx, φy, 0]^T

IMU 输出在 IMU 坐标系（需旋转到车体系）：
  gyro_x: 绕 IMU x 轴角速度 → 转到车体系后 ≈ 俯仰角速度
  gyro_y: 绕 IMU y 轴角速度 → 转到车体系后 ≈ 横滚角速度
  gyro_z: 绕 IMU z 轴角速度 → 转到车体系后 ≈ 偏航角速度
  acc_x: 沿 IMU x 轴加速度（静止时 ≈ -9.8·sin(φy) ≈ 微小值）
  acc_y: 沿 IMU y 轴加速度（静止时 ≈ 9.8·sin(φx) ≈ 微小值）
  acc_z: 沿 IMU z 轴加速度（静止时 ≈ +9.8 m/s²）

四元数 q_：车体系 → 世界系
  v_world = q_ * v_body
  v_body  = q_.inverse() * v_world = R^T * v_world
```

---

## 十、IMU 外参可观测性分析

### 10.1 可观测性链路

$\delta\boldsymbol{\phi}$ 能否被 ESKF 正确估计，取决于它是否可观测。有两条传播链路：

**链路 1：重力泄漏（始终活跃）**

$$\delta\boldsymbol{\phi} \xrightarrow{\boldsymbol{F}_{\boldsymbol{v}\boldsymbol{\phi}} \approx \boldsymbol{R}\begin{bmatrix}0&-9.8\\9.8&0\\0&0\end{bmatrix}} \delta\boldsymbol{v} \xrightarrow{\boldsymbol{R}^T} \text{轮速观测}$$

静止时即活跃：安装角误差使重力泄漏到水平方向，产生虚假加速度→速度误差→被轮速零速观测检测。

**链路 2：角速度耦合（转弯时活跃）**

$$\delta\boldsymbol{\phi} \xrightarrow{\boldsymbol{F}_{\boldsymbol{\theta}\boldsymbol{\phi}} \approx \begin{bmatrix}0&-\omega_z\\\omega_z&0\\0&0\end{bmatrix}} \delta\boldsymbol{\theta} \xrightarrow{\boldsymbol{H}_{tilt}} \text{零倾斜观测}$$

转弯时活跃：安装角误差使偏航角速度投影到 pitch/roll 轴→姿态漂移→被零倾斜观测检测。

### 10.2 实际可观测性判断

| 外参分量 | 可观测条件 | 麦轮底盘实际 | 结论 |
|----------|-----------|-------------|------|
| $\phi_x$ (Roll) | 静止时重力泄漏到 y 轴 + 轮速观测 | 始终可观测 | ✅ 强可观测 |
| $\phi_y$ (Pitch) | 静止时重力泄漏到 x 轴 + 轮速观测 | 始终可观测 | ✅ 强可观测 |

> **与位置偏移的关键区别**：位置偏移 $\boldsymbol{r}$ 需要角速度（转弯）才可观测，而姿态安装角 $\boldsymbol{\phi}$ 仅需重力（静止即可）就可观测。这是因为重力始终存在，安装角误差使重力泄漏到水平轴，即使零速也能通过轮速观测检测到。这使得姿态外参比位置外参**更容易在线标定**。

---

## 十一、关键设计决策总结

| 决策 | 理由 |
|------|------|
| 使用 ESKF 而非 EKF | 误差状态始终很小，线性化精度高；姿态用 3 维旋转向量表示，无过约束 |
| $\delta\boldsymbol{\theta}$ 定义在车体系（右乘） | 与 IMU 在车体系测量一致，雅可比矩阵推导更自然 |
| $\boldsymbol{\phi}$ 定义在车体系，仅含 Pitch/Roll | IMU 固连于车体，安装角为常量；Yaw 不影响里程计精度，无需估计 |
| 轮速观测在车体系 | 麦轮运动学直接输出车体系速度，无需旋转，避免坐标系混淆 |
| wz 纳入轮速观测 | 为 $\delta b_{g,z}$ 提供观测通路，帮助陀螺仪 z 轴零偏收敛 |
| 零倾斜观测 | 地面机器人 pitch/roll≈0 是强先验，直接约束角度抖动和 z 轴漂移 |
| Joseph 形式更新协方差 | 数值稳定，保证 $\boldsymbol{P}$ 对称正定 |
| 用 MCU 时间戳 `t_ms` 算 dt | 比 ROS `now()` 更精确（无 USB 传输延迟抖动） |
| 17 维状态向量含 $\delta\boldsymbol{\phi}$ | IMU 姿态安装误差使重力泄漏到水平轴，必须在线估计才能消除 |
| 不估计 $\phi_z$ | Yaw 安装误差不影响加速度/重力的投影关系，对里程计精度无影响 |

---

## 附录：6-DOF 外参扩展说明

当 IMU 同时存在位置偏移 $\boldsymbol{r} = [r_x, r_y, r_z]^T$ 和姿态安装误差 $\boldsymbol{\phi} = [\phi_x, \phi_y, \phi_z]^T$ 时，误差状态向量扩展为 21 维（15 + 3 + 3），$\boldsymbol{F}$ 矩阵需同时包含两类外参的耦合项：

| 耦合项 | 表达式 | 物理来源 |
|--------|--------|---------|
| $\boldsymbol{F}_{\boldsymbol{v}\boldsymbol{r}}$ | $-\boldsymbol{R}[\boldsymbol{\omega}]_\times^2$ | 位置偏移→向心加速度补偿残差 |
| $\boldsymbol{F}_{\boldsymbol{v}\boldsymbol{\phi}}$ | $\boldsymbol{R}[\boldsymbol{a}^{imu}]_\times \boldsymbol{J}_{2\to3}$ | 姿态误差→加速度投影偏移（重力泄漏） |
| $\boldsymbol{F}_{\boldsymbol{\theta}\boldsymbol{\phi}}$ | $[\boldsymbol{\omega}^{imu}]_\times \boldsymbol{J}_{2\to3}$ | 姿态误差→角速度投影偏移 |

其中 $\boldsymbol{J}_{2\to3} = \begin{bmatrix}1&0\\0&1\\0&0\end{bmatrix}$（若估计全部 3 维姿态外参则此项退化为 $\boldsymbol{I}_{3\times3}$）。

此外，6-DOF 外参中位置和姿态存在交叉耦合：IMU 姿态误差会改变位置偏移在世界系下的表达（$\boldsymbol{R}\boldsymbol{r}$ 变为 $\boldsymbol{R}(\boldsymbol{I}-[\delta\boldsymbol{\phi}]_\times)\boldsymbol{r}$），导致 $\boldsymbol{F}_{\boldsymbol{v}\boldsymbol{\phi}}$ 增加一项 $\boldsymbol{R}[\boldsymbol{r}]_\times[\boldsymbol{\omega}_{clean}]_\times^2$；以及速度转换中 $\delta\boldsymbol{\phi}$ 和 $\delta\boldsymbol{r}$ 的联合效应。这些交叉项通常为二阶小量，实践中可根据精度需求决定是否保留。

对于地面麦轮底盘，可观测性总结：

| 外参分量 | 可观测性 | 条件 |
|----------|---------|------|
| $\phi_x, \phi_y$ | ✅ 强 | 静止即可（重力泄漏） |
| $r_x, r_y$ | ✅ 中 | 需要转弯（向心加速度） |
| $r_z$ | ❌ 弱 | 需要 pitch/roll 角速度 |
| $\phi_z$ | ❌ 弱 | 需要多方向激励 |
