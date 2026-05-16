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

其中 $\boldsymbol{F} = \frac{\partial \delta\dot{\boldsymbol{x}}}{\partial \delta\boldsymbol{x}}$，$\boldsymbol{G} = \frac{\partial \delta\dot{\boldsymbol{x}}}{\partial \boldsymbol{w}}$。下面逐行推导各分块。

---

#### 第 1 行：位置误差 $\delta\dot{\boldsymbol{p}}$

**名义方程**：$\dot{\boldsymbol{p}} = \boldsymbol{v}$

**真实方程**：$\dot{\boldsymbol{p}}_t = \boldsymbol{v}_t = (\boldsymbol{v} + \delta\boldsymbol{v})$

**误差方程**：

$$\delta\dot{\boldsymbol{p}} = \dot{\boldsymbol{p}}_t - \dot{\boldsymbol{p}} = \delta\boldsymbol{v}$$

**对各误差状态偏导**：

| 偏导 | 结果 | 物理含义 |
|------|------|---------|
| $\frac{\partial \delta\dot{\boldsymbol{p}}}{\partial \delta\boldsymbol{p}}$ | $\boldsymbol{0}$ | 位置误差不自我增长 |
| $\frac{\partial \delta\dot{\boldsymbol{p}}}{\partial \delta\boldsymbol{v}}$ | $\boldsymbol{I}$ | 速度误差直接积分进位置误差 |
| $\frac{\partial \delta\dot{\boldsymbol{p}}}{\partial \delta\boldsymbol{\theta}}$ | $\boldsymbol{0}$ | 位置变化率不依赖姿态误差 |
| $\frac{\partial \delta\dot{\boldsymbol{p}}}{\partial \delta\boldsymbol{b}_a}$ | $\boldsymbol{0}$ | 位置变化率不依赖加计零偏 |
| $\frac{\partial \delta\dot{\boldsymbol{p}}}{\partial \delta\boldsymbol{b}_g}$ | $\boldsymbol{0}$ | 位置变化率不依赖陀螺零偏 |

$$\boxed{\delta\dot{\boldsymbol{p}} = \delta\boldsymbol{v} \implies \boldsymbol{F}_{\boldsymbol{pp}} = \boldsymbol{0},\; \boldsymbol{F}_{\boldsymbol{pv}} = \boldsymbol{I},\; \boldsymbol{F}_{\boldsymbol{p\theta}} = \boldsymbol{0},\; \boldsymbol{F}_{\boldsymbol{pb}_a} = \boldsymbol{0},\; \boldsymbol{F}_{\boldsymbol{pb}_g} = \boldsymbol{0}}$$

---

#### 第 2 行：速度误差 $\delta\dot{\boldsymbol{v}}$

**名义方程**：$\dot{\boldsymbol{v}} = \boldsymbol{R}\boldsymbol{a}_c + \boldsymbol{g}$

其中 $\boldsymbol{a}_c = \boldsymbol{a}_{clean}^{body}$ 是补偿零偏后的车体系加速度。

**真实方程**：真实加速度计测量包含零偏误差 $\delta\boldsymbol{b}_a$ 和白噪声 $\boldsymbol{n}_a$：

$$\dot{\boldsymbol{v}}_t = \boldsymbol{R}_t(\boldsymbol{a}_c - \delta\boldsymbol{b}_a - \boldsymbol{n}_a) + \boldsymbol{g}$$

**误差推导——姿态扰动的影响**：

真实旋转矩阵受姿态误差 $\delta\boldsymbol{\theta}$（右乘，车体系定义）扰动：

$$\boldsymbol{R}_t = \boldsymbol{R}(\boldsymbol{I} - [\delta\boldsymbol{\theta}]_\times) + O(\|\delta\boldsymbol{\theta}\|^2)$$

代入速度方程，保留一阶项：

$$\dot{\boldsymbol{v}}_t = \boldsymbol{R}(\boldsymbol{I} - [\delta\boldsymbol{\theta}]_\times)(\boldsymbol{a}_c - \delta\boldsymbol{b}_a - \boldsymbol{n}_a) + \boldsymbol{g}$$

展开，忽略二阶小量 $\delta\boldsymbol{\theta} \cdot \delta\boldsymbol{b}_a$ 等：

$$\dot{\boldsymbol{v}}_t = \boldsymbol{R}\boldsymbol{a}_c + \boldsymbol{g} - \boldsymbol{R}[\delta\boldsymbol{\theta}]_\times\boldsymbol{a}_c - \boldsymbol{R}\delta\boldsymbol{b}_a - \boldsymbol{R}\boldsymbol{n}_a$$

因此误差方程为：

$$\delta\dot{\boldsymbol{v}} = \dot{\boldsymbol{v}}_t - \dot{\boldsymbol{v}} = -\boldsymbol{R}[\delta\boldsymbol{\theta}]_\times\boldsymbol{a}_c - \boldsymbol{R}\delta\boldsymbol{b}_a - \boldsymbol{R}\boldsymbol{n}_a$$

**利用反对称矩阵性质化简**：$[\delta\boldsymbol{\theta}]_\times\boldsymbol{a}_c = -[\boldsymbol{a}_c]_\times\delta\boldsymbol{\theta}$

$$\delta\dot{\boldsymbol{v}} = -\boldsymbol{R}[\boldsymbol{a}_c]_\times\delta\boldsymbol{\theta} - \boldsymbol{R}\delta\boldsymbol{b}_a - \boldsymbol{R}\boldsymbol{n}_a$$

**对各误差状态偏导**：

| 偏导 | 结果 | 物理含义 |
|------|------|---------|
| $\frac{\partial \delta\dot{\boldsymbol{v}}}{\partial \delta\boldsymbol{p}}$ | $\boldsymbol{0}$ | 速度变化率不依赖位置误差 |
| $\frac{\partial \delta\dot{\boldsymbol{v}}}{\partial \delta\boldsymbol{v}}$ | $\boldsymbol{0}$ | 速度误差不自我增长（无阻尼） |
| $\frac{\partial \delta\dot{\boldsymbol{v}}}{\partial \delta\boldsymbol{\theta}}$ | $-\boldsymbol{R}[\boldsymbol{a}_c]_\times$ | 姿态误差导致加速度投影方向偏移，$\|\boldsymbol{a}_c\|$ 越大耦合越强 |
| $\frac{\partial \delta\dot{\boldsymbol{v}}}{\partial \delta\boldsymbol{b}_a}$ | $-\boldsymbol{R}$ | 加计零偏误差经旋转矩阵投影到世界系 |
| $\frac{\partial \delta\dot{\boldsymbol{v}}}{\partial \delta\boldsymbol{b}_g}$ | $\boldsymbol{0}$ | 陀螺零偏不直接影响速度（仅通过姿态间接影响） |

**噪声驱动**：$\boldsymbol{G}_{\boldsymbol{v}\boldsymbol{n}_a} = -\boldsymbol{R}$，即加速度计白噪声经旋转投影到世界系驱动速度误差。

$$\boxed{\delta\dot{\boldsymbol{v}} = -\boldsymbol{R}[\boldsymbol{a}_c]_\times\delta\boldsymbol{\theta} - \boldsymbol{R}\delta\boldsymbol{b}_a - \boldsymbol{R}\boldsymbol{n}_a}$$

---

#### 第 3 行：姿态误差 $\delta\dot{\boldsymbol{\theta}}$

**名义方程**：$\dot{\boldsymbol{q}} = \frac{1}{2}\boldsymbol{q} \otimes \boldsymbol{\omega}_c$

其中 $\boldsymbol{\omega}_c = \boldsymbol{\omega}_{clean}^{body}$ 是补偿零偏后的车体系角速度。

**真实方程**：真实陀螺仪测量包含零偏误差 $\delta\boldsymbol{b}_g$ 和白噪声 $\boldsymbol{n}_g$：

$$\dot{\boldsymbol{q}}_t = \frac{1}{2}\boldsymbol{q}_t \otimes (\boldsymbol{\omega}_c - \delta\boldsymbol{b}_g - \boldsymbol{n}_g)$$

**误差推导**：

姿态误差定义为右乘扰动：$\boldsymbol{q}_t = \boldsymbol{q} \otimes \delta\boldsymbol{q}$，其中 $\delta\boldsymbol{q} \approx [\frac{1}{2}\delta\boldsymbol{\theta}, 1]^T$。

对时间求导：

$$\dot{\boldsymbol{q}}_t = \dot{\boldsymbol{q}} \otimes \delta\boldsymbol{q} + \boldsymbol{q} \otimes \delta\dot{\boldsymbol{q}}$$

将 $\dot{\boldsymbol{q}}_t$ 和 $\dot{\boldsymbol{q}}$ 的表达式代入，整理可得：

$$\delta\dot{\boldsymbol{\theta}} = -[\boldsymbol{\omega}_c]_\times\delta\boldsymbol{\theta} - \delta\boldsymbol{b}_g - \boldsymbol{n}_g$$

**关键步骤**：从四元数导数到旋转向量导数的推导利用了小角度近似和李括号性质：

$$[\delta\boldsymbol{\theta}]_\times\boldsymbol{\omega}_c - [\boldsymbol{\omega}_c]_\times\delta\boldsymbol{\theta} = [\delta\boldsymbol{\theta} \times \boldsymbol{\omega}_c]_\times = -[\boldsymbol{\omega}_c]_\times\delta\boldsymbol{\theta} + [\delta\boldsymbol{\theta}]_\times\boldsymbol{\omega}_c$$

在右乘扰动定义下，合并后得到 $-[\boldsymbol{\omega}_c]_\times\delta\boldsymbol{\theta}$ 项。

**对各误差状态偏导**：

| 偏导 | 结果 | 物理含义 |
|------|------|---------|
| $\frac{\partial \delta\dot{\boldsymbol{\theta}}}{\partial \delta\boldsymbol{p}}$ | $\boldsymbol{0}$ | 姿态变化率不依赖位置误差 |
| $\frac{\partial \delta\dot{\boldsymbol{\theta}}}{\partial \delta\boldsymbol{v}}$ | $\boldsymbol{0}$ | 姿态变化率不依赖速度误差 |
| $\frac{\partial \delta\dot{\boldsymbol{\theta}}}{\partial \delta\boldsymbol{\theta}}$ | $-[\boldsymbol{\omega}_c]_\times$ | 姿态误差受角速度叉乘旋转，反映 SO3 流形曲率 |
| $\frac{\partial \delta\dot{\boldsymbol{\theta}}}{\partial \delta\boldsymbol{b}_a}$ | $\boldsymbol{0}$ | 加计零偏不影响角速度 |
| $\frac{\partial \delta\dot{\boldsymbol{\theta}}}{\partial \delta\boldsymbol{b}_g}$ | $-\boldsymbol{I}$ | 陀螺零偏误差 1:1 注入姿态误差 |

**噪声驱动**：$\boldsymbol{G}_{\boldsymbol{\theta}\boldsymbol{n}_g} = -\boldsymbol{I}$，即陀螺仪白噪声直接驱动姿态误差。

$$\boxed{\delta\dot{\boldsymbol{\theta}} = -[\boldsymbol{\omega}_c]_\times\delta\boldsymbol{\theta} - \delta\boldsymbol{b}_g - \boldsymbol{n}_g}$$

---

#### 第 4、5 行：零偏误差 $\delta\dot{\boldsymbol{b}}_a$, $\delta\dot{\boldsymbol{b}}_g$

**建模假设**：零偏为慢时变常量，其变化用随机游走描述：

$$\dot{\boldsymbol{b}}_a = \boldsymbol{n}_{ba}, \quad \dot{\boldsymbol{b}}_g = \boldsymbol{n}_{bg}$$

**误差方程**：

$$\delta\dot{\boldsymbol{b}}_a = \boldsymbol{n}_{ba}, \quad \delta\dot{\boldsymbol{b}}_g = \boldsymbol{n}_{bg}$$

所有偏导为零，噪声直接驱动。物理含义：零偏误差没有自愈机制，只能靠观测更新修正。

$$\boxed{\boldsymbol{F}_{\boldsymbol{b}_a\boldsymbol{*}} = \boldsymbol{0},\; \boldsymbol{G}_{\boldsymbol{b}_a\boldsymbol{n}_{ba}} = \boldsymbol{I},\; \boldsymbol{F}_{\boldsymbol{b}_g\boldsymbol{*}} = \boldsymbol{0},\; \boldsymbol{G}_{\boldsymbol{b}_g\boldsymbol{n}_{bg}} = \boldsymbol{I}}$$

---

#### 汇总：F 矩阵与 G 矩阵

将以上各行偏导组合，得到：

$$\boldsymbol{F} = \frac{\partial \delta\dot{\boldsymbol{x}}}{\partial \delta\boldsymbol{x}} = \begin{bmatrix} \boldsymbol{0} & \boldsymbol{I} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} \\ \boldsymbol{0} & \boldsymbol{0} & -\boldsymbol{R}[\boldsymbol{a}_c]_\times & -\boldsymbol{R} & \boldsymbol{0} \\ \boldsymbol{0} & \boldsymbol{0} & -[\boldsymbol{\omega}_c]_\times & \boldsymbol{0} & -\boldsymbol{I} \\ \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} \\ \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} \end{bmatrix}$$

$$\boldsymbol{G} = \frac{\partial \delta\dot{\boldsymbol{x}}}{\partial \boldsymbol{w}} = \begin{bmatrix} \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} \\ \boldsymbol{0} & -\boldsymbol{R} & \boldsymbol{0} & \boldsymbol{0} \\ -\boldsymbol{I} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} \\ \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{I} & \boldsymbol{0} \\ \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{I} \end{bmatrix}$$

噪声向量 $\boldsymbol{w} = [\boldsymbol{n}_g;\, \boldsymbol{n}_a;\, \boldsymbol{n}_{ba};\, \boldsymbol{n}_{bg}] \in \mathbb{R}^{12}$。

| 噪声分量 | 维度 | 物理含义 | 驱动路径 |
|----------|------|---------|---------|
| $\boldsymbol{n}_g$ | 3 | 陀螺仪白噪声 | $\delta\dot{\boldsymbol{\theta}} \leftarrow -\boldsymbol{n}_g$ |
| $\boldsymbol{n}_a$ | 3 | 加速度计白噪声 | $\delta\dot{\boldsymbol{v}} \leftarrow -\boldsymbol{R}\boldsymbol{n}_a$ |
| $\boldsymbol{n}_{ba}$ | 3 | 加计零偏随机游走 | $\delta\dot{\boldsymbol{b}}_a \leftarrow \boldsymbol{n}_{ba}$ |
| $\boldsymbol{n}_{bg}$ | 3 | 陀螺零偏随机游走 | $\delta\dot{\boldsymbol{b}}_g \leftarrow \boldsymbol{n}_{bg}$ |

连续时间噪声谱密度矩阵 $\boldsymbol{Q}_c \in \mathbb{R}^{12 \times 12}$：

$$\boldsymbol{Q}_c = \begin{bmatrix} \sigma_g^2\boldsymbol{I} & & & \\ & \sigma_a^2\boldsymbol{I} & & \\ & & \sigma_{ba}^2\boldsymbol{I} & \\ & & & \sigma_{bg}^2\boldsymbol{I} \end{bmatrix}$$

离散化过程噪声矩阵（一阶近似）：

$$\boldsymbol{Q}_d \approx \boldsymbol{G}\boldsymbol{Q}_c\boldsymbol{G}^T\Delta t = \begin{bmatrix} \boldsymbol{0} & & & & \\ & \boldsymbol{R}\sigma_a^2\boldsymbol{I}\boldsymbol{R}^T\Delta t & & & \\ & & \sigma_g^2\boldsymbol{I}\Delta t & & \\ & & & \sigma_{ba}^2\boldsymbol{I}\Delta t & \\ & & & & \sigma_{bg}^2\boldsymbol{I}\Delta t \end{bmatrix}$$

> **注意**：当前代码实现中 `Q_` 为 15×15 对角阵（`Identity * 0.005`），是 $\boldsymbol{Q}_d$ 的简化近似，未严格按 $\boldsymbol{G}\boldsymbol{Q}_c\boldsymbol{G}^T\Delta t$ 构建。若需精确建模，应按上式分别设置各分块对角元素。

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

---

## 十二、IESKF 改进方案

### 12.1 为什么从 ESKF 升级到 IESKF

当前 ESKF 存在两个已验证的精度瓶颈：

**问题一：v_anti 注入虚假 heading 修正**

实测数据（nav_debug1）：
- wheel 积分 = 352°，imu 积分 = 352°，但 odom = 428°（多 76°）
- /odom_raw（无观测修正）= 345°（接近真值 360°）
- δθ_z 来源分析：99.9% 来自 v_anti 的 yaw 列，每秒注入 ±1°~15°

根因：当前 predict 的 Fx 对 δθ 使用一阶近似 $\boldsymbol{I} - [\boldsymbol{\omega}\Delta t]_\times$，在大角度时线性化误差大。v_anti 的 Jacobian 在错误的工作点上做线性化，导致 $\delta\boldsymbol{\theta}_z$ 的修正方向和大小都不准。

**问题二：Q 矩阵统一导致 P(δθ) 过大**

$\boldsymbol{Q}$ 对所有 15 个状态使用同一个 $Q_{init}=0.005$，但 $\delta\boldsymbol{\theta}$ 的真实过程噪声（IMU 积分精度）远小于此。结果 $P(\delta\boldsymbol{\theta})$ 保持高位 → v_anti 增益过大 → heading 被噪声随机游走。

**IESKF 如何解决这两个问题**：

1. **A_matrix 修正 Fx**：用 SO3 左雅可比精确替代一阶近似，线性化精度提升 → v_anti Jacobian 更准 → heading 修正更合理
2. **迭代更新**：多次线性化收敛到最优解，消除单次线性化误差
3. **流形上的协方差传播**：P 在 SO3 切空间中传播，物理意义更正确

### 12.2 核心概念：SO3 流形与误差状态

当前 ESKF 的姿态误差定义：

$$\boldsymbol{q}_t = \boldsymbol{q} \otimes \delta\boldsymbol{q}, \quad \delta\boldsymbol{q} \approx \begin{bmatrix} \frac{1}{2}\delta\boldsymbol{\theta} \\ 1 \end{bmatrix}$$

这是**欧氏空间的加法近似**——把 $\delta\boldsymbol{\theta}$ 当作 $\mathbb{R}^3$ 中的向量来加减。在小角度时足够精确，但角度增大后线性化误差显著。

IESKF 使用**流形上的 boxplus/boxminus 运算**：

**boxplus**（$\oplus$）：将误差向量注入流形上的点

$$\boldsymbol{q} \oplus \delta\boldsymbol{\theta} = \boldsymbol{q} \otimes \exp(\delta\boldsymbol{\theta})$$

其中 $\exp: \mathfrak{so}(3) \rightarrow SO(3)$ 是指数映射（Rodrigues 公式）：

$$\exp(\boldsymbol{v}) = \cos\|\boldsymbol{v}\| \cdot \boldsymbol{I} + \frac{\sin\|\boldsymbol{v}\|}{\|\boldsymbol{v}\|} [\boldsymbol{v}]_\times + \frac{1 - \cos\|\boldsymbol{v}\|}{\|\boldsymbol{v}\|^2} \boldsymbol{v}\boldsymbol{v}^T$$

**boxminus**（$\ominus$）：计算流形上两点之间的"距离"（误差向量）

$$\boldsymbol{q}_a \ominus \boldsymbol{q}_b = \log(\boldsymbol{q}_b^{-1} \otimes \boldsymbol{q}_a)$$

其中 $\log: SO(3) \rightarrow \mathfrak{so}(3)$ 是对数映射：

$$\log(\boldsymbol{R}) = \frac{\theta}{2\sin\theta}(\boldsymbol{R} - \boldsymbol{R}^T), \quad \theta = \arccos\frac{\text{tr}(\boldsymbol{R}) - 1}{2}$$

**含义**：boxplus/boxminus 确保误差状态始终在 SO3 的切空间（李代数 $\mathfrak{so}(3)$）中操作，而非在欧氏空间中近似。这是 IESKF 的数学基础。

### 12.3 A_matrix：SO3 左雅可比

**这是 IESKF 与标准 ESKF 的最关键区别。**

标准 ESKF 在 predict 中计算 Fx 的姿态块：

$$\boldsymbol{F}_{6:9, 6:9} = \boldsymbol{I} - [\boldsymbol{\omega}\Delta t]_\times$$

这是 $\exp(\boldsymbol{\omega}\Delta t)$ 的一阶泰勒展开。当 $\|\boldsymbol{\omega}\Delta t\| = 0.1$ rad 时误差约 0.17%，但 $\|\boldsymbol{\omega}\Delta t\| = 0.5$ rad 时误差约 4.2%，$\|\boldsymbol{\omega}\Delta t\| = 1.0$ rad 时误差约 16%。

IESKF 使用 **A_matrix**（SO3 的左雅可比）精确修正：

$$\boldsymbol{A}(\boldsymbol{v}) = \boldsymbol{I} + \frac{1 - \cos\|\boldsymbol{v}\|}{\|\boldsymbol{v}\|^2} [\boldsymbol{v}]_\times + \frac{\|\boldsymbol{v}\| - \sin\|\boldsymbol{v}\|}{\|\boldsymbol{v}\|^3} [\boldsymbol{v}]_\times^2$$

**物理含义**：A_matrix 是从李代数到李群的"转移算子"。当误差状态 $\delta\boldsymbol{\theta}$ 从当前切空间传递到下一个切空间时，A_matrix 补偿了流形曲率造成的非线性效应。

**小角度近似**：

$$\boldsymbol{A}(\boldsymbol{v}) \approx \boldsymbol{I} - \frac{1}{2}[\boldsymbol{v}]_\times + \frac{1}{6}[\boldsymbol{v}]_\times^2 + \cdots$$

一阶近似 $\boldsymbol{I} - \frac{1}{2}[\boldsymbol{v}]_\times$ 就是当前 ESKF 用的（但系数是 $\frac{1}{2}$ 而非 1，这是关键差异）。

**为什么当前 ESKF 用 $\boldsymbol{I} - [\boldsymbol{\omega}\Delta t]_\times$ 而 IESKF 用 $\boldsymbol{A}(\boldsymbol{\omega}\Delta t)$**：

当前 ESKF 的 Fx 是从连续时间误差状态方程 $\delta\dot{\boldsymbol{\theta}} = -[\boldsymbol{\omega}_c]_\times \delta\boldsymbol{\theta}$ 离散化得到的，本质是 $\boldsymbol{I} + \boldsymbol{F}_{\theta\theta}\Delta t = \boldsymbol{I} - [\boldsymbol{\omega}_c\Delta t]_\times$。

IESKF 认为这种离散化忽略了 SO3 的曲率。正确的离散化应该是：先计算 $\exp(\boldsymbol{\omega}_c\Delta t)$，再用 A_matrix 将切空间中的 Jacobian 映射回正确的切空间。

### 12.4 predict 改造

**当前 predict 代码**（`data_handle.cpp`）：

```cpp
Fx.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
Fx.block<3, 3>(3, 6) = -R * skew_symmetric(acc) * dt;
Fx.block<3, 3>(3, 9) = -R * dt;
Fx.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() - skew_symmetric(w * dt);  // ← 一阶近似
Fx.block<3, 3>(6, 12) = -Eigen::Matrix3d::Identity() * dt;
```

**IESKF predict 改造**：

```cpp
Eigen::Vector3d dtheta = w * dt;  // 角增量向量
Eigen::Matrix3d A = A_matrix(dtheta);  // SO3 左雅可比（精确）

Fx.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
Fx.block<3, 3>(3, 6) = -R * skew_symmetric(acc) * A * dt;  // 速度对姿态误差
Fx.block<3, 3>(3, 9) = -R * A * dt;                          // 速度对 b_a 误差
Fx.block<3, 3>(6, 6) = A;                                     // ← A_matrix 替代一阶近似
Fx.block<3, 3>(6, 12) = -A * dt;                              // 姿态对 b_g 误差
```

**各改动的物理意义**：

| 行 | 当前 | IESKF | 原因 |
|----|------|-------|------|
| $\boldsymbol{F}_{\theta\theta}$ | $\boldsymbol{I} - [\boldsymbol{\omega}\Delta t]_\times$ | $\boldsymbol{A}(\boldsymbol{\omega}\Delta t)$ | 精确的 SO3 切空间传递，消除一阶截断误差 |
| $\boldsymbol{F}_{v\theta}$ | $-\boldsymbol{R}[\boldsymbol{a}_c]_\times \Delta t$ | $-\boldsymbol{R}[\boldsymbol{a}_c]_\times \boldsymbol{A} \Delta t$ | A_matrix 修正速度对姿态误差的耦合 |
| $\boldsymbol{F}_{vb_a}$ | $-\boldsymbol{R}\Delta t$ | $-\boldsymbol{R}\boldsymbol{A}\Delta t$ | A_matrix 修正速度对零偏的耦合 |
| $\boldsymbol{F}_{\theta b_g}$ | $-\boldsymbol{I}\Delta t$ | $-\boldsymbol{A}\Delta t$ | A_matrix 修正姿态对零偏的耦合 |

### 12.5 A_matrix 实现

从 FAST-LIO 的 `mtkmath.hpp` 搬来，纯头文件无依赖：

```cpp
// SO3 左雅可比（A_matrix）
// 输入：v ∈ R^3，旋转向量（弧度）
// 输出：3×3 矩阵 A(v)
// 公式：A(v) = I + (1-cos||v||)/||v||^2 * [v]x + (||v||-sin||v||)/||v||^3 * [v]x^2
// 小角度近似：A(v) ≈ I - 0.5*[v]x
static Eigen::Matrix3d A_matrix(const Eigen::Vector3d &v) {
    double nv = v.norm();
    if (nv < 1e-10) {
        // 小角度：用泰勒展开避免除零
        // A(v) ≈ I - 0.5*[v]x + (1/6)*[v]x^2
        Eigen::Matrix3d vx = skew_symmetric(v);
        return Eigen::Matrix3d::Identity() - 0.5 * vx + (1.0/6.0) * vx * vx;
    }
    Eigen::Matrix3d vx = skew_symmetric(v);
    double nv2 = nv * nv;
    return Eigen::Matrix3d::Identity()
           + (1.0 - std::cos(nv)) / nv2 * vx
           + (nv - std::sin(nv)) / (nv2 * nv) * vx * vx;
}
```

### 12.6 迭代观测更新

**当前 ESKF 的更新流程**：

```
observeWheel()    → δx += K·(y - h), P 更新
observeZeroTilt() → δx += K·(y - h), P 更新
injectAndReset()  → x = x ⊕ δx, δx = 0
```

问题：两个观测是**串行单次**执行的。每次观测更新后，名义状态未变（要等 inject 才变），导致第二次观测的 H 矩阵在过时的工作点上计算。

**IESKF 迭代更新**：

```
循环直到收敛（||dx_|| < ε）：
  1. 计算误差：δθ = log(R_prop^{-1} · R_current)
  2. 旋转到切空间：P ← A(δθ)^T · P · A(δθ)
  3. 合并所有观测：H = [H_wheel; H_tilt], y = [y_wheel; y_tilt]
  4. 标准 KF 更新：K, innovation, dx_
  5. 注入：x ← x ⊕ dx_
  6. 检查收敛
```

**为什么迭代能提升精度**：

每次注入后名义状态更新，H 矩阵在新的工作点重新计算。对于轮速观测中的 v_anti（$[\boldsymbol{v}_{body}]_\times$），$\boldsymbol{v}_{body} = \boldsymbol{R}^T\boldsymbol{v}$ 依赖当前姿态 $\boldsymbol{R}$。迭代保证 H 在正确的 $\boldsymbol{R}$ 上计算，消除了单次更新的线性化误差。

**迭代更新的关键公式**（来自 FAST-LIO `esekfom.hpp`）：

$$\delta\boldsymbol{x}_{new} = \boldsymbol{K}\boldsymbol{h} + (\boldsymbol{K}\boldsymbol{H} - \boldsymbol{I})\delta\boldsymbol{x}_{current}$$

其中 $\delta\boldsymbol{x}_{current}$ 是当前迭代的误差状态估计，$\boldsymbol{K}\boldsymbol{h}$ 是标准卡尔曼修正项，$(\boldsymbol{K}\boldsymbol{H} - \boldsymbol{I})\delta\boldsymbol{x}_{current}$ 是对上一次迭代估计的修正。这两项叠加后注入名义状态，实现"逐步逼近最优解"。

### 12.7 协方差在切空间中的传播

当前 ESKF 的 P 矩阵在 inject 后保持不变（$\delta\boldsymbol{x}$ 清零但 P 不变）。这在数学上不严格——P 描述的是 $\delta\boldsymbol{\theta}$ 的不确定性，但 $\delta\boldsymbol{\theta}$ 的"零点"在注入后变了。

IESKF 在注入后用 A_matrix 旋转 P：

$$\boldsymbol{P}_{new} = \boldsymbol{A}(\delta\boldsymbol{\theta}_{inject})^T \cdot \boldsymbol{P}_{old} \cdot \boldsymbol{A}(\delta\boldsymbol{\theta}_{inject})$$

**物理含义**：注入改变了名义姿态，切空间也随之旋转。A_matrix 将 P 从旧切空间旋转到新切空间，确保协方差的几何意义正确。

这对 $\delta\boldsymbol{\theta}_z$ 的抑制效果：由于 A_matrix 是精确的旋转，P(δθ) 不会因注入操作而人为膨胀，从根本上压制了 v_anti 的随机游走。

### 12.8 可选改进：S2 流形重力估计

当前 ESKF 用 observeZeroTilt 硬约束 pitch=0, roll=0。IESKF 可将重力方向作为 S2（单位球面）状态估计：

$$\boldsymbol{g} = \|\boldsymbol{g}\| \cdot \hat{\boldsymbol{g}}, \quad \hat{\boldsymbol{g}} \in S^2$$

S2 流形的误差状态是 2 维（球面切空间），用 Bx 基矩阵参数化：

$$\hat{\boldsymbol{g}} \oplus \delta\boldsymbol{\xi} = \exp(\boldsymbol{B}_x \delta\boldsymbol{\xi}) \cdot \hat{\boldsymbol{g}}, \quad \delta\boldsymbol{\xi} \in \mathbb{R}^2$$

**对本项目的适用性**：地面机器人在平地上运动，重力方向近似恒定 $[0, 0, -9.8]^T$。硬约束 pitch=0, roll=0 更直接有效，**建议保留当前的 observeZeroTilt 方案，不改 S2 流形**。

### 12.9 改造总结

| 改动项 | 当前代码 | IESKF 版本 | 收益 | 必要性 |
|--------|---------|-----------|------|--------|
| Fx 姿态块 | $\boldsymbol{I} - [\boldsymbol{\omega}\Delta t]_\times$ | $\boldsymbol{A}(\boldsymbol{\omega}\Delta t)$ | 消除一阶截断误差，大角度精度提升 | **必做** |
| Fx 速度-姿态耦合 | $-\boldsymbol{R}[\boldsymbol{a}_c]_\times\Delta t$ | $-\boldsymbol{R}[\boldsymbol{a}_c]_\times\boldsymbol{A}\Delta t$ | Jacobian 更精确 | **必做** |
| Fx 速度-零偏耦合 | $-\boldsymbol{R}\Delta t$ | $-\boldsymbol{R}\boldsymbol{A}\Delta t$ | Jacobian 更精确 | **必做** |
| Fx 姿态-零偏耦合 | $-\boldsymbol{I}\Delta t$ | $-\boldsymbol{A}\Delta t$ | Jacobian 更精确 | **必做** |
| 观测更新 | 单次串行 | 迭代合并 | 消除 H 矩阵工作点误差 | 推荐 |
| inject 后 P 旋转 | 无 | $\boldsymbol{A}(\delta\boldsymbol{\theta})^T \boldsymbol{P} \boldsymbol{A}(\delta\boldsymbol{\theta})$ | P 几何意义正确，抑制随机游走 | 推荐 |
| boxplus/boxminus | 手动四元数乘法 | 流形运算 | 代码清晰，大角度安全 | 推荐 |
| S2 重力估计 | observeZeroTilt 硬约束 | S2 流形估计 | 不依赖地面水平假设 | 可选（本项目不需要） |

### 12.10 预期效果

以实测数据量化：

| 指标 | 当前 ESKF | 预期 IESKF | 理论依据 |
|------|-----------|-----------|---------|
| nav_debug1 odom 误差 | 76° / 40s | <15° / 40s | A_matrix 消除 Fx 线性化误差 |
| nav_debug2 odom 误差 | 227° / 42s | <25° / 42s | 迭代更新 + P 切空间旋转 |
| 转弯过头 | +16°~22° | <5° | v_anti Jacobian 精度提升 |
| 直走漂移率 | 1.6°/s | <0.5°/s | P(δθ) 不再人为膨胀 |

---

## 十三、IESKF 调试笔记：转弯过程中的"虚拟后退"问题

### 13.1 现象描述 (The Symptom)

**环境**：在从 ESKF 升级为 IESKF（迭代误差状态卡尔曼滤波）的过程中。

**问题**：小车在原地转弯或大角度转向时，位姿估计出现明显的"后退"现象。

**关键特征**：
- 该问题在 ESKF 模式下不明显，在 IESKF 下极其突出。
- 负向位移量随迭代次数增加而增大（迭代次数越多，后退越严重）。

### 13.2 诊断逻辑 (Diagnostic Logic)

采取了严密的"排除法"定位问题：

1. **数据源校验**：检查底层 $v_x$ 原始数据。发现转弯时 $v_x$ 波动极小，排除传感器原始数据计算错误。
2. **作用域定位**：名义状态 $p$（位置）在迭代循环外，排除。误差状态更新步骤 $\delta x = K(z - h(x))$ 在循环内，锁定问题源于此步骤。
3. **矩阵解析 (Matrix Anatomy)**：
   - **$H$ 矩阵**：轮速计观测函数 $h(x)$ 对位置误差 $\delta p$ 的偏导为 $0$。理论上轮速计不直接贡献位置修正。
   - **$P$ 矩阵**：由预测步产生，迭代中保持静态，排除。
   - **$R$ 矩阵 (观测噪声)**：锁定为核心变量。

### 13.3 根因分析 (Root Cause)

**物理层面**：$R_{w_z}$ 设置过大，代表滤波器"极度不信任"陀螺仪。

**数学连锁反应**：
1. 转弯时，加速度计感受到向心力。
2. 由于不信任旋转（$R_{w_z}$ 大），滤波器无法通过姿态旋转完全解释该加速度。
3. 为了减小残差，滤波器通过协方差交叉项将该"无法解释的力"错误地补偿到了速度 $\delta v$ 和位置 $\delta p$ 上。

**IESKF 的放大效应**：每一轮迭代都在前一轮错误的姿态基础上重新计算雅可比，导致这种错误的补偿在循环中被不断放大，最终表现为明显的后退。

### 13.4 解决方案 (The Fix)

**操作**：将观测噪声矩阵 $R$ 中 $\omega_z$ 维度的参数降低一个数量级（增加对陀螺仪的信任）。

**结果**：转弯后退现象立刻消失，位姿估计趋于平稳且精确。

### 13.5 沉淀的调试经验 (Key Takeaways)

#### A. IESKF 的"双刃剑"效应

- **优势**：迭代能更好地处理 $h(x)$ 的非线性，使收敛更接近全局最优。
- **风险**：迭代会放大**模型不匹配 (Model Mismatch)**。如果 $R$ 阵比例失调，迭代会将误差强制分配到错误的维度。

#### B. 观测矩阵 $H$ 的"间接修正"原理

即便 $H$ 中某维度的偏导为 $0$（如 $\frac{\partial h}{\partial \delta p} = 0$），该维度依然会通过协方差矩阵 $P$ 的耦合项受到修正。

**结论**：观测噪声 $R$ 的调参不仅影响该维度本身，还会通过耦合关系影响到那些"不可直接观测"的状态量。

#### C. 面对算法 Bug 的分析路径

1. **观察现象随迭代次数的变化**：这是区分 EKF 和 IESKF 特有问题的分水岭。
2. **公式维度拆解**：像本次通过锁定 $\delta p$ 维度，在 $K$ 阵中逆推 $H$、$P$、$R$ 的关系。
3. **信任权重分配**：在多传感器融合中，如果出现物理上不合理的补偿（如转弯导致位移），优先检查 $R$ 矩阵中各传感器信任比例的合理性。

### 13.6 下一步计划提示

当接下来引入 GICP（雷达位姿观测）时，由于此时 $H_p \neq 0$：

- **信任分配**：需要平衡"雷达位姿"与"轮速计速度"的信任权重。
- **动态调整**：考虑根据 GICP 的 fitness_score 动态调整 $R_{lidar}$，防止雷达在退化环境（如窄长廊）中带偏系统。

---

## 十四、近期工程化修复记录

本节记录近期围绕 `src/my_nav2_robot/src/data_handle.cpp` 和 `src/my_nav2_robot/src/imu_adapter.cpp` 的工程化修复。目标不是修改 ESKF 基本数学模型，而是修正**时间基准、点云刚体假设、回调调度方式、yaw 观测时间一致性**四类实现偏差。

### 14.1 IMU 时间戳与 dt 计算修复

#### 14.1.1 问题背景

`data_handle` 的 predict 依赖 IMU 帧间隔 $\Delta t$。若 $\Delta t$ 使用 ROS 当前时间 `this->now()` 计算，在 rosbag 回放、CPU 卡顿、暂停恢复时会引入调度层抖动，导致：

1. IMU 积分步长被回放节奏污染
2. `dt` 偶发突增，predict 误差放大
3. ESKF 将回放器抖动误认为真实传感器采样间隔变化

#### 14.1.2 修复方案

在 `imu_adapter_node` 中，不再使用节点当前时间生成 `Gimbal.t_ms`，而是改用 `/livox/imu.header.stamp` 的相对时间：

$$t_{ms}(k) = \frac{stamp_{imu}(k) - stamp_{imu}(0)}{10^6}$$

其中：
- `stamp_imu(0)`：首帧有效 IMU header 时间
- `stamp_imu(k)`：第 $k$ 帧 IMU 的 ROS 时间戳

同时加入两条保护：

1. 若 `header.stamp` 无效，则退回 `this->now()` 并节流告警
2. 若时间戳回退，则保持 `t_ms` 单调，避免 `data_handle` 看到负 `dt`

#### 14.1.3 设计理由

`header.stamp` 代表**传感器采样时间**，而 `this->now()` 代表**节点处理时间**。ESKF 的物理积分必须依赖采样时间，而不是处理时间。

因此，`data_handle` 中的

$$\Delta t = \frac{t_{ms}(k) - t_{ms}(k-1)}{1000}$$

应严格对应 IMU 真实采样间隔，而不应受到 ROS 调度和回放器行为影响。

### 14.2 点云运动畸变修复

#### 14.2.1 问题背景

MID360 的 `/livox/lidar/pointcloud` 一帧扫描持续约 `100ms`。若车辆在这段时间内发生边走边转，则一帧点云不再对应同一个刚体位姿，而是由多个采样时刻的点共同组成：

$$\mathcal{P} = \{ \mathbf{p}(t_i) \mid t_i \in [t_{start}, t_{end}] \}$$

若直接把整帧点云当成同一时刻的刚体观测送入 GICP，相当于假设：

$$\mathbf{T}(t_i) = \mathbf{T}(t_{ref}), \quad \forall i$$

该假设在静止时近似成立，但在转弯或轻微 yaw 抖动时会失效，表现为：

1. 转弯时点云出现拖影或滞后
2. GICP 的 yaw residual 在边走边转时变大
3. scan-to-scan 或 scan-to-submap 的刚体配准前提被破坏
4. 打开后验平移 deskew 后，若轨迹来源不可靠，可能进一步放大误差

因此，本轮修复方向从原来的“ESKF 历史姿态插值做旋转 deskew，GICP 通过后再补平移”调整为更接近 DLIO 的思路：

$$\text{IMU scan 内连续时间轨迹} \rightarrow \text{逐点 6DoF deskew} \rightarrow \text{GICP}$$

当前已经完成的是连续时间轨迹相关基础设施，尚未把 `parseLidarFrame()` 的实际 deskew 主路径切换到该轨迹。

#### 14.2.2 每点时间解析

`parseLidarFrame()` 手动解析 `PointCloud2` 字段，而不直接使用 `pcl::fromROSMsg`。原因是原始点云中包含每点 `timestamp/time/offset_time` 字段，直接转成 `PointXYZ` 会丢失每点采样时间。

对第 $i$ 个点，读取：

$$\mathbf{p}_i = [x_i, y_i, z_i]^T, \quad t_i = timestamp_i$$

并统计整帧时间范围：

$$t_{min} = \min_i t_i, \quad t_{max} = \max_i t_i$$

本工程中选取参考时刻：

$$t_{ref} = t_{max}$$

即将整帧点云 deskew 到扫描末端时刻。

#### 14.2.3 状态历史扩展

原先状态历史只保存：

$$\mathcal{H}_{old} = \{ (t_k, \mathbf{p}_k, \mathbf{q}_k) \}$$

这只能支持按时间插值位置和姿态，不足以构建 scan 内连续运动轨迹。本轮已将 `StateSnapshot` 扩展为：

$$\mathcal{H}_{state} = \{ (t_k, \mathbf{p}_k, \mathbf{v}_k, \mathbf{q}_k, \mathbf{b}_{a,k}, \mathbf{b}_{g,k}) \}$$

对应代码字段：

```cpp
struct StateSnapshot {
    int64_t stamp_ns;
    Eigen::Vector3d p;
    Eigen::Quaterniond q;
    Eigen::Vector3d v;
    Eigen::Vector3d b_a;
    Eigen::Vector3d b_g;
};
```

`pushStateHistory()` 在 IMU 回调完成 predict 和观测更新后写入当前名义状态，为后续 scan trajectory 找 anchor 状态提供依据。

#### 14.2.4 IMU 历史缓存

为避免仅依赖稀疏状态插值，本轮新增 `ImuSample` 历史：

```cpp
struct ImuSample {
    int64_t stamp_ns;
    Eigen::Vector3d acc_body;
    Eigen::Vector3d gyro_body;
};
```

IMU 回调中，在低通滤波和零偏补偿后，将 IMU 测量旋转到车体系并写入 `imu_history_`：

$$\mathbf{a}_{body} = \mathbf{R}_{imu}^{body}(\mathbf{a}_{filtered} - \mathbf{b}_a)$$

$$\boldsymbol{\omega}_{body} = \mathbf{R}_{imu}^{body}(\boldsymbol{\omega}_{filtered} - \mathbf{b}_g)$$

该缓存使用独立的 `imu_history_mtx_` 保护，并按 `state_history_duration_` 清理旧数据。后续 `buildImuTrajectory()` 会从该缓存中取出覆盖当前 scan 时间段的 IMU 样本。

#### 14.2.5 Anchor 状态选择

连续时间轨迹不能直接从 `t_start` 假设已有状态。如果 anchor 状态时间戳不等于 scan 起点，需要先从 anchor 传播到 `t_start`。

因此新增 `findAnchorState()`，从 `state_history_` 中寻找：

$$t_{anchor} \le t_{start}$$

且尽量靠近 `t_start` 的状态：

$$\mathbf{x}_{anchor} = (\mathbf{p}_{anchor}, \mathbf{v}_{anchor}, \mathbf{q}_{anchor})$$

这样 `buildImuTrajectory()` 的传播时间范围实际是：

$$[t_{anchor}, t_{end}]$$

而不是只取：

$$[t_{start}, t_{end}]$$

否则当 `t_anchor != t_start` 时，轨迹的时间原点会错位。

#### 14.2.6 Scan 内 IMU 轨迹构建

本轮新增 `ScanPoseSample`：

```cpp
struct ScanPoseSample {
    int64_t stamp_ns;
    Eigen::Vector3d p;
    Eigen::Quaterniond q;
    Eigen::Vector3d v;
};
```

`buildImuTrajectory()` 当前实现目标：

1. 输入 anchor 状态、`t_start`、`t_end`
2. 从 `imu_history_` 拷贝 `(t_anchor, t_end]` 范围内的 IMU 样本
3. 从 anchor 状态开始，按 IMU 时间顺序传播
4. 当传播跨过 `t_start` 时，插入第一个 scan 轨迹点
5. 后续每个 IMU 样本时刻写入一个 `ScanPoseSample`
6. 若最后一个 IMU 样本早于 `t_end`，用最后一条 IMU 近似传播到 `t_end`

传播模型与主 ESKF predict 保持一致：

$$\mathbf{a}_{world} = \mathbf{R}(\mathbf{q})\mathbf{a}_{body} + \mathbf{g}$$

$$\mathbf{p}_{k+1} = \mathbf{p}_k + \mathbf{v}_k\Delta t + \frac{1}{2}\mathbf{a}_{world}\Delta t^2$$

$$\mathbf{v}_{k+1} = \mathbf{v}_k + \mathbf{a}_{world}\Delta t$$

$$\mathbf{q}_{k+1} = \mathbf{q}_k \otimes \Delta \mathbf{q}(\boldsymbol{\omega}_{body}\Delta t)$$

这里的轨迹不是最终滤波状态，只用于一帧 LiDAR 内部的点云去畸变。

#### 14.2.7 Scan 轨迹查询

本轮新增 `lookupScanPose()`，用于在 `std::vector<ScanPoseSample>` 中按时间查询任意点对应的局部 pose：

$$\mathbf{p}(t) = (1 - \alpha)\mathbf{p}_a + \alpha\mathbf{p}_b$$

$$\mathbf{q}(t) = \operatorname{slerp}(\mathbf{q}_a, \mathbf{q}_b, \alpha)$$

其中：

$$\alpha = \frac{t - t_a}{t_b - t_a}$$

注意：这里仍然存在局部插值，但它发生在高频 IMU propagation 得到的 scan 内轨迹样本之间，不再是直接在稀疏 ESKF 状态历史上逐点查询。

#### 14.2.8 当前未完成部分

截至当前记录，`parseLidarFrame()` 尚未切换到新的 scan trajectory 主路径。也就是说：

- `ImuSample`
- `ScanPoseSample`
- `findAnchorState()`
- `buildImuTrajectory()`
- `lookupScanPose()`

这些基础设施已经写入代码，但实际点云 deskew 仍然使用旧路径：

$$\text{state\_history 插值} \rightarrow \text{旋转 deskew}$$

当前还没有完成：

1. 在 `parseLidarFrame()` 中调用 `findAnchorState()`
2. 在 `parseLidarFrame()` 中调用 `buildImuTrajectory()`
3. 使用 `lookupScanPose()` 获取 `t_i` 和 `t_{ref}` 的 pose
4. 将每个点执行完整 6DoF deskew：

$$\mathbf{p}_i^{body} = \mathbf{R}_{lidar}^{body}\mathbf{p}_i^{lidar}$$

$$\mathbf{p}_i^{world} = \mathbf{R}(\mathbf{q}(t_i))\mathbf{p}_i^{body} + \mathbf{p}(t_i)$$

$$\mathbf{p}_{i \rightarrow ref}^{body} =
\mathbf{R}(\mathbf{q}(t_{ref}))^T
(\mathbf{p}_i^{world} - \mathbf{p}(t_{ref}))$$

$$\mathbf{p}_{i \rightarrow ref}^{lidar} =
(\mathbf{R}_{lidar}^{body})^T\mathbf{p}_{i \rightarrow ref}^{body}$$

5. 停用或移除 `applyGicpTranslationDeskew()` 这条 GICP 后验补偿路径

#### 14.2.9 当前状态判断

本轮修复目前完成到“连续时间轨迹基础设施”阶段，还没有完成真正的 GICP 前 6DoF deskew。下一步应优先把 `parseLidarFrame()` 的 deskew 主体从旧的 `interpolateState()` 切换到 scan trajectory。

在完成切换前，系统的点云运动畸变表现仍然主要由旧的旋转 deskew 决定。

### 14.3 双线程与防积压修复

#### 14.3.1 问题背景

IMU 频率约 `200Hz`，LiDAR 频率约 `10Hz`，而 GICP 单次配准耗时可达 `30ms~50ms`。若 IMU 回调与 LiDAR/GICP 共用单线程执行器，则：

1. GICP 会阻塞 IMU 回调
2. IMU `dt` 被人为拉大
3. P 矩阵传播与姿态积分出现额外误差

#### 14.3.2 修复方案

1. LiDAR 订阅放入独立回调组
2. 使用 `MultiThreadedExecutor(2)` 让 IMU 与 LiDAR 并行执行
3. 加入防积压原子标志：

```cpp
if (gicp_running_.exchange(true)) {
    return;
}
```

语义为：若上一帧 GICP 尚未完成，则直接丢弃当前 LiDAR 帧，不允许 GICP 排队。

#### 14.3.3 设计理由

本系统里 IMU 是主时钟，LiDAR 是低频观测。若必须二选一，应优先保证 IMU predict 的连续性，而不是强行处理所有 LiDAR 帧。

因此，策略不是“排队等待所有 GICP 都算完”，而是“宁可丢 LiDAR 帧，也不要让 IMU 积分失真”。

### 14.4 GICP 时间基准与初值修复

#### 14.4.1 LiDAR 帧间隔

原始 GICP 观测若用 wall time 或回调触发时间算帧间隔，会受到调度、CPU 抢占、回放节奏影响。修复后统一使用点云时间戳：

$$\Delta t_{lidar} = t_{ref}(k) - t_{ref}(k-1)$$

其中 `t_ref = max_point_stamp_ns`。

这样 `v_icp` 与 `w_icp` 表示的就是传感器真实帧间速度，而不是 ROS 执行器观察到的回调间隔速度。

#### 14.4.2 GICP 初值

GICP 不再盲目复用上一帧配准结果，而是用 ESKF 历史估计相邻两帧 LiDAR 的先验相对运动：

$$\mathbf{T}_{source \rightarrow target}^{pred} = \mathbf{T}_{lidar}(t_{k-1})^{-1}\mathbf{T}_{lidar}(t_k)$$

这一步由 `estimateLidarMotion()` 生成，并写入 `gicp_init_guess_`。

#### 14.4.3 设计理由

GICP 初值越接近真实解，越不容易在弱纹理或转弯时收敛到错误局部极值。ESKF 负责给出近似先验，GICP 再用几何结构细化。

### 14.5 observeYaw 的时间一致性修复

#### 14.5.1 原问题

GICP 的 yaw 观测本质上是相邻 LiDAR 参考时刻之间的偏航变化：

$$\Delta \psi_{icp} = \psi(t_k) - \psi(t_{k-1})$$

若在 IMU 当前回调时直接用“当前 `q_` 减上一帧 LiDAR 姿态”构造残差，则观测时间跨度变成：

$$[t_{k-1}, t_{imu,now}]$$

而不是 GICP 实际对应的：

$$[t_{k-1}, t_k]$$

这会引入时间基准不一致，尤其在 IMU 高频、LiDAR 低频时更明显。

#### 14.5.2 修复方案

在 LiDAR 回调中，先用相同的两帧 LiDAR 参考时刻，从状态历史中取出姿态并计算预测 yaw 变化：

$$\Delta \psi_{pred} = \psi_{eskf}(t_k) - \psi_{eskf}(t_{k-1})$$

再构造 yaw 残差：

$$r_{\psi} = \Delta \psi_{icp} - \Delta \psi_{pred}$$

并将该残差缓存到 `icp_yaw_innovation_`。后续 IMU 回调中的 `iteratedObserve()` 不再重新推导时间跨度，而是直接消费这个已时间对齐的残差。

#### 14.5.3 设计理由

GICP 是“帧间观测”，不是“当前时刻绝对姿态观测”。因此，yaw 更新必须在与 GICP 完全相同的时间区间上构造残差，否则观测模型与观测数据不匹配。

这次修复的本质是把 `observeYaw` 从“当前 IMU 时刻修正”改成了“LiDAR 帧间残差修正”。

### 14.6 工程结论

这几次修复没有改变 ESKF 的基础状态定义与 predict/update 主框架，但修正了四类工程实现问题：

1. **IMU dt 必须来自传感器时间戳，而非 ROS 调度时间**
2. **LiDAR 点云必须先 deskew，才能满足 GICP 的刚体假设**
3. **IMU 与 GICP 必须线程隔离，并通过防积压避免低频重任务拖坏高频积分**
4. **yaw 观测必须用 LiDAR 帧间时间构造残差，不能和当前 IMU 时刻混用**

这些修复的共同目标是：让滤波器处理的每一类量都对应其真实物理时间与几何意义，而不是把 ROS 执行器行为、回放节奏或一帧点云内部的扫描过程误当成机器人本体运动。
