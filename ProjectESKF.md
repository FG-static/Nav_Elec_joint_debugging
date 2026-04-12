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
| 运算空间 | 非线性流形 | 线性向量空间 $\mathbb{R}^{18}$ |
| 数值稳定性 | 差 | 好（误差小，协方差矩阵行为良好） |

**关键结论**：误差状态 $\delta\boldsymbol{x}$ 始终很小，在零点附近泰勒展开的精度远高于在大信号处展开。这使得 ESKF 的线性化模型比 EKF 更精确。

---

## 二、状态量定义与物理意义

### 2.1 问题描述：IMU 外参

IMU 未严格安装在车辆中心，而是偏移了 $\boldsymbol{r} = [r_x, r_y, r_z]^T$（在车体系下表示）。这导致：

1. **加速度测量偏移**：IMU 测到的加速度 = 车体中心加速度 + 角速度叉乘项 + 角加速度叉乘项：
   $$\boldsymbol{a}_{imu} = \boldsymbol{a}_{center} + \dot{\boldsymbol{\omega}} \times \boldsymbol{r} + \boldsymbol{\omega} \times (\boldsymbol{\omega} \times \boldsymbol{r})$$
2. **速度转换偏移**：车体中心速度 $\neq$ IMU 位置速度：
   $$\boldsymbol{v}_{center} = \boldsymbol{v}_{imu} - \boldsymbol{\omega} \times \boldsymbol{r}$$

如果不估计 $\boldsymbol{r}$，转弯时加速度和速度的叉乘项将引入系统性误差，导致里程计在转弯时漂移。

### 2.2 状态向量

ESKF 将真实状态 $\boldsymbol{x}_t$ 分解为**名义状态** $\boldsymbol{x}$ 和**误差状态** $\delta\boldsymbol{x}$：

$$\boldsymbol{x}_t = \boldsymbol{x} \oplus \delta\boldsymbol{x}$$

其中 $\oplus$ 对位置/速度/零偏/外参是加法，对姿态是四元数乘法。

**名义状态**（6 组，共 20 参数，其中姿态 4 参数为四元数）：

| 符号 | 维度 | 物理意义 | 代码变量 |
|------|------|---------|---------|
| $\boldsymbol{p}$ | 3 | 世界系位置（车体中心） | `p_` |
| $\boldsymbol{v}$ | 3 | 世界系速度（车体中心） | `v_` |
| $\boldsymbol{q}$ | 4 | 姿态四元数（车体系→世界系） | `q_` |
| $\boldsymbol{b}_a$ | 3 | 加速度计零偏 | `b_a_` |
| $\boldsymbol{b}_g$ | 3 | 陀螺仪零偏 | `b_g_` |
| $\boldsymbol{r}$ | 3 | IMU 相对车体中心的偏移（车体系） | `r_` |

**误差状态**（6 组，共 18 维——姿态用 3 维旋转向量表示）：

$$\delta\boldsymbol{x} = \begin{bmatrix} \delta\boldsymbol{p} \\ \delta\boldsymbol{v} \\ \delta\boldsymbol{\theta} \\ \delta\boldsymbol{b}_a \\ \delta\boldsymbol{b}_g \\ \delta\boldsymbol{r} \end{bmatrix} \in \mathbb{R}^{18}$$

| 分量 | 维度 | 索引 | 物理意义 |
|------|------|------|---------|
| $\delta\boldsymbol{p}$ | 3 | 0–2 | 位置误差（世界系，米） |
| $\delta\boldsymbol{v}$ | 3 | 3–5 | 速度误差（世界系，m/s） |
| $\delta\boldsymbol{\theta}$ | 3 | 6–8 | 姿态误差角（车体系，弧度） |
| $\delta\boldsymbol{b}_a$ | 3 | 9–11 | 加速度计零偏误差（m/s²） |
| $\delta\boldsymbol{b}_g$ | 3 | 12–14 | 陀螺仪零偏误差（rad/s） |
| $\delta\boldsymbol{r}$ | 3 | 15–17 | IMU 外参误差（车体系，米） |

> **注意**：$\boldsymbol{r}$ 定义在车体系下。这是因为 IMU 固连在车体上，偏移量随车体旋转而旋转。ESKF 估计的是车体系下的固定偏移，物理意义更直观且可观测性更好。

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

### 2.4 IMU 测量模型（含外参）

当 IMU 不在车体中心时，IMU 测量到的加速度和角速度为：

**角速度**：IMU 刚性固连在车体上，所有点角速度相同：
$$\boldsymbol{\omega}_{imu} = \boldsymbol{\omega}_{center}$$

**加速度**：IMU 测到的比力包含向心加速度和欧拉加速度分量：
$$\boldsymbol{a}_{imu} = \boldsymbol{a}_{center} + \dot{\boldsymbol{\omega}} \times \boldsymbol{r} + \boldsymbol{\omega} \times (\boldsymbol{\omega} \times \boldsymbol{r})$$

其中：
- $\boldsymbol{a}_{center}$：车体中心的比力加速度（车体系）
- $\dot{\boldsymbol{\omega}} \times \boldsymbol{r}$：欧拉加速度（角加速度引起），由于 $\dot{\boldsymbol{\omega}}$ 不直接可测，通常将其视为过程噪声
- $\boldsymbol{\omega} \times (\boldsymbol{\omega} \times \boldsymbol{r})$：向心加速度（角速度引起），可由 IMU 角速度和当前估计的 $\boldsymbol{r}$ 计算

IMU 的实际测量值包含零偏和白噪声：
$$\boldsymbol{a}_m = \boldsymbol{a}_{imu} + \boldsymbol{b}_a + \boldsymbol{n}_a$$
$$\boldsymbol{\omega}_m = \boldsymbol{\omega}_{center} + \boldsymbol{b}_g + \boldsymbol{n}_g$$

展开得：
$$\boldsymbol{a}_m = \boldsymbol{a}_{center} + \dot{\boldsymbol{\omega}} \times \boldsymbol{r} + \boldsymbol{\omega} \times (\boldsymbol{\omega} \times \boldsymbol{r}) + \boldsymbol{b}_a + \boldsymbol{n}_a$$

> **IMU 特性**：静止时加速度计 z 轴读数 ≈ +9.8 m/s²（重力反作用力），并非 0。

### 2.5 速度关系（含外参）

车体中心速度与 IMU 位置速度的关系：

$$\boldsymbol{v}_{center}^{world} = \boldsymbol{v}_{imu}^{world} - \boldsymbol{R}(\boldsymbol{\omega} \times \boldsymbol{r})$$

其中 $\boldsymbol{R} = \boldsymbol{R}(\boldsymbol{q})$ 是车体系→世界系的旋转矩阵。车体系中的叉乘 $\boldsymbol{\omega} \times \boldsymbol{r}$ 转到世界系后从 IMU 速度中减去，即可得到车体中心速度。

---

## 三、名义状态传播（predict）

### 3.1 连续时间运动方程

IMU 补偿零偏和外参偏移后输出车体中心的"干净"加速度和角速度：

$$\boldsymbol{\omega}_{clean} = \boldsymbol{\omega}_m - \boldsymbol{b}_g$$

$$\boldsymbol{a}_{clean}^{center} = \boldsymbol{a}_m - \boldsymbol{b}_a - \boldsymbol{\omega}_{clean} \times (\boldsymbol{\omega}_{clean} \times \boldsymbol{r})$$

> **注意**：角加速度项 $\dot{\boldsymbol{\omega}} \times \boldsymbol{r}$ 无法直接测量，故不显式补偿，而是将其归入过程噪声 $\boldsymbol{Q}$ 中。对于地面麦轮底盘，角加速度通常较小（转弯速度变化缓慢），此近似合理。

名义状态的连续时间微分方程：

$$\dot{\boldsymbol{p}} = \boldsymbol{v}$$

$$\dot{\boldsymbol{v}} = \boldsymbol{R}(\boldsymbol{q}) \cdot \boldsymbol{a}_{clean}^{center} + \boldsymbol{g}$$

$$\dot{\boldsymbol{q}} = \frac{1}{2}\boldsymbol{q} \otimes \boldsymbol{\omega}_{clean}$$

$$\dot{\boldsymbol{r}} = \boldsymbol{0} \quad (\text{IMU 外参为常量})$$

$$\dot{\boldsymbol{b}}_a = \boldsymbol{0}, \quad \dot{\boldsymbol{b}}_g = \boldsymbol{0} \quad (\text{零偏为慢时变常量})$$

其中：
- $\boldsymbol{R}(\boldsymbol{q})$：四元数对应的旋转矩阵（车体系→世界系）
- $\boldsymbol{g} = [0, 0, -9.8]^T$：世界系重力加速度

### 3.2 离散时间积分

采用一阶欧拉法（IMU 频率 200Hz，dt≈5ms，精度足够）：

$$\boldsymbol{p}_{k+1} = \boldsymbol{p}_k + \boldsymbol{v}_k \Delta t + \frac{1}{2}(\boldsymbol{R}_k \boldsymbol{a}_{clean}^{center} + \boldsymbol{g})\Delta t^2$$

$$\boldsymbol{v}_{k+1} = \boldsymbol{v}_k + (\boldsymbol{R}_k \boldsymbol{a}_{clean}^{center} + \boldsymbol{g})\Delta t$$

$$\boldsymbol{q}_{k+1} = \boldsymbol{q}_k \otimes \Delta\boldsymbol{q}(\boldsymbol{\omega}_{clean} \Delta t)$$

$$\boldsymbol{r}_{k+1} = \boldsymbol{r}_k$$

其中角增量四元数：

$$\Delta\boldsymbol{q} = \begin{bmatrix} \sin(\frac{\|\Delta\boldsymbol{\theta}\|}{2}) \frac{\Delta\boldsymbol{\theta}}{\|\Delta\boldsymbol{\theta}\|} \\ \cos(\frac{\|\Delta\boldsymbol{\theta}\|}{2}) \end{bmatrix}, \quad \Delta\boldsymbol{\theta} = \boldsymbol{\omega}_{clean} \Delta t$$

> **代码实现参考**（`data_handle.cpp`，15维版本，扩展后需修改）：
> ```cpp
> // 补偿零偏
> Eigen::Vector3d acc = acc_raw - b_a_;
> Eigen::Vector3d w = w_raw - b_g_;
> // 补偿外参：减去向心加速度项
> Eigen::Vector3d acc_center = acc - w.cross(w.cross(r_));
> // 名义状态积分
> p_ = p_ + v_ * dt + 0.5 * (q_ * acc_center + G_VEC_) * dt * dt;
> v_ = v_ + (q_ * acc_center + G_VEC_) * dt;
> ```

### 3.3 误差状态传播方程

对误差状态在名义状态附近做一阶泰勒展开：

$$\delta\dot{\boldsymbol{x}} = \boldsymbol{F} \delta\boldsymbol{x} + \boldsymbol{G}\boldsymbol{w}$$

其中系统矩阵 $\boldsymbol{F} \in \mathbb{R}^{18 \times 18}$：

$$\boldsymbol{F} = \begin{bmatrix} \boldsymbol{0} & \boldsymbol{I} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} \\ \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{F}_{\boldsymbol{v}\boldsymbol{\theta}} & -\boldsymbol{R} & \boldsymbol{0} & \boldsymbol{F}_{\boldsymbol{v}\boldsymbol{r}} \\ \boldsymbol{0} & \boldsymbol{0} & -[\boldsymbol{\omega}_{clean}]_\times & \boldsymbol{0} & -\boldsymbol{I} & \boldsymbol{0} \\ \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} \\ \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} \\ \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} \end{bmatrix}$$

**各分块的物理意义推导**：

#### $\delta\dot{\boldsymbol{v}}$ 对 $\delta\boldsymbol{\theta}$ 的偏导：$\boldsymbol{F}_{\boldsymbol{v}\boldsymbol{\theta}}$

速度传播方程：$\dot{\boldsymbol{v}} = \boldsymbol{R}(\boldsymbol{q})\boldsymbol{a}_{clean}^{center} + \boldsymbol{g}$

其中 $\boldsymbol{a}_{clean}^{center} = \boldsymbol{a}_{clean}^{imu} - \boldsymbol{\omega}_{clean} \times (\boldsymbol{\omega}_{clean} \times \boldsymbol{r})$，记 $\boldsymbol{a}_{cpr} = \boldsymbol{\omega}_{clean} \times (\boldsymbol{\omega}_{clean} \times \boldsymbol{r})$。

当姿态有扰动 $\delta\boldsymbol{\theta}$（车体系，右乘）时，$\boldsymbol{R}$ 和 $\boldsymbol{r}$ 都受影响：

1. $\boldsymbol{R}$ 的变化：$\boldsymbol{R}_t = \boldsymbol{R}(\boldsymbol{I} - [\delta\boldsymbol{\theta}]_\times)$
2. $\boldsymbol{r}$ 定义在车体系下，在世界系中为 $\boldsymbol{R}\boldsymbol{r}$。当姿态扰动后，世界系下的 $\boldsymbol{r}$ 变为 $\boldsymbol{R}_t\boldsymbol{r} = \boldsymbol{R}(\boldsymbol{I} - [\delta\boldsymbol{\theta}]_\times)\boldsymbol{r}$。但 $\boldsymbol{a}_{cpr}$ 在车体系内计算，$\boldsymbol{\omega}$ 也在车体系，$\boldsymbol{r}$ 在车体系——$\delta\boldsymbol{\theta}$ 不改变车体系内的 $\boldsymbol{r}$ 值（$\boldsymbol{r}$ 是车体系下的常数），因此 $\boldsymbol{a}_{cpr}$ 不受 $\delta\boldsymbol{\theta}$ 影响（一阶近似下）。

所以只需考虑 $\boldsymbol{R}$ 的变化：

$$\dot{\boldsymbol{v}}_t = \boldsymbol{R}_t \boldsymbol{a}_{clean}^{center} + \boldsymbol{g} = (\boldsymbol{R} - \boldsymbol{R}[\delta\boldsymbol{\theta}]_\times)\boldsymbol{a}_{clean}^{center} + \boldsymbol{g}$$

$$\delta\dot{\boldsymbol{v}} = -\boldsymbol{R}[\delta\boldsymbol{\theta}]_\times \boldsymbol{a}_{clean}^{center} = -\boldsymbol{R}[\boldsymbol{a}_{clean}^{center}]_\times \delta\boldsymbol{\theta}$$

因此：

$$\boxed{\boldsymbol{F}_{\boldsymbol{v}\boldsymbol{\theta}} = -\boldsymbol{R}[\boldsymbol{a}_{clean}^{center}]_\times}$$

> 与 15 维版本的 $-\boldsymbol{R}[\boldsymbol{a}_{clean}]_\times$ 形式相同，只是 $\boldsymbol{a}_{clean}$ 替换为补偿外参后的 $\boldsymbol{a}_{clean}^{center}$。

#### $\delta\dot{\boldsymbol{v}}$ 对 $\delta\boldsymbol{r}$ 的偏导：$\boldsymbol{F}_{\boldsymbol{v}\boldsymbol{r}}$

当外参有误差 $\delta\boldsymbol{r}$ 时，向心加速度补偿项变为：

$$\boldsymbol{a}_{cpr}(\boldsymbol{r} + \delta\boldsymbol{r}) = \boldsymbol{\omega} \times (\boldsymbol{\omega} \times (\boldsymbol{r} + \delta\boldsymbol{r}))$$

误差为：

$$\delta\boldsymbol{a}_{cpr} = \boldsymbol{\omega} \times (\boldsymbol{\omega} \times \delta\boldsymbol{r})$$

这个误差在车体系下，需要转到世界系：

$$\delta\dot{\boldsymbol{v}} = -\boldsymbol{R} \cdot \delta\boldsymbol{a}_{cpr} = -\boldsymbol{R}[\boldsymbol{\omega} \times (\boldsymbol{\omega} \times \delta\boldsymbol{r})]$$

利用叉乘的线性性质，对 $\delta\boldsymbol{r}$ 取偏导：

$$\boldsymbol{F}_{\boldsymbol{v}\boldsymbol{r}} = \frac{\partial \delta\dot{\boldsymbol{v}}}{\partial \delta\boldsymbol{r}} = -\boldsymbol{R} \cdot \frac{\partial [\boldsymbol{\omega} \times (\boldsymbol{\omega} \times \delta\boldsymbol{r})]}{\partial \delta\boldsymbol{r}}$$

注意到 $\boldsymbol{\omega} \times (\boldsymbol{\omega} \times \delta\boldsymbol{r}) = [\boldsymbol{\omega}]_\times^2 \delta\boldsymbol{r}$，其中 $[\boldsymbol{\omega}]_\times^2 = \boldsymbol{\omega}\boldsymbol{\omega}^T - \|\boldsymbol{\omega}\|^2\boldsymbol{I}$：

$$\boxed{\boldsymbol{F}_{\boldsymbol{v}\boldsymbol{r}} = -\boldsymbol{R}[\boldsymbol{\omega}_{clean}]_\times^2 = -\boldsymbol{R}(\boldsymbol{\omega}_{clean}\boldsymbol{\omega}_{clean}^T - \|\boldsymbol{\omega}_{clean}\|^2\boldsymbol{I})}$$

> **物理意义**：当 IMU 偏移量估计不准时，向心加速度补偿不完整，剩余的向心加速度误差经 $\boldsymbol{R}$ 转到世界系后影响速度。$\|\boldsymbol{\omega}\|^2$ 项说明角速度越大，外参误差对速度的影响越显著——这正是转弯漂移的根源。

#### $\delta\dot{\boldsymbol{v}}$ 对 $\delta\boldsymbol{b}_a$ 的偏导：$-\boldsymbol{R}$

与 15 维版本相同：

$$\boldsymbol{a}_{clean}^{*} = \boldsymbol{a}_m - (\boldsymbol{b}_a + \delta\boldsymbol{b}_a) - \boldsymbol{\omega} \times (\boldsymbol{\omega} \times \boldsymbol{r}) = \boldsymbol{a}_{clean}^{center} - \delta\boldsymbol{b}_a$$

$$\delta\dot{\boldsymbol{v}} = -\boldsymbol{R}\delta\boldsymbol{b}_a$$

#### $\delta\dot{\boldsymbol{\theta}}$ 对各误差的偏导

与 15 维版本相同，$\boldsymbol{r}$ 不影响角速度传播：

$$\frac{\partial \delta\dot{\boldsymbol{\theta}}}{\partial \delta\boldsymbol{\theta}} = -[\boldsymbol{\omega}_{clean}]_\times, \quad \frac{\partial \delta\dot{\boldsymbol{\theta}}}{\partial \delta\boldsymbol{b}_g} = -\boldsymbol{I}$$

#### $\delta\dot{\boldsymbol{r}}$ 对各误差的偏导

$\boldsymbol{r}$ 为常量，$\delta\dot{\boldsymbol{r}} = \boldsymbol{0}$，所有偏导为零。

### 3.4 离散化状态转移矩阵

一阶近似：$\boldsymbol{\Phi} \approx \boldsymbol{I} + \boldsymbol{F}\Delta t$

$$\boldsymbol{\Phi} = \begin{bmatrix} \boldsymbol{I} & \boldsymbol{I}\Delta t & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} \\ \boldsymbol{0} & \boldsymbol{I} & -\boldsymbol{R}[\boldsymbol{a}_{c}^{center}]_\times\Delta t & -\boldsymbol{R}\Delta t & \boldsymbol{0} & -\boldsymbol{R}[\boldsymbol{\omega}]_\times^2\Delta t \\ \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{I} - [\boldsymbol{\omega}_{clean}]_\times\Delta t & \boldsymbol{0} & -\boldsymbol{I}\Delta t & \boldsymbol{0} \\ \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{I} & \boldsymbol{0} & \boldsymbol{0} \\ \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{I} & \boldsymbol{0} \\ \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{I} \end{bmatrix}$$

> **代码实现参考**（18维版本）：
> ```cpp
> Eigen::Matrix<double, 18, 18> Fx = Eigen::Matrix<double, 18, 18>::Identity();
> Eigen::Matrix3d R = q_.toRotationMatrix();
> Eigen::Matrix3d w_cross = skew_symmetric(w);
> Eigen::Matrix3d w_cross_sq = w * w.transpose() - w.squaredNorm() * Eigen::Matrix3d::Identity();
>
> Fx.block<3, 3>(0, 3)   = Eigen::Matrix3d::Identity() * dt;              // ∂δp/∂δv
> Fx.block<3, 3>(3, 6)   = -R * skew_symmetric(acc_center) * dt;          // ∂δv/∂δθ
> Fx.block<3, 3>(3, 9)   = -R * dt;                                        // ∂δv/∂δb_a
> Fx.block<3, 3>(6, 6)   = Eigen::Matrix3d::Identity() - w_cross * dt;    // ∂δθ/∂δθ
> Fx.block<3, 3>(6, 12)  = -Eigen::Matrix3d::Identity() * dt;             // ∂δθ/∂δb_g
> Fx.block<3, 3>(3, 15)  = -R * w_cross_sq * dt;                          // ∂δv/∂δr  ★新增
> ```

### 3.5 协方差预测

$$\boldsymbol{P}_{k|k-1} = \boldsymbol{\Phi}\boldsymbol{P}_{k-1|k-1}\boldsymbol{\Phi}^T + \boldsymbol{Q}_d$$

其中 $\boldsymbol{Q}_d \in \mathbb{R}^{18 \times 18}$ 是离散化过程噪声矩阵：

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
| $v_x^{body}$ | 麦轮运动学 | 车体系前进速度（车体中心） |
| $v_y^{body}$ | 麦轮运动学 | 车体系横向速度（车体中心） |
| $v_z^{body}$ | 约束为 0 | 地面机器人 z 速度≈0（零速观测） |
| $\omega_z^{body}$ | 麦轮运动学 | 车体系偏航角速度 |

> **注意**：麦轮运动学反算的是车体中心的线速度，而非 IMU 位置的线速度。因此观测模型需要将 ESKF 的车体中心速度（状态量）与轮速观测对齐。

#### 4.1.2 麦轮运动学

四轮麦克纳姆轮的运动学关系：

$$v_x = \frac{r}{4}(w_{fl} + w_{fr} + w_{rl} + w_{rr})$$

$$v_y = \frac{r}{4}(-w_{fl} + w_{fr} + w_{rl} - w_{rr})$$

$$\omega_z = \frac{r}{4(l_x + l_y)}(-w_{fl} + w_{fr} - w_{rl} + w_{rr})$$

参数：
- $r = 0.0815$ m（轮半径）
- $l_x + l_y \approx 0.3005$ m（轮对角线半距离之和）

> 代码实现（`data_handle.cpp:241-249`）：
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

ESKF 的速度状态 $\boldsymbol{v}$ 是车体中心的**世界系**速度，需要转到车体系：

$$h_v(\boldsymbol{x}) = \boldsymbol{R}^T \boldsymbol{v}$$

> **注意**：这里不需要额外的外参补偿，因为 $\boldsymbol{v}$ 已经是车体中心速度（在 predict 中已通过向心加速度补偿将 IMU 加速度转换为车体中心加速度）。

**角速度部分**（第 4 维）：

轮速反算的 $\omega_z$ 与 IMU 测量（已补偿零偏）的陀螺仪 z 轴输出应该一致：

$$h_w(\boldsymbol{x}) = \omega_{z,gyro} - b_{g,z}$$

综合观测预测值：

$$\boldsymbol{h}(\boldsymbol{x}) = \begin{bmatrix} \boldsymbol{R}^T\boldsymbol{v} \\ \omega_{z,gyro} - b_{g,z} \end{bmatrix}$$

#### 4.1.4 雅可比矩阵 $\boldsymbol{H}$ 的推导

$\boldsymbol{H} = \frac{\partial \boldsymbol{h}}{\partial \delta\boldsymbol{x}} \in \mathbb{R}^{4 \times 18}$

**速度部分** $h_v = \boldsymbol{R}^T\boldsymbol{v}$ 对各误差状态的偏导：

**(a) 对 $\delta\boldsymbol{v}$ 的偏导**：

$\boldsymbol{v}_t = \boldsymbol{v} + \delta\boldsymbol{v}$，$\boldsymbol{R}$ 不受 $\delta\boldsymbol{v}$ 影响：

$$\frac{\partial h_v}{\partial \delta\boldsymbol{v}} = \boldsymbol{R}^T$$

**(b) 对 $\delta\boldsymbol{\theta}$ 的偏导**：

当姿态有扰动 $\delta\boldsymbol{\theta}$（车体系，右乘）时：

$$\boldsymbol{R}_t^T = (\boldsymbol{I} + [\delta\boldsymbol{\theta}]_\times)\boldsymbol{R}^T$$

$$h_v(\boldsymbol{x}_t) = \boldsymbol{R}^T\boldsymbol{v} + [\delta\boldsymbol{\theta}]_\times \boldsymbol{R}^T\boldsymbol{v}$$

$$\frac{\partial h_v}{\partial \delta\boldsymbol{\theta}} = -[\boldsymbol{R}^T\boldsymbol{v}]_\times = -[\boldsymbol{v}_{body}]_\times$$

**(c) 对 $\delta\boldsymbol{r}$ 的偏导**：

$h_v$ 不显式依赖 $\boldsymbol{r}$（速度状态 $\boldsymbol{v}$ 已经是车体中心速度），因此：

$$\frac{\partial h_v}{\partial \delta\boldsymbol{r}} = \boldsymbol{0}$$

> **深入分析**：虽然 $\boldsymbol{v}$ 在 predict 步骤中已受 $\boldsymbol{r}$ 影响（通过向心加速度补偿），但在 ESKF 框架下，$h(\boldsymbol{x})$ 对 $\delta\boldsymbol{x}$ 求导时，$\boldsymbol{x}$ 是名义状态（固定值），$\delta\boldsymbol{x}$ 是误差状态（变量）。$\boldsymbol{v}$ 作为名义状态的一部分，在求导时视为常数。$\delta\boldsymbol{r}$ 对 $h_v$ 的影响已经在 predict 步骤的 $\boldsymbol{F}_{\boldsymbol{v}\boldsymbol{r}}$ 中体现（$\delta\boldsymbol{r}$ 影响 $\delta\boldsymbol{v}$，$\delta\boldsymbol{v}$ 再影响 $h_v$）。

**角速度部分** $h_w = \omega_{z,gyro} - b_{g,z}$ 对各误差状态的偏导：

**(d) 对 $\delta\boldsymbol{b}_g$ 的偏导**：

$$\frac{\partial h_w}{\partial \delta b_{g,z}} = -1$$

**综合雅可比矩阵**：

$$\boldsymbol{H} = \begin{bmatrix} \boldsymbol{0}_{3\times3} & \boldsymbol{R}^T & -[\boldsymbol{v}_{body}]_\times & \boldsymbol{0}_{3\times3} & \boldsymbol{0}_{3\times3} & \boldsymbol{0}_{3\times3} \\ \boldsymbol{0}_{1\times3} & \boldsymbol{0}_{1\times3} & \boldsymbol{0}_{1\times3} & \boldsymbol{0}_{1\times3} & [0\;0\;{-1}] & \boldsymbol{0}_{1\times3} \end{bmatrix}$$

> 与 15 维版本相比，$\boldsymbol{H}$ 只是在右侧增加了 3 列零（对应 $\delta\boldsymbol{r}$），这是因为轮速观测的是车体中心速度，而 ESKF 状态中的速度已经是车体中心的。

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

旋转矩阵使用 ZYX 欧拉角分解：

$$\boldsymbol{R} = \boldsymbol{R}_z(yaw) \cdot \boldsymbol{R}_y(-pitch) \cdot \boldsymbol{R}_x(roll)$$

当存在车体系扰动 $\delta\boldsymbol{\theta}$ 时，与 15 维版本推导完全相同：

对 pitch（= $-\arcsin(R_{31})$），当 pitch≈0, roll≈0 时：

$$\delta(pitch) \approx \delta\theta_y$$

对 roll（= $\arctan2(R_{32}, R_{33})$），当 pitch≈0 时：

$$\delta(roll) \approx \delta\theta_x$$

因此雅可比矩阵为（注意维度扩展为 $2 \times 18$）：

$$\boldsymbol{H}_{tilt} = \begin{bmatrix} \boldsymbol{0}_{1\times6} & 0 & 1 & 0 & \boldsymbol{0}_{1\times9} \\ \boldsymbol{0}_{1\times6} & 1 & 0 & 0 & \boldsymbol{0}_{1\times9} \end{bmatrix}$$

> 与 15 维版本相比，只是在右侧增加了 3 列零（对应 $\delta\boldsymbol{b}_a$、$\delta\boldsymbol{b}_g$、$\delta\boldsymbol{r}$），因为 pitch/roll 只与 $\delta\boldsymbol{\theta}$ 有关。

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

$$\boldsymbol{r} \leftarrow \boldsymbol{r} + \delta\boldsymbol{r}$$

其中角度注入使用轴角→四元数：

$$\Delta\boldsymbol{q}(\delta\boldsymbol{\theta}) = \begin{bmatrix} \sin(\frac{\|\delta\boldsymbol{\theta}\|}{2})\frac{\delta\boldsymbol{\theta}}{\|\delta\boldsymbol{\theta}\|} \\ \cos(\frac{\|\delta\boldsymbol{\theta}\|}{2}) \end{bmatrix}$$

> **代码实现参考**（18维版本）：
> ```cpp
> p_ += delta_x_.segment<3>(0);
> v_ += delta_x_.segment<3>(3);
> b_a_ += delta_x_.segment<3>(9);
> b_g_ += delta_x_.segment<3>(12);
> r_ += delta_x_.segment<3>(15);  // ★新增
> Eigen::Vector3d dtheta = delta_x_.segment<3>(6);
> if (dtheta.norm() > 1e-10) {
>     Eigen::Quaterniond dq(Eigen::AngleAxisd(dtheta.norm(), dtheta.normalized()));
>     q_ = (q_ * dq).normalized();
> }
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
│   ├── 补偿外参：acc_center = acc - w×(w×r_)     ★新增
│   ├── 名义状态积分：p_, v_, q_, r_ 不变
│   ├── 计算状态转移矩阵 Fx (18×18)：
│   │   ├── Fx(3,6)  = -R * [acc_center]× * dt    （修改：acc→acc_center）
│   │   └── Fx(3,15) = -R * [w]×² * dt            ★新增
│   └── 协方差预测：P = Fx·P·Fx^T + Q (18×18)
│
├── 2. observeWheel(msg)
│   ├── 麦轮运动学计算观测量 y = [vx, vy, 0, wz]
│   ├── 计算观测预测 h(x) = [R^T·v, gyro_z - b_g_z]
│   ├── 计算雅可比矩阵 H (4×18)，δr 列为零
│   ├── 卡尔曼增益 K = P·H^T·(H·P·H^T + R)^{-1}
│   ├── 更新误差状态 δx += K·(y - h)
│   └── 更新协方差 P（Joseph 形式）
│
├── 3. observeZeroTilt()
│   ├── 提取 pitch, roll
│   ├── 观测量 y = [0, 0]，预测 h = [pitch, roll]
│   ├── 雅可比 H (2×18)：∂pitch/∂δθ_y = 1, ∂roll/∂δθ_x = 1，δr 列为零
│   ├── K = P·H^T·(H·P·H^T + R_tilt)^{-1}
│   ├── δx += K·(y - h)
│   └── 更新协方差 P（Joseph 形式）
│
├── 4. injectAndReset()
│   ├── 将 δx 注入名义状态：p_, v_, q_, b_a_, b_g_, r_  ★新增 r_
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
| $\boldsymbol{P}_0$ | `Identity * 0.01` | 18×18 | 初始状态不确定性 | 初始位姿越不确定应越大；$\delta\boldsymbol{r}$ 分量的初始不确定性取决于 IMU 安装精度 |
| $\boldsymbol{Q}$ | `Identity * 0.005` | 18×18 | 过程噪声 | IMU 精度越低应越大；$\delta\boldsymbol{r}$ 对应行应设小值（外参为常量） |
| $\boldsymbol{R}$ | `4×4 Identity * 0.005` | 4×4 | 轮速观测噪声 | 轮速打滑越多应越大 |
| $\boldsymbol{R}_{tilt}$ | `2×2 Identity * 0.005` | 2×2 | 零倾斜观测噪声 | 值越小→约束越强→pitch/roll越稳定 |

### 外参相关参数调参建议

| 参数 | 建议值 | 说明 |
|------|--------|------|
| $P_0$ 的 $\delta\boldsymbol{r}$ 分量 | `0.01` ~ `0.1` | 初始外参不确定性（米²）。若 IMU 位置已知大概，可设较小值 |
| $Q$ 的 $\delta\boldsymbol{r}$ 分量 | `1e-6` ~ `1e-4` | 外参为常量，过程噪声应极小。过大会导致 $\boldsymbol{r}$ 在运行中持续变化 |

**调参原则**：

- $\boldsymbol{Q}$ ↑ → 更信任观测（轮速），IMU 漂移被更快修正，但里程计更抖
- $\boldsymbol{R}$ ↑ → 更信任 IMU，轮速修正弱，直线性好但可能漂移
- $\boldsymbol{R}_{tilt}$ ↓ → 零倾斜约束强，pitch/roll 被锁死，但上坡时不灵活
- $Q_{\delta\boldsymbol{r}}$ ↑ → 外参估计更灵活，但可能震荡；↓ → 外参更稳定，但收敛慢

---

## 九、坐标系约定

```
世界系 (odom)：          车体系 (base_footprint)：
  ↑ z                      ↑ z (上)
  |                        |
  +---→ y                  +---→ y (左)
 /                        /
↙ x                      ↙ x (前)

IMU 位置偏移 r_ = [rx, ry, rz]^T（车体系下）：
  rx: IMU 相对车体中心的前方偏移（正值=IMU在前方）
  ry: IMU 相对车体中心的左方偏移（正值=IMU在左方）
  rz: IMU 相对车体中心的上方偏移（正值=IMU在上方）

IMU 输出已在车体系（电控已转换）：
  gyro_x: 绕车体 x 轴（前后方向）= 俯仰角速度
  gyro_y: 绕车体 y 轴（左右方向）= 横滚角速度
  gyro_z: 绕车体 z 轴（上下方向）= 偏航角速度
  acc_x: 沿车体前方加速度（IMU 位置处，含向心加速度）
  acc_y: 沿车体左方加速度（IMU 位置处，含向心加速度）
  acc_z: 沿车体上方加速度（IMU 位置处，静止时 ≈ +9.8 m/s²）

四元数 q_：车体系 → 世界系
  v_world = q_ * v_body
  v_body  = q_.inverse() * v_world = R^T * v_world

速度关系：
  v_center = v_imu - ω × r_      （车体系下）
  v_center_world = v_imu_world - R * (ω × r_)  （世界系下）
```

---

## 十、IMU 外参可观测性分析

### 10.1 可观测性条件

$\delta\boldsymbol{r}$ 能否被 ESKF 正确估计，取决于它是否可观测。分析传播链路：

$$\delta\boldsymbol{r} \xrightarrow{\boldsymbol{F}_{\boldsymbol{v}\boldsymbol{r}} = -\boldsymbol{R}[\boldsymbol{\omega}]_\times^2} \delta\boldsymbol{v} \xrightarrow{\boldsymbol{H}_v = \boldsymbol{R}^T} \text{轮速观测}$$

关键：$\boldsymbol{F}_{\boldsymbol{v}\boldsymbol{r}} = -\boldsymbol{R}[\boldsymbol{\omega}]_\times^2$ 中包含 $[\boldsymbol{\omega}]_\times^2 = \boldsymbol{\omega}\boldsymbol{\omega}^T - \|\boldsymbol{\omega}\|^2\boldsymbol{I}$。

- **静止时**（$\boldsymbol{\omega} \approx \boldsymbol{0}$）：$[\boldsymbol{\omega}]_\times^2 \approx \boldsymbol{0}$，$\delta\boldsymbol{r}$ 对 $\delta\boldsymbol{v}$ 无影响，**不可观测**
- **匀速旋转时**（$\boldsymbol{\omega}$ 恒定）：$[\boldsymbol{\omega}]_\times^2 \neq \boldsymbol{0}$，向心加速度耦合出速度误差，**可观测**
- **仅绕 z 轴旋转时**（$\boldsymbol{\omega} = [0, 0, \omega_z]^T$）：

$$[\boldsymbol{\omega}]_\times^2 = \begin{bmatrix} -\omega_z^2 & 0 & 0 \\ 0 & -\omega_z^2 & 0 \\ 0 & 0 & 0 \end{bmatrix}$$

可见 $\omega_z$ 使 $r_x$ 和 $r_y$ 可观测，但 $r_z$ 不可观测（需要 pitch/roll 角速度才能激发 $r_z$ 的可观测量）。

### 10.2 实际可观测性判断

| 外参分量 | 可观测条件 | 麦轮底盘实际 | 结论 |
|----------|-----------|-------------|------|
| $r_x$ | 存在 $\omega_z$（偏航角速度） | 转弯时 $\omega_z$ 显著 | ✅ 可观测 |
| $r_y$ | 存在 $\omega_z$（偏航角速度） | 转弯时 $\omega_z$ 显著 | ✅ 可观测 |
| $r_z$ | 存在 $\omega_x$ 或 $\omega_y$（俯仰/横滚角速度） | 地面机器人 $\omega_x, \omega_y \approx 0$ | ❌ 弱可观测 |

> **建议**：对于地面麦轮底盘，$r_z$ 的可观测性很弱。可以：
> 1. 将 $r_z$ 固定为测量值，仅在线估计 $r_x$ 和 $r_y$
> 2. 或在 $Q$ 中给 $\delta r_z$ 设极小值，使其几乎不变
> 3. 若后续增加上坡/过坎场景（产生 $\omega_x, \omega_y$），$r_z$ 的可观测性会改善

---

## 十一、关键设计决策总结

| 决策 | 理由 |
|------|------|
| 使用 ESKF 而非 EKF | 误差状态始终很小，线性化精度高；姿态用 3 维旋转向量表示，无过约束 |
| $\delta\boldsymbol{\theta}$ 定义在车体系（右乘） | 与 IMU 在车体系测量一致，雅可比矩阵推导更自然 |
| $\boldsymbol{r}$ 定义在车体系 | IMU 固连于车体，车体系下偏移为常量，物理意义直观 |
| 轮速观测在车体系 | 麦轮运动学直接输出车体系速度，无需旋转，避免坐标系混淆 |
| wz 纳入轮速观测 | 为 $\delta b_{g,z}$ 提供观测通路，帮助陀螺仪 z 轴零偏收敛 |
| 零倾斜观测 | 地面机器人 pitch/roll≈0 是强先验，直接约束角度抖动和 z 轴漂移 |
| Joseph 形式更新协方差 | 数值稳定，保证 $\boldsymbol{P}$ 对称正定 |
| 用 MCU 时间戳 `t_ms` 算 dt | 比 ROS `now()` 更精确（无 USB 传输延迟抖动） |
| 不显式补偿 $\dot{\boldsymbol{\omega}} \times \boldsymbol{r}$ | 角加速度不可直接测量；地面底盘角加速度较小，归入过程噪声 |
| 18 维状态向量含 $\delta\boldsymbol{r}$ | IMU 偏移导致转弯时系统性加速度误差，必须在线估计才能消除 |
