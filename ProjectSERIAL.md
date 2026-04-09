# 项目串口通信技术文档

> 基于 `src/serial_driver/` 和 `src/my_nav2_robot/` 的完整实现分析

---

## 一、系统架构总览

### 1.1 数据流全景

```
电控 MCU ──USB CDC──> rm_serial_driver ──/tracker/gimbal──> my_nav2_robot(data_handle)
        (串口驱动+解帧)   (串口驱动+解帧)                    (ESKF 里程计)
                        <──/tracker/target──                 ──/odom──> Nav2
                        (下行指令)                           ──/path──> RViz
                                                             ──TF: odom→base_footprint
```

### 1.2 涉及的 ROS 2 包

| 包名 | 路径 | 职责 |
|------|------|------|
| `rm_serial_driver` | `src/serial_driver/` | 串口通信、帧解析、CRC 校验、上下行数据桥接 |
| `rm_interfaces` | `src/rm_interfaces/` | 自定义消息定义（`Gimbal.msg`、`Target.msg`） |
| `my_nav2_robot` | `src/my_nav2_robot/` | 数据处理（ESKF 里程计）、导航配置 |

### 1.3 话题拓扑

| 话题 | 消息类型 | 发布者 | 订阅者 | QoS | 方向 |
|------|---------|--------|--------|-----|------|
| `/tracker/gimbal` | `rm_interfaces/msg/Gimbal` | `rm_serial_driver` | `data_handle_node` | `SensorDataQoS` | 上行（MCU→导航） |
| `/tracker/target` | `rm_interfaces/msg/Target` | `data_handle_node` | `rm_serial_driver` | `SensorDataQoS`(下行) / `10`(上行) | 下行（导航→MCU） |
| `/odom` | `nav_msgs/msg/Odometry` | `data_handle_node` | Nav2 | `10` | 内部 |
| `/path` | `nav_msgs/msg/Path` | `data_handle_node` | RViz | `10` | 内部 |

---

## 二、通信协议详解

### 2.1 物理层

| 参数 | 值 | 说明 |
|------|----|------|
| 接口类型 | USB CDC（虚拟串口） | MCU 端实现 USB CDC 类，上位机识别为 `/dev/ttyACM*` |
| 默认设备路径 | `/tmp/ttyACM0` | 由 `serial_driver.yaml` 配置 |
| 波特率 | 115200 | USB CDC 实际不受波特率限制，此值为兼容性配置 |
| 数据位 | 8 | 默认 |
| 校验位 | None | 无奇偶校验 |
| 停止位 | 1 | 默认 |
| 流控 | None | 无硬件/软件流控 |

### 2.2 帧格式（上行：MCU→导航）

```
字节偏移:  0     1     2     3     4 ... 49    50    51
内容:    [SOF0][SOF1][seq][len][   payload  ][CRC_L][CRC_H]
           A5    5A   1B   1B      46 B        2 B
           └──────────── 帧头 ────────────┘   └── CRC ──┘

总帧长: 2 + 1 + 1 + 46 + 2 = 52 字节
```

| 字段 | 偏移 | 长度 | 值/类型 | 说明 |
|------|------|------|---------|------|
| SOF0 | 0 | 1 | `0xA5` | 帧头第 1 字节，固定值 |
| SOF1 | 1 | 1 | `0x5A` | 帧头第 2 字节，固定值 |
| seq | 2 | 1 | `uint8` | 序列号，0~255 循环，用于检测丢帧 |
| len | 3 | 1 | `uint8 = 46` | payload 长度，固定 46 字节 |
| payload | 4–49 | 46 | 见下表 | 数据负载 |
| CRC_L | 50 | 1 | `uint8` | CRC16-CCITT 低字节（小端序） |
| CRC_H | 51 | 1 | `uint8` | CRC16-CCITT 高字节（小端序） |

**CRC 校验范围**：帧的前 50 字节（`frame[0..49]`），即 SOF0 + SOF1 + seq + len + payload。CRC 值本身不参与计算。

### 2.3 Payload 内存布局（上行，46 字节）

使用 `__attribute__((packed))` 确保无对齐填充，小端序（Little-Endian）：

| 偏移 | 长度 | 类型 | 字段 | 单位 | 说明 |
|------|------|------|------|------|------|
| 0–3 | 4 | `uint32_t` | `t_ms` | ms | MCU 采样时间戳 |
| 4–7 | 4 | `float` | `w_fl` | rad/s | 前左轮角速度（ESC ID 2） |
| 8–11 | 4 | `float` | `w_fr` | rad/s | 前右轮角速度（ESC ID 1） |
| 12–15 | 4 | `float` | `w_rl` | rad/s | 后左轮角速度（ESC ID 3） |
| 16–19 | 4 | `float` | `w_rr` | rad/s | 后右轮角速度（ESC ID 4） |
| 20–23 | 4 | `float` | `gyro_x` | rad/s | 底盘车体系 x 轴角速度 |
| 24–27 | 4 | `float` | `gyro_y` | rad/s | 底盘车体系 y 轴角速度 |
| 28–31 | 4 | `float` | `gyro_z` | rad/s | 底盘车体系 z 轴角速度 |
| 32–35 | 4 | `float` | `acc_x` | m/s² | 底盘车体系 x 轴加速度 |
| 36–39 | 4 | `float` | `acc_y` | m/s² | 底盘车体系 y 轴加速度 |
| 40–43 | 4 | `float` | `acc_z` | m/s² | 底盘车体系 z 轴加速度 |
| 44–45 | 2 | `uint16_t` | `flags` | — | 有效性/故障状态位 |

**编译时校验**：

```cpp
static_assert(sizeof(ReceivePacket) == 46, "ReceivePacket size mismatch with protocol");
```

### 2.4 下行帧格式（导航→MCU）

当前下行协议较简单，仅用于通信测试：

| 字段 | 长度 | 值 | 说明 |
|------|------|----|------|
| header | 1 | `0xFF` | 帧头 |
| test | 1 | 0~255 | 测试数据 |
| checksum | 1 | `0xFE` | 校验尾 |

```cpp
struct SendPacket {
    uint8_t header = 0xFF;
    uint8_t test;      // 0~255，通信测试
    uint8_t checksum = 0xFE;
} __attribute__((packed));
```

### 2.5 发送频率

| 方向 | 频率 | 说明 |
|------|------|------|
| 上行（MCU→导航） | 200 Hz | IMU + 轮速数据，每 5ms 一帧 |
| 下行（导航→MCU） | 按需 | 当前仅测试用途 |

---

## 三、CRC16-CCITT 校验算法

### 3.1 算法参数

| 参数 | 值 |
|------|----|
| 多项式 | `0x1021` |
| 初始值 | `0xFFFF` |
| 处理方式 | MSB-first（非反射） |
| 输入/输出 | 不反转 |

### 3.2 算法实现

```cpp
uint16_t Calc_CRC16_CCITT(const uint8_t * data, size_t len)
{
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int j = 0; j < 8; j++) {
      if (crc & 0x8000U) {
        crc = static_cast<uint16_t>((crc << 1) ^ 0x1021U);
      } else {
        crc = static_cast<uint16_t>(crc << 1);
      }
    }
  }
  return crc;
}
```

### 3.3 Python 等效实现（用于调试/验证）

```python
def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc
```

### 3.4 重要注意事项

项目中存在**两种**不同的 CRC16 算法，**不可混用**：

| 函数 | 算法 | 用途 | 备注 |
|------|------|------|------|
| `Calc_CRC16_CCITT()` | MSB-first, poly=0x1021 | 帧校验（当前使用） | 与电控协议一致 |
| `Get_CRC16_Check_Sum()` | LSB-first, 查表法 | 遗留代码 | **不可用于帧校验** |

---

## 四、串口驱动节点实现（rm_serial_driver）

### 4.1 节点架构

```cpp
class RMSerialDriver : public rclcpp::Node {
public:
  explicit RMSerialDriver(const rclcpp::NodeOptions & options);
  ~RMSerialDriver() override;

private:
  void receiveData();                                    // 接收线程主循环
  void sendData(rm_interfaces::msg::Target::SharedPtr); // 下行发送
  void reopenPort();                                     // 串口重连
  void getParams();                                      // 读取串口参数

  // 串口对象
  std::unique_ptr<IoContext> owned_ctx_;
  std::string device_name_;
  std::unique_ptr<SerialPortConfig> device_config_;
  std::unique_ptr<SerialDriver> serial_driver_;

  // ROS 话题
  rclcpp::Subscription<rm_interfaces::msg::Target>::SharedPtr target_sub_;
  rclcpp::Publisher<rm_interfaces::msg::Gimbal>::SharedPtr gimbal_pub_;

  std::thread receive_thread_;   // 独立接收线程
};
```

### 4.2 初始化流程

```
RMSerialDriver 构造函数
│
├── 1. getParams()
│   ├── 读取 device_name（默认 ""）
│   ├── 读取 baud_rate（默认 0）
│   ├── 读取 flow_control（none/hardware/software）
│   ├── 读取 parity（none/odd/even）
│   └── 读取 stop_bits（1/1.5/2）
│
├── 2. 创建 Publisher/Subscriber（必须在接收线程之前）
│   ├── target_sub_ ← /tracker/target
│   └── gimbal_pub_ → /tracker/gimbal
│
├── 3. 初始化串口
│   ├── serial_driver_->init_port(device_name_, *device_config_)
│   └── serial_driver_->port()->open()
│
└── 4. 启动接收线程
    └── receive_thread_ = std::thread(&RMSerialDriver::receiveData, this)
```

> **关键设计**：Publisher/Subscriber 必须在 `receive_thread_` 启动前初始化，否则线程调用 `publish()` 时指针为空，导致 SIGSEGV。

### 4.3 接收线程主循环（receiveData）

```
receiveData() ─── 阻塞式逐字节读取
│
└── while (rclcpp::ok())
    │
    ├── 1. 找帧头第一字节 A5
    │   └── serial_driver_->port()->receive(hdr)
    │   └── if (hdr[0] != 0xA5) continue
    │
    ├── 2. 确认帧头第二字节 5A
    │   └── serial_driver_->port()->receive(hdr)
    │   └── if (hdr[0] != 0x5A) continue
    │
    ├── 3. 读取剩余 50 字节
    │   └── serial_driver_->port()->receive(rest)  // 阻塞直到全部到达
    │
    ├── 4. 校验 len 字段
    │   └── if (rest[1] != 46) → WARN + continue
    │
    ├── 5. CRC16-CCITT 校验
    │   ├── 拼完整帧: frame = [A5, 5A, rest[0..49]]
    │   ├── rx_crc  = frame[50] | (frame[51] << 8)  // 小端读取
    │   ├── calc_crc = Calc_CRC16_CCITT(frame, 50)
    │   └── if (rx_crc != calc_crc) → WARN + continue
    │
    ├── 6. 解析 payload
    │   └── memcpy(&packet, frame + 4, 46)
    │
    ├── 7. 填充并发布 Gimbal 消息
    │   ├── gimbal_msg.header.stamp = now()
    │   ├── gimbal_msg.t_ms = packet.t_ms
    │   ├── gimbal_msg.angular_velocity = {gyro_x, gyro_y, gyro_z}
    │   ├── gimbal_msg.linear_acceleration = {acc_x, acc_y, acc_z}
    │   ├── gimbal_msg.wheel_velocity = {w_fl, w_fr, w_rl, w_rr}  // 借用 Quaternion
    │   └── gimbal_pub_->publish(gimbal_msg)
    │
    └── 异常处理
        ├── catch → RCLCPP_ERROR_THROTTLE
        └── reopenPort()  // 自动重连
```

### 4.4 解帧策略详解

串口数据是**字节流**，不保证一次 `read()` 恰好读出一帧，因此采用**逐字节扫描+固定长度读取**策略：

1. **逐字节扫描帧头**：每次读取 1 字节，直到匹配 `0xA5`，再读 1 字节确认 `0x5A`
2. **固定长度读取**：帧头确认后，`receive(rest)` 阻塞读取剩余 50 字节（`port->receive()` 保证读满才返回）
3. **多重校验**：
   - `len` 字段必须等于 46
   - CRC16-CCITT 必须匹配
4. **失败处理**：任何校验失败都丢弃当前同步点，重新扫描帧头

> **已知局限**：当第二字节不是 `0x5A` 但恰好是下一帧的 `0xA5` 时，会丢失一个同步机会。由于串口 API 不支持 `unread`，无法回退。CRC 兜底保证了数据正确性。

### 4.5 下行数据发送（sendData）

```
sendData(Target::SharedPtr msg)
│
├── 1. 构造 SendPacket
│   └── packet.test = msg->test
│
├── 2. 序列化为字节流
│   └── toVector(packet) → std::vector<uint8_t>
│
└── 3. 通过串口发送
    └── serial_driver_->port()->send(data)
```

### 4.6 串口重连机制（reopenPort）

```
reopenPort()
│
├── 关闭串口
│   └── if (port->is_open()) port->close()
│
├── 重新打开
│   └── port->open()
│
└── 失败重试
    ├── RCLCPP_ERROR
    ├── sleep_for(1s)
    └── reopenPort()  // 递归重试，直到 rclcpp::ok() 为 false
```

### 4.7 析构流程

```
~RMSerialDriver()
│
├── 等待接收线程结束
│   └── if (receive_thread_.joinable()) receive_thread_.join()
│
├── 关闭串口
│   └── if (port->is_open()) port->close()
│
└── 等待 IO 上下文退出
    └── owned_ctx_->waitForExit()
```

---

## 五、自定义消息接口（rm_interfaces）

### 5.1 Gimbal.msg（上行）

```
# 时间戳
std_msgs/Header header

# MCU 采样时间戳（单位 ms，用于 ESKF dt 计算，与 header.stamp 独立）
uint32 t_ms

# IMU数据
geometry_msgs/Vector3 angular_velocity    # 角速度 (rad/s)
geometry_msgs/Vector3 linear_acceleration # 线加速度 (m/s²)

# 轮速
geometry_msgs/Quaternion wheel_velocity # 轮分速度 (rad/s)

# 轮子标签对应：
#         ^     ^
# 左前轮   1 > < 2   右前轮
#
#         ^     ^
# 左后轮 < 3     4 > 右后轮

# 测试消息
string log
```

**字段映射说明**：

| Gimbal.msg 字段 | ReceivePacket 字段 | 说明 |
|------------------|-------------------|------|
| `header.stamp` | — | ROS 系统时间戳，由 `rm_serial_driver` 填充 |
| `t_ms` | `t_ms` | MCU 采样时间戳，直接透传 |
| `angular_velocity.x` | `gyro_x` | 车体系 x 轴角速度 |
| `angular_velocity.y` | `gyro_y` | 车体系 y 轴角速度 |
| `angular_velocity.z` | `gyro_z` | 车体系 z 轴角速度 |
| `linear_acceleration.x` | `acc_x` | 车体系 x 轴加速度 |
| `linear_acceleration.y` | `acc_y` | 车体系 y 轴加速度 |
| `linear_acceleration.z` | `acc_z` | 车体系 z 轴加速度 |
| `wheel_velocity.x` | `w_fl` | 前左轮角速度 |
| `wheel_velocity.y` | `w_fr` | 前右轮角速度 |
| `wheel_velocity.z` | `w_rl` | 后左轮角速度 |
| `wheel_velocity.w` | `w_rr` | 后右轮角速度 |

> **设计取舍**：`wheel_velocity` 借用 `geometry_msgs/Quaternion` 的四个 float 字段（x/y/z/w）来传递四个轮速，避免自定义 4 维浮点消息类型。映射关系为 x=fl, y=fr, z=rl, w=rr。

### 5.2 Target.msg（下行）

```
# 通讯测试
uint8 test
```

### 5.3 依赖关系

```
rm_interfaces 依赖：
  - geometry_msgs（Vector3, Quaternion）
  - std_msgs（Header）

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Target.msg"
  "msg/Gimbal.msg"
  DEPENDENCIES geometry_msgs std_msgs
)
```

---

## 六、数据处理节点（data_handle）

### 6.1 订阅与发布

```cpp
// 订阅上行电控数据
gimbal_sub_ = create_subscription<rm_interfaces::msg::Gimbal>(
    "/tracker/gimbal", rclcpp::SensorDataQoS(), gimbalCallBack);

// 发布下行指令
target_pub_ = create_publisher<rm_interfaces::msg::Target>("/tracker/target", 10);

// 发布导航数据
odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
path_pub_ = create_publisher<nav_msgs::msg::Path>("/path", 10);

// TF 广播
tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
```

### 6.2 回调处理流程

```
gimbalCallBack(msg)
│
├── 1. 初始化检查
│   └── 首帧仅记录 t_ms，跳过 ESKF
│
├── 2. 计算 dt
│   ├── dt = (msg->t_ms - last_t_ms_) / 1000.0
│   └── 异常过滤: dt ≤ 0 或 dt > 0.1 → 丢弃
│
├── 3. ESKF 滤波
│   ├── predict(msg, dt)         — 名义状态积分 + 协方差预测
│   ├── observeWheel(msg)        — 轮速观测更新
│   ├── observeZeroTilt()        — 零倾斜约束更新
│   └── injectAndReset()         — 误差注入 + 重置
│
├── 4. 发布里程计
│   └── publishOdometry(msg->header.stamp)
│       ├── TF: odom → base_footprint
│       ├── /odom 消息
│       └── /path 轨迹（最多 5000 点）
│
└── 5. 更新时间戳
    └── last_t_ms_ = msg->t_ms
```

### 6.3 时间戳双轨制

| 时间戳 | 来源 | 用途 |
|--------|------|------|
| `msg->header.stamp` | ROS `get_clock()->now()` | TF/odom 等 ROS 生态组件，保证时间单调递增 |
| `msg->t_ms` | MCU 采样时刻 | ESKF 计算 `dt`，精确反映真实采样间隔 |

使用 MCU 时间戳计算 `dt` 的优势：
- 避免 USB 传输延迟抖动（ROS 时间戳包含传输延迟）
- `uint32_t` 减法自然处理溢出回绕（约 49 天绕一圈）
- 更精确的积分时间步长

---

## 七、参数配置

### 7.1 串口参数（serial_driver.yaml）

```yaml
/**:
  ros__parameters:
    device_name: /tmp/ttyACM0    # 串口设备路径
    baud_rate: 115200             # 波特率
    flow_control: none            # 流控: none / hardware / software
    parity: none                  # 校验: none / odd / even
    stop_bits: "1"                # 停止位: "1" / "1.5" / "2"
```

| 参数 | 类型 | 可选值 | 说明 |
|------|------|--------|------|
| `device_name` | `string` | 任意设备路径 | 实际使用 `/dev/ttyACM0`，配置中写 `/tmp/ttyACM0` 可能需要按实际修改 |
| `baud_rate` | `int` | 任意正值 | USB CDC 不受此限制，但必须非零 |
| `flow_control` | `string` | `none`, `hardware`, `software` | 通常选 `none` |
| `parity` | `string` | `none`, `odd`, `even` | 通常选 `none` |
| `stop_bits` | `string` | `"1"`, `"1.5"`, `"2"` | 通常选 `"1"` |

### 7.2 参数读取流程

```cpp
void getParams() {
  device_name_ = declare_parameter<std::string>("device_name", "");
  baud_rate    = declare_parameter<int>("baud_rate", 0);
  // flow_control, parity, stop_bits 通过字符串匹配转为枚举
  device_config_ = make_unique<SerialPortConfig>(baud_rate, fc, pt, sb);
}
```

无效参数会抛出 `rclcpp::ParameterTypeException` 或 `std::invalid_argument`，节点启动失败。

---

## 八、启动与部署

### 8.1 启动文件链

```
full.launch.py
├── serial_driver.launch.py      ← 启动 rm_serial_driver_node
└── full_navigation.launch.py    ← 启动导航栈
    ├── static_transform_publisher (map→odom)
    ├── Nav2 导航组件 (controller/planner/bt_navigator/...)
    ├── robot_state_publisher
    ├── joint_state_publisher
    ├── rviz2
    └── data_handle_node         ← ESKF 数据处理
```

### 8.2 启动命令

```bash
# 安装依赖
sudo apt install ros-humble-serial-driver

# 编译
colcon build --packages-select rm_interfaces rm_serial_driver my_nav2_robot

# 运行
source install/setup.bash
ros2 launch my_nav2_robot full.launch.py
```

### 8.3 USB 设备权限

默认情况下普通用户无法访问 `/dev/ttyACM*`，需要配置 udev 规则：

```bash
# 临时方案
sudo chmod 666 /dev/ttyACM0

# 永久方案：创建 udev 规则
sudo tee /etc/udev/rules.d/99-ttyACM.rules << 'EOF'
KERNEL=="ttyACM[0-9]*", MODE="0666"
EOF
sudo udevadm control --reload-rules
sudo udevadm trigger
```

---

## 九、异常处理机制

### 9.1 串口通信层

| 异常场景 | 检测方式 | 处理策略 | 代码位置 |
|---------|---------|---------|---------|
| 串口打开失败 | `std::exception` | 抛出异常，节点终止 | `rm_serial_driver.cpp:48-58` |
| 帧头不匹配 | 逐字节比对 `0xA5`/`0x5A` | 跳过，继续扫描 | `rm_serial_driver.cpp:94-103` |
| len 字段错误 | `rest[1] != 46` | `RCLCPP_WARN_THROTTLE` + 丢弃 | `rm_serial_driver.cpp:110-114` |
| CRC 校验失败 | `rx_crc != calc_crc` | `RCLCPP_WARN_THROTTLE` + 丢弃 | `rm_serial_driver.cpp:130-134` |
| 接收异常 | `std::exception` | `RCLCPP_ERROR_THROTTLE` + `reopenPort()` | `rm_serial_driver.cpp:165-169` |
| 串口重连失败 | `std::exception` | sleep 1s + 递归重试 | `rm_serial_driver.cpp:287-293` |
| 发送异常 | `std::exception` | `RCLCPP_ERROR` + `reopenPort()` | `rm_serial_driver.cpp:189-192` |
| 参数无效 | `ParameterTypeException` | 抛出异常，节点终止 | `rm_serial_driver.cpp:210-272` |

### 9.2 数据处理层

| 异常场景 | 检测方式 | 处理策略 | 代码位置 |
|---------|---------|---------|---------|
| 首帧数据 | `!initialized` | 仅记录 `t_ms`，跳过 ESKF | `data_handle.cpp:44-48` |
| dt 异常 | `dt ≤ 0 \|\| dt > 0.1` | 更新 `last_t_ms_`，跳过本轮 ESKF | `data_handle.cpp:53-57` |
| 路径点过多 | `path_.poses.size() > 5000` | 清空路径 | `data_handle.cpp:326` |

### 9.3 日志节流

关键位置使用 `RCLCPP_WARN_THROTTLE` / `RCLCPP_ERROR_THROTTLE` 避免高频异常场景下日志洪泛：

- CRC 不匹配：200ms 节流
- 接收异常：20ms 节流

---

## 十、坐标系与数据约定

### 10.1 车体系定义

```
底盘车体系 (base_footprint)：
  +x = 底盘前方
  +y = 底盘左方
  +z = 底盘上方
```

### 10.2 IMU 数据方向

所有 IMU 数据已经由电控转换到**底盘车体系**：

```
gyro_x > 0：绕底盘 x 轴正向转动（俯仰）
gyro_y > 0：绕底盘 y 轴正向转动（横滚）
gyro_z > 0：底盘逆时针自转（偏航）

acc_x > 0：沿底盘前方加速
acc_y > 0：沿底盘左方加速
acc_z > 0：沿底盘上方加速（静止时 ≈ +9.8 m/s²，含重力反作用力）
```

### 10.3 轮速方向约定

```
         ^     ^
左前轮   1 > < 2   右前轮
  (ESC ID 2)  (ESC ID 1)

         ^     ^
左后轮 < 3     4 > 右后轮
  (ESC ID 3)  (ESC ID 4)
```

运动学符号关系：

```
vx 前进：    w_fl +, w_fr +, w_rl +, w_rr +
vy 左移：    w_fl -, w_fr +, w_rl +, w_rr -
wz 逆时针：  w_fl -, w_fr +, w_rl -, w_rr +
```

麦轮运动学反算：

$$v_x = \frac{r}{4}(w_{fl} + w_{fr} + w_{rl} + w_{rr})$$

$$v_y = \frac{r}{4}(-w_{fl} + w_{fr} + w_{rl} - w_{rr})$$

$$\omega_z = \frac{r}{4(l_x + l_y)}(-w_{fl} + w_{fr} - w_{rl} + w_{rr})$$

机械参数：
- 轮半径 $r = 0.0815$ m
- $l_x + l_y \approx 0.3005$ m（需与电控确认布局是否为正方形）

---

## 十一、调试与验证

### 11.1 Python 串口读取脚本

可用于独立验证串口数据，无需启动 ROS：

```python
import struct, serial

SOF = b"\xA5\x5A"
PAYLOAD_LEN = 46
FRAME_LEN = 52

def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1) & 0xFFFF
    return crc

def read_frames(port: str):
    ser = serial.Serial(port, baudrate=115200, timeout=0.2)
    buf = bytearray()
    while True:
        buf.extend(ser.read(ser.in_waiting or 1))
        while len(buf) >= FRAME_LEN:
            sof = buf.find(SOF)
            if sof < 0: del buf[:-1]; break
            if sof > 0: del buf[:sof]
            if len(buf) < FRAME_LEN: break
            frame = bytes(buf[:FRAME_LEN])
            if frame[3] != PAYLOAD_LEN: del buf[0]; continue
            if crc16_ccitt(frame[:-2]) != struct.unpack("<H", frame[-2:])[0]:
                del buf[0]; continue
            del buf[:FRAME_LEN]
            values = struct.unpack("<I10fH", frame[4:50])
            yield dict(zip(
                ["t_ms","w_fl","w_fr","w_rl","w_rr",
                 "gyro_x","gyro_y","gyro_z","acc_x","acc_y","acc_z","flags"],
                values))
```

### 11.2 ROS 2 调试命令

```bash
# 查看上行数据
ros2 topic echo /tracker/gimbal

# 查看下行指令
ros2 topic echo /tracker/target

# 检查话题频率
ros2 topic hz /tracker/gimbal

# 查看 TF 树
ros2 run tf2_tools view_frames

# 查看串口节点参数
ros2 param list /serial_driver
ros2 param get /serial_driver device_name
```

### 11.3 常见问题排查

| 问题 | 可能原因 | 排查方法 |
|------|---------|---------|
| 节点启动即崩溃 | 串口设备不存在/无权限 | `ls -la /dev/ttyACM*`，检查 udev 规则 |
| 无数据输出 | 帧头/CRC 不匹配 | 降低日志节流阈值，查看 WARN 日志 |
| 数据频率异常 | USB 断连 | 检查 `reopenPort()` 日志，确认物理连接 |
| dt 计算为 0 或负值 | MCU 时间戳回绕 | 正常现象，`uint32_t` 减法自然处理 |
| ESKF 发散 | 观测噪声参数不当 | 调整 `Q_`/`R_`/`R_tilt_` 参数 |
| 里程计漂移 | 陀螺仪零偏未收敛 | 确保开机静置数秒让 ESKF 收敛 |

---

## 十二、源文件索引

| 文件 | 路径 | 职责 |
|------|------|------|
| `rm_serial_driver.cpp` | `src/serial_driver/src/` | 串口驱动主逻辑：初始化、接收、发送、重连 |
| `rm_serial_driver.hpp` | `src/serial_driver/include/rm_serial_driver/` | 类声明 |
| `packet.hpp` | `src/serial_driver/include/rm_serial_driver/` | 帧格式定义、ReceivePacket/SendPacket 结构体 |
| `crc.cpp` | `src/serial_driver/src/` | CRC16-CCITT 及遗留 CRC16 实现 |
| `crc.hpp` | `src/serial_driver/include/rm_serial_driver/` | CRC 函数声明 |
| `serial_driver.yaml` | `src/serial_driver/config/` | 串口参数配置 |
| `serial_driver.launch.py` | `src/serial_driver/launch/` | 串口驱动启动文件 |
| `Gimbal.msg` | `src/rm_interfaces/msg/` | 上行消息定义 |
| `Target.msg` | `src/rm_interfaces/msg/` | 下行消息定义 |
| `data_handle.cpp` | `src/my_nav2_robot/src/` | ESKF 里程计实现 |
| `data_handle.hpp` | `src/my_nav2_robot/include/my_nav2_robot/` | 数据处理类声明 |
| `full.launch.py` | `src/my_nav2_robot/launch/` | 总启动入口 |
| `full_navigation.launch.py` | `src/my_nav2_robot/launch/` | 导航栈启动 |
| `imu_coordinate.md` | 项目根目录 | 电控数据坐标约定与 Python 示例 |
