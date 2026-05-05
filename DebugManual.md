# 联调技术手册

> 本文档涵盖远程联调场景下的串口数据转发与 ROS2 数据可视化方案。  
> 典型场景：STM32 等嵌入式设备连接在远程电脑 A 上，需要在本地电脑 B 上运行 ROS2 节点读取设备数据。

---

## 一、串口数据远程转发（socat）

### 1.1 原理

使用 `socat` 将物理串口设备的数据流转发为 TCP/UDP 网络流，接收端创建虚拟串口设备，从而实现跨主机的串口透传。

```
[STM32] ──USB──> [电脑A /dev/ttyACM0] ──socat TCP──> 网络 ──socat──> [电脑B /tmp/ttyV0] ──ROS2读取
```

### 1.2 发送端（设备连接方）

将物理串口 `/dev/ttyACM0` 转发为 TCP 监听端口 8888：

```bash
socat -v /dev/ttyACM0,raw,echo=0,ispeed=115200,ospeed=115200 TCP-LISTEN:8888,reuseaddr,tcp-nodelay
```

**参数说明：**

| 参数 | 含义 |
|------|------|
| `-v` | 详细输出模式，用于调试 |
| `raw` | 原始数据模式，不做终端处理 |
| `echo=0` | 禁止回显 |
| `ispeed=115200` / `ospeed=115200` | 输入/输出波特率（与 MCU 端一致即可，可省略） |
| `TCP-LISTEN:8888` | 监听 TCP 端口 8888 |
| `reuseaddr` | 允许端口复用，断开后可立即重新绑定 |
| `tcp-nodelay` | 禁用 Nagle 算法，即收即发，**强烈建议开启**以降低延迟 |

> **注意**：默认 TCP 使用 Nagle 算法（攒一批数据再发送），会导致数据传输延迟和包间粘连。对于 200Hz 的串口数据流，务必在两端都加上 `tcp-nodelay`。

### 1.3 接收端（ROS2 运行方）

在本地电脑创建虚拟串口 `/tmp/ttyV0`，连接远程 TCP 流：

```bash
socat -d -d pty,link=/tmp/ttyV0,raw,echo=0 TCP:192.168.1.100:8888,tcp-nodelay
```

**参数说明：**

| 参数 | 含义 |
|------|------|
| `-d -d` | 两级调试输出，便于排查连接问题 |
| `pty,link=/tmp/ttyV0` | 创建伪终端并软链接为 `/tmp/ttyV0` |
| `TCP:IP:8888` | 连接发送端 IP 和端口（替换为实际 IP） |
| `tcp-nodelay` | 必须与发送端一致，否则可能出现数据包乱序或粘包 |

### 1.4 网络前提

- 两台电脑需处于**同一局域网**（校园网、路由器等均可）
- 若不在同一局域网，需使用内网穿透工具（如 Radmin LAN、Tailscale 等）建立虚拟局域网
- 确保防火墙放行对应端口（如 8888）

### 1.5 UDP 模式（可选）

若网络丢包可接受且追求更低延迟，可使用 UDP：

**发送端：**
```bash
socat -v /dev/ttyACM0,raw,echo=0 UDP-LISTEN:8888,reuseaddr
```

**接收端：**
```bash
socat -d -d pty,link=/tmp/ttyV0,raw,echo=0 UDP:192.168.1.100:8888
```

> UDP 无需 `tcp-nodelay`（本身无 Nagle 算法），但不保证数据有序到达，不建议用于串口协议帧解析场景。

### 1.6 验证与排障

**验证虚拟串口是否工作：**
```bash
# 查看数据流（十六进制）
xxd /tmp/ttyV0
# 或直接读取原始字节
cat /tmp/ttyV0
```

**常见问题：**

| 现象 | 原因 | 解决 |
|------|------|------|
| 连接后无数据 | 发送端设备未插入或路径错误 | `ls /dev/ttyACM*` 确认设备存在；检查波特率 |
| 数据延迟明显 | 未开启 `tcp-nodelay` | 两端均加上 `tcp-nodelay` |
| 数据包粘包/拆包 | Nagle 算法或缓冲区策略 | 开启 `tcp-nodelay`；确认 `raw` 模式 |
| 连接被拒绝 | 防火墙或端口占用 | `sudo ufw allow 8888`；`ss -tlnp \| grep 8888` 检查端口 |
| Permission denied | 串口设备权限不足 | `sudo chmod 666 /dev/ttyACM0` 或添加 udev 规则 |

### 1.7 与本项目串口驱动的配合

本项目的 `rm_serial_driver` 默认读取 `/tmp/ttyACM0`（见 `serial_driver.yaml`）。使用 socat 转发时，接收端创建的虚拟设备路径需与之匹配：

```bash
# 方式一：直接创建为 /tmp/ttyACM0（与配置一致，无需改配置）
socat -d -d pty,link=/tmp/ttyACM0,raw,echo=0 TCP:IP:8888,tcp-nodelay

# 方式二：创建为 /tmp/ttyV0，需修改 serial_driver.yaml 中的 device_name
```

> **建议**：直接用方式一，避免修改配置文件。本地实车调试时，udev 规则可将设备 symlink 到 `/tmp/ttyACM0`，保持一致。

---

## 二、ROS2 数据可视化（Foxglove Studio）

### 2.1 原理

Foxglove Studio 是一个开源的机器人数据可视化工具，通过 WebSocket 连接 ROS2 的 `foxglove_bridge` 节点，远程实时查看话题数据、绘制图表、显示 TF 树等。

```
[ROS2 节点] ──话题──> [foxglove_bridge] ──WebSocket──> [Foxglove Studio]
```

### 2.2 ROS2 端启动桥接

在运行 ROS2 节点的电脑上启动 `foxglove_bridge`：

```bash
# 安装（如未安装）
sudo apt install ros-jazzy-foxglove-bridge

# 启动
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

默认监听端口为 `8765`（WebSocket）。

### 2.3 Foxglove Studio 连接

1. 下载并安装 [Foxglove Studio](https://foxglove.dev/)
2. 登录注册后进入主界面
3. 选择 **Open connection** → **Foxglove WebSocket**
4. 输入连接地址：
   ```
   ws://<ubuntu-ip>:8765
   ```
   其中 `<ubuntu-ip>` 为运行 `foxglove_bridge` 的电脑 IP

### 2.4 网络前提

- 同一局域网：直接使用内网 IP 连接
- 跨网络：使用 Tailscale 等工具建立虚拟局域网，用 Tailscale 分配的 IP 替换

### 2.5 常用可视化面板

| 面板 | 用途 | 对应话题 |
|------|------|---------|
| 3D | 显示 TF 树、机器人模型、路径 | `/tf`, `/tf_static`, `/path` |
| Raw Messages | 查看话题原始消息 | 任意话题 |
| Plot | 绘制数值曲线（如速度、角速度） | `/odom`, `/tracker/gimbal` |
| Map | 显示占据栅格地图 | `/map` |
| Teleop | 键盘遥控发送速度指令 | `/cmd_vel` |

### 2.6 常见问题

| 现象 | 原因 | 解决 |
|------|------|------|
| 连接超时 | 防火墙或端口未开放 | `sudo ufw allow 8765`；检查 bridge 是否正常启动 |
| 看不到话题 | bridge 启动时话题尚未发布 | 重启 bridge 或等话题发布后刷新 |
| 3D 面板无模型 | URDF 未加载或 robot_state_publisher 未启动 | 确保 `robot_state_publisher` 节点在运行 |

---

## 三、完整联调流程

### 3.1 远程联调（设备在另一台电脑）

```bash
# === 电脑 A（设备端）===
# 1. 插入 STM32 设备，确认识别
ls /dev/ttyACM*

# 2. 添加串口读取权限
sudo chmod 666 /dev/ttyACM0

# 3. 启动 socat 转发
socat -v /dev/ttyACM0,raw,echo=0,ispeed=115200,ospeed=115200 TCP-LISTEN:8888,reuseaddr,tcp-nodelay
```

```bash
# === 电脑 B（ROS2 端）===
# 1. 启动 socat 接收（创建虚拟串口）
socat -d -d pty,link=/tmp/ttyACM0,raw,echo=0 TCP:<电脑A的IP>:8888,tcp-nodelay

# 2. 另一终端：source 并构建工作空间
source /opt/ros/jazzy/setup.bash
cd ~/ace_ass && colcon build
source install/setup.bash

# 3. 启动完整系统
ros2 launch my_nav2_robot full.launch.py

# 4.（可选）启动 foxglove_bridge 供远程可视化
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

### 3.2 本地联调（设备直连本机）

```bash
# 1. 插入 STM32 设备
ls /dev/ttyACM*

# 2. 添加权限或创建软链接（与 serial_driver.yaml 中 device_name 匹配）
sudo chmod 666 /dev/ttyACM0
sudo ln -sf /dev/ttyACM0 /tmp/ttyACM0

# 3. 启动系统
source /opt/ros/jazzy/setup.bash
source ~/ace_ass/install/setup.bash
ros2 launch my_nav2_robot full.launch.py
```

---

## 四、iPad 远程编写代码

### 4.1 使用 code-server（网页 VSCode）

code-server 在 Ubuntu 上运行 VS Code 的服务端，iPad 通过浏览器访问即可获得完整的 VS Code 编辑体验。

#### 4.1.1 安装 code-server

```bash
# 官方一键安装脚本
curl -fsSL https://code-server.dev/install.sh | sh
```

#### 4.1.2 配置连接密码与端口

编辑配置文件：

```bash
nano ~/.config/code-server/config.yaml
```

默认内容如下，按需修改：

```yaml
bind-addr: 0.0.0.0:8080   # 监听所有网卡的 8080 端口（默认仅 127.0.0.1，需改为 0.0.0.0）
auth: password              # 认证方式：password
password: your_password     # 连接密码，改为自己的强密码
cert: false                 # 无 HTTPS 证书（局域网内可关闭）
```

> **关键**：`bind-addr` 必须改为 `0.0.0.0:8080`，否则仅本机可访问，iPad 无法连接。

#### 4.1.3 启动与状态管理

```bash
# 启动并设置开机自启
sudo systemctl enable --now code-server@$USER

# 查看运行状态
sudo systemctl status code-server@$USER

# 重启（修改配置后需要重启生效）
sudo systemctl restart code-server@$USER

# 停止
sudo systemctl stop code-server@$USER

# 查看日志（排查问题）
journalctl -u code-server@$USER -f
```

#### 4.1.4 防火墙放行

```bash
sudo ufw allow 8080
```

#### 4.1.5 iPad Safari 连接

1. 确保 iPad 与 Ubuntu 处于**同一局域网**
2. 在 Ubuntu 上查看 IP：

```bash
ip addr show | grep "inet " | grep -v 127.0.0.1
```

3. iPad 打开 **Safari**，地址栏输入：

```
http://<ubuntu-ip>:8080
```

4. 输入配置文件中设置的密码，即可进入完整的 VS Code 界面，开始编写代码

> **提示**：Safari 对 WebSocket 支持良好，code-server 的终端、扩展等功能均可正常使用。建议将页面"添加到主屏幕"以获得类 App 体验。

---

### 4.2 使用 Koder（SFTP 快速编辑）

Koder 是 iPad 上的轻量级代码编辑器，支持 SFTP 协议直接读写远程服务器文件，适合快速编辑单个文件而无需启动完整 VS Code。

#### 4.2.1 安装 Koder

在 App Store 搜索 **Koder** 并安装。

#### 4.2.2 新建 SFTP 连接

1. 打开 Koder → 点击 **+** → 选择 **SFTP/SSH**
2. 填写连接信息：

| 字段 | 填写内容 |
|------|---------|
| Name | 自定义名称，如 `Ubuntu Dev` |
| Host | Ubuntu 的 IP 地址 |
| Port | `22`（默认 SSH 端口） |
| Username | Ubuntu 用户名 |
| Password | 对应用户密码（或使用 SSH 密钥） |
| URL | 一般前端调试用，可空 |
| Path | 远程目录，如 `/home/<user>/ace_ass/src/`，可空，但建议填写|

3. 点击 **Connect** 建立连接（或直接点击文件夹图标进入目录）

#### 4.2.3 编辑代码

- 连接成功后浏览远程目录（如 `/home/<user>/ace_ass/src/`）
- 点击文件即可编辑，保存时自动通过 SFTP 写回 Ubuntu
- 支持语法高亮、多标签页编辑

> **适用场景**：快速修改配置文件、单文件编辑、查看日志等轻量操作。如需终端交互或复杂项目管理，请使用 code-server。

---

## 五、参考

- [socat 官方手册](https://linux.die.net/man/1/socat)
- [Foxglove Studio 官方文档](https://foxglove.dev/docs)
- [ros-jazzy-foxglove-bridge](https://index.ros.org/p/foxglove_bridge/)
