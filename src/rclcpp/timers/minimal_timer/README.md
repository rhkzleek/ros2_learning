# minimal\_timer — ROS 2 rclcpp 最小定时器示例

该包展示了两种极简 **Timer** 用法：使用 **lambda 回调** 与 **成员函数回调**。二者功能一致（定期在控制台打印问候语），仅体现 C++ 写法差异，便于对比与上手。

```
.
├── CMakeLists.txt
├── include
│   └── minimal_timer/
├── package.xml
└── src
    ├── lambda.cpp                # create_wall_timer + lambda 回调
    └── member_function.cpp       # create_wall_timer + 成员函数回调
```

---

## 1. 快速开始

### 1.1 依赖

* ROS 2（建议 Humble 及以上）
* `rclcpp`

### 1.2 编译

```bash
colcon build --packages-select minimal_timer
source install/setup.bash
```

> 可执行名请以 `CMakeLists.txt` 实际安装项或 `ros2 pkg executables minimal_timer` 输出为准。

### 1.3 运行与验证

```bash
# 方式一：lambda 写法
eros2 run minimal_timer timer_lambda

# 方式二：成员函数写法
eros2 run minimal_timer timer_member_function
```

你应能在终端看到固定周期（如 500ms/1s）的“Hello”打印。

---

## 2. 关键概念与最佳实践

### 2.1 `create_wall_timer` vs `create_timer`

* **`create_wall_timer(period, cb)`**：使用 **Steady/Wall Clock**，与 ROS 时间无关；即使启用仿真时间（`use_sim_time=true`）也按壁钟节拍触发，适合**节点内部维护任务**（心跳、后台清理等）。
* **`create_timer(clock, period, cb)`**：使用传入的 **Clock**；如传入 `this->get_clock()` 则受 `use_sim_time` 影响。当启用仿真时间且 `/clock` 存在时，**随仿真时间流逝触发**，适合**与仿真时钟对齐**的逻辑。

> 新手易错：启用了 `use_sim_time` 但没有 `/clock` 发布者时，基于 ROS Clock 的定时器将**不前进**；而 `create_wall_timer` 不受影响。

### 2.2 回调组（Callback Group）

* 对实时性敏感的周期任务，建议：

  * 使用 `MutuallyExclusiveCallbackGroup` 将关键 Timer 与其它 I/O 隔离；
  * 或改用 `ReentrantCallbackGroup` 配合多线程执行器提升并发。

### 2.3 周期与延迟

* 请避免在回调中做耗时阻塞；否则会导致**定时漂移**与**积压**。
* 需要固定频率且计算耗时显著时，考虑**计算-发布解耦**：将计算移到工作线程/协程，Timer 只做触发/调度。

### 2.4 动态调整周期

* rclcpp 的 Timer 不支持直接改周期；如需变更：

  1. `cancel()` 旧定时器；
  2. 以新周期 `create_*_timer()` 重新创建。
* 仅需“从当前时间重新计时”可调用 `reset()`。

---

## 3. 最小示例模板

### 3.1 lambda 写法

```cpp
#include <rclcpp/rclcpp.hpp>

class TimerNode : public rclcpp::Node {
public:
  TimerNode() : Node("minimal_timer_lambda"), n_(0) {
    using namespace std::chrono_literals;
    timer_ = create_wall_timer(500ms, [this]{
      RCLCPP_INFO(get_logger(), "Hello (lambda) #%zu", n_++);
    });
  }
private:
  rclcpp::TimerBase::SharedPtr timer_;
  size_t n_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TimerNode>());
  rclcpp::shutdown();
  return 0;
}
```

### 3.2 成员函数写法

```cpp
#include <rclcpp/rclcpp.hpp>

class TimerNode : public rclcpp::Node {
public:
  TimerNode() : Node("minimal_timer_member"), n_(0) {
    using namespace std::chrono_literals;
    timer_ = create_wall_timer(1s, std::bind(&TimerNode::onTick, this));
  }
private:
  void onTick() {
    RCLCPP_INFO(get_logger(), "Hello (member) #%zu", n_++);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  size_t n_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TimerNode>());
  rclcpp::shutdown();
  return 0;
}
```

---

## 4. 故障排查（FAQ）

* **“启用 use\_sim\_time 后 Timer 不走动”**：

  * 若你使用 `create_timer(get_clock(), ...)`，需要确保有 `/clock` 发布者（Gazebo、rosbag2 播放等）。
  * 或改用 `create_wall_timer`，与仿真时钟解耦。
* **回调执行不准时/漂移**：

  * 回调里存在耗时操作或 I/O 阻塞；考虑移出或异步化。
  * 单线程执行器下回调互相抢占；使用回调组 + `MultiThreadedExecutor`。
* **周期过短 CPU 飙高**：

  * 提高周期（减小频率），或在回调内做速率限制、批处理。

---

## 5. 建议

* 需要与仿真时间精准对齐的周期逻辑，使用 `create_timer(get_clock(), ...)`。
* 节点心跳、后台清理、watchdog 等与仿真无关的定时器，统一使用 `create_wall_timer`，更稳健。
* 复杂系统建议引入**诊断**：在回调中每 N 次打印统计（平均耗时、抖动等），便于监控。

---

**License**：与本仓库一致（见根目录 LICENSE）。
**维护者**：请在 `package.xml` 的 `<maintainer>` 中填写你的信息。
