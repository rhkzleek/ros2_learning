# minimal\_publisher — ROS 2 rclcpp 最小发布者示例

该包演示了多种“极简但地道”的 C++ 发布者写法：用 `rclcpp::TimerBase` 定时发布、以类成员函数或 lambda 作为回调、使用 Type Adapter 适配消息类型、等待消息全部确认（wait\_for\_all\_acked），以及一个“不推荐”的非可组合风格示例，帮助你快速对比与上手。

```
.
├── CMakeLists.txt
├── include
│   └── minimal_publisher/          # 头文件（若有公共声明）
├── package.xml
└── src
    ├── lambda.cpp                                  # 定时器 + lambda 回调
    ├── member_function.cpp                         # 定时器 + 成员函数回调
    ├── member_function_with_type_adapter.cpp       # 成员函数 + TypeAdapter 适配
    ├── member_function_with_unique_network_flow_endpoints.cpp  # 网络流端点唯一化
    ├── member_function_with_wait_for_all_acked.cpp # 等待所有订阅端 ACK
    └── not_composable.cpp                          # 非可组合风格（不推荐）
```

---

## 1. 快速开始

### 1.1 依赖

* ROS 2（建议 Humble 及以上）
* `rclcpp`, `std_msgs`

### 1.2 编译

在你的工作区根目录执行（以包名 `minimal_publisher` 为例）：

```bash
colcon build --packages-select minimal_publisher
source install/setup.bash
```

> 无法确定可执行名？用：
>
> ```bash
> ros2 pkg executables minimal_publisher
> ```
>
> 列出该包中所有可运行目标（名字以 CMakeLists.txt 为准）。

### 1.3 运行与验证

以 **lambda** 示例为例（可执行名可能为 `talker_timer_lambda`，以实际构建结果为准）：

```bash
ros2 run minimal_publisher talker_timer_lambda
# 新终端：
ros2 topic echo /chatter
ros2 topic hz /chatter
```

---

## 2. 示例速查表

| 源文件                                                      | 关注点                             | 适用场景                    | 备注          |
| -------------------------------------------------------- | ------------------------------- | ----------------------- | ----------- |
| `lambda.cpp`                                             | `create_wall_timer` + lambda 回调 | 最少样板代码，快速起步             | 简洁直观        |
| `member_function.cpp`                                    | 成员函数回调                          | 正规工程化风格                 | 便于扩展与单测     |
| `member_function_with_type_adapter.cpp`                  | Type Adapter                    | 当内部类型与外部消息类型不同，需要零拷贝/转换 | 见 §3.2      |
| `member_function_with_unique_network_flow_endpoints.cpp` | 网络流端点唯一化                        | 排障/监控需求，需要区分网络流         | 见 §3.3      |
| `member_function_with_wait_for_all_acked.cpp`            | 等待订阅端 ACK                       | 关机/关键消息投递前确保“已送达”       | 见 §3.1      |
| `not_composable.cpp`                                     | 不可组合风格（直接 `rclcpp::Node node;`） | Demo/临时脚本               | 不推荐，难以组件化复用 |

---

## 3. 关键点讲解

### 3.1 等待所有 ACK（wait\_for\_all\_acked）

**用途**：在退出或关键阶段确保“所有订阅端都确认接收”。
**典型做法**（伪代码）：

```cpp
auto pub = this->create_publisher<std_msgs::msg::String>(
  "/chatter", rclcpp::QoS(10).reliable());
pub->publish(msg);
// 在关闭前等待至多 1s，直到所有已连接订阅者 ACK：
pub->wait_for_all_acked(std::chrono::seconds(1));
```

**注意**：

* 仅对 **可靠(reliable)** QoS 有意义；`best_effort` 模式没有 ACK 语义。
* 等待会阻塞当前线程，请在可控点调用（如关闭前或关键事务后）。

### 3.2 Type Adapter（类型适配）

当你的业务内部用的是 **自定义/高效** 类型，但对外需发布标准 ROS 消息时，可用 `rclcpp::TypeAdapter` 适配，避免到处手写转换代码。

**思路**：
适配器定义内部类型 `T` 与消息类型 `Msg` 的相互转换；发布/订阅端只看到其中一侧类型，另一侧由适配器透明完成。

**示意**：

```cpp
struct StringAdapter : rclcpp::TypeAdapter<std::string, std_msgs::msg::String> {
  static void convert_to_ros_message(const std::string & in, std_msgs::msg::String & out) {
    out.data = in;
  }
  static void convert_to_custom(const std_msgs::msg::String & in, std::string & out) {
    out = in.data;
  }
};

// 使用时：
auto pub = this->create_publisher<
  rclcpp::TypeAdapter<std::string, std_msgs::msg::String>
>("/chatter", 10);
pub->publish(std::string("hello with adapter"));
```

### 3.3 网络流端点唯一化（Unique Network Flow Endpoints）

**目的**：为发布者请求“**唯一的网络流端点**”，便于底层 RMW/网络层按端区分与统计（监控/调试友好）。
**要点**：

* 通过 RMW 层的“唯一网络流端点需求”枚举来表达“严格需要 / 可选 / 不需要 / 系统默认”。
* 不是所有 RMW 都实现；未实现时若设为“严格需要”会报错，设为“可选”则会优雅降级。

**适用**：网络抓包、流量分析、QoS 监控等对“每个发布端独立可辨识”有需求的场景。

---

## 4. 推荐用法与风格

* **优先**使用“**Node 子类 + 成员函数回调**”写法（`member_function.cpp`），更利于可维护性与单元测试。
* **快速验证**场景可用 `lambda.cpp`，但随着工程增长建议迁移到成员函数形式。
* **不要**在项目中长期保留 `not_composable.cpp` 的非可组合风格；后续往组件化迁移成本会变高。
* 需要强语义投递保障时，配合 **reliable QoS + wait\_for\_all\_acked**。
* 内外类型不一致或追求零拷贝/统一转换入口时使用 **Type Adapter**。

---

## 5. 运行示例（参考命令）

> 具体可执行名请以 `ros2 pkg executables minimal_publisher` 输出或 CMakeLists 为准。

```bash
# 1) Lambda 回调
ros2 run minimal_publisher talker_timer_lambda

# 2) 成员函数回调
ros2 run minimal_publisher talker_timer_member_function

# 3) TypeAdapter 示例
ros2 run minimal_publisher talker_timer_member_function_with_type_adapter

# 4) 唯一网络流端点
ros2 run minimal_publisher talker_timer_member_function_with_unique_network_flow_endpoints

# 5) 等待所有 ACK
ros2 run minimal_publisher talker_timer_member_function_with_wait_for_all_acked

# 6) 非可组合（不推荐）
ros2 run minimal_publisher talker_without_subclass
```

配套观察：

```bash
# 观察数据
ros2 topic echo /chatter
ros2 topic hz /chatter
ros2 topic info /chatter -v
```

---

## 6. 一个超简发布者模板

```cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class MinimalTalker : public rclcpp::Node {
public:
  MinimalTalker() : Node("minimal_talker"), count_(0) {
    pub_ = create_publisher<std_msgs::msg::String>(
      "/chatter", rclcpp::QoS(10).reliable());
    timer_ = create_wall_timer(std::chrono::milliseconds(200), [this]{
      std_msgs::msg::String msg;
      msg.data = "Hello #" + std::to_string(count_++);
      pub_->publish(msg);
    });
  }
  ~MinimalTalker() override {
    // 关键阶段确保 ACK（可选）
    pub_->wait_for_all_acked(std::chrono::milliseconds(500));
  }
private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalTalker>());
  rclcpp::shutdown();
  return 0;
}
```

---

## 7. 数据流简图

```mermaid
flowchart LR
    T[Timer 200ms] -->|触发| CB[回调函数<br/>构造消息]
    CB --> PUB[Publisher<br/>/chatter]
    PUB --> ROS[ROS 2 中间件]
    ROS --> SUBS[订阅者们]
```

---

## 8. 常见问题（FAQ）

* **看不到可执行？**
  检查 `CMakeLists.txt` 是否 `add_executable()` 并 `ament_target_dependencies()`、`install(TARGETS ...)`。重新 `colcon build` 后 `source install/setup.bash`。

* **`wait_for_all_acked` 无效？**
  确认 QoS 使用 `reliable()`；Best Effort 没有 ACK 语义。还要确保确实有活跃订阅者连接。

* **Type Adapter 编译错误？**
  检查模板参数顺序与 `convert_to_ros_message/convert_to_custom` 的签名是否匹配；确认包含头与命名空间。

---

## 9. 相关建议

* 想继续学习**可组合节点**（Component/Composition），可对比 ROS 2 的 composition 示例包，把节点导出为组件并在 `component_container_mt` 中动态装载。
* 若要在多主机或复杂网络下调试 QoS 和网络流，建议配合 `ros2 doctor hello`、`ros2 topic info -v`、RMW 层日志一起使用。

---

**License**：与本仓库整体一致（见仓库根目录 LICENSE）。
**维护者**：请在 `package.xml` 中的 `<maintainer>` 标签维护
