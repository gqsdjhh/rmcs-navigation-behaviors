#pragma once
#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_msgs/action/wait.hpp"

#include <memory>

namespace rmcs::navigation::behaviors {

/// 自定义行为插件示例：复用 nav2_msgs::action::Wait 的 action 类型，
/// 因此可直接被现有 Wait BT 节点驱动，无需额外编写 BT 节点。
class ExampleBehavior final : public nav2_behaviors::TimedBehavior<nav2_msgs::action::Wait> {
public:
    ExampleBehavior();
    ~ExampleBehavior() override;

    auto onRun(const std::shared_ptr<const nav2_msgs::action::Wait::Goal> command)
        -> nav2_behaviors::ResultStatus override;
    auto onCycleUpdate() -> nav2_behaviors::ResultStatus override;

    auto getResourceInfo() -> nav2_core::CostmapInfoType override {
        return nav2_core::CostmapInfoType::LOCAL;
    }
};

} // namespace rmcs::navigation::behaviors
