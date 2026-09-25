#include "example_behavior.hh"
#include "pluginlib/class_list_macros.hpp"

namespace rmcs::navigation::behaviors {

ExampleBehavior::ExampleBehavior()
    : TimedBehavior{} {}

ExampleBehavior::~ExampleBehavior() = default;

auto ExampleBehavior::onRun(const std::shared_ptr<const nav2_msgs::action::Wait::Goal> command)
    -> nav2_behaviors::ResultStatus {
    RCLCPP_INFO(logger_, "Example behavior run, wait %d s", command->time.sec);
    return nav2_behaviors::ResultStatus{nav2_behaviors::Status::SUCCEEDED};
}

auto ExampleBehavior::onCycleUpdate() -> nav2_behaviors::ResultStatus {
    return nav2_behaviors::ResultStatus{nav2_behaviors::Status::SUCCEEDED};
}

} // namespace rmcs::navigation::behaviors

PLUGINLIB_EXPORT_CLASS(rmcs::navigation::behaviors::ExampleBehavior, nav2_core::Behavior)
