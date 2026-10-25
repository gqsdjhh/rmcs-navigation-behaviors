#include "local_map_ignore_layer.hh"

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>

#include <nav2_costmap_2d/costmap_2d.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace rmcs::navigation::behaviors {

auto LocalMapIgnoreLayer::onInitialize() -> void {
    auto node = node_.lock();
    if (!node)
        throw std::runtime_error{"LocalMapIgnoreLayer requires a valid lifecycle node"};

    declareParameter(
        "enable_topic", rclcpp::ParameterValue{std::string{"/rmcs_navigation/ignore_local_map"}});
    const auto topic = node->get_parameter(getFullName("enable_topic")).as_string();

    auto options = rclcpp::SubscriptionOptions{};
    options.callback_group = callback_group_;

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    ignore_subscription_ = node->create_subscription<std_msgs::msg::Bool>(
        topic, qos,
        [this](const std_msgs::msg::Bool::SharedPtr message) {
            const auto ignore = message->data;
            if (ignore_local_map_.exchange(ignore) == ignore)
                return;

            if (ignore)
                RCLCPP_INFO(logger_, "local map is now ignored by local costmap");
            else
                RCLCPP_INFO(logger_, "local map is no longer ignored by local costmap");
        },
        options);
}

auto LocalMapIgnoreLayer::reset() -> void {}

auto LocalMapIgnoreLayer::isClearable() -> bool { return false; }

auto LocalMapIgnoreLayer::updateBounds(
    double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double* min_x, double* min_y,
    double* max_x, double* max_y) -> void {
    if (!ignore_local_map_.load())
        return;

    auto* costmap = layered_costmap_->getCostmap();
    const auto origin_x = costmap->getOriginX();
    const auto origin_y = costmap->getOriginY();
    const auto size_x = static_cast<double>(costmap->getSizeInCellsX()) * costmap->getResolution();
    const auto size_y = static_cast<double>(costmap->getSizeInCellsY()) * costmap->getResolution();

    *min_x = std::min(*min_x, origin_x);
    *min_y = std::min(*min_y, origin_y);
    *max_x = std::max(*max_x, origin_x + size_x);
    *max_y = std::max(*max_y, origin_y + size_y);
}

auto LocalMapIgnoreLayer::updateCosts(
    nav2_costmap_2d::Costmap2D& master_grid, int /*min_i*/, int /*min_j*/, int /*max_i*/,
    int /*max_j*/) -> void {
    if (!ignore_local_map_.load())
        return;

    master_grid.resetMap(0, 0, master_grid.getSizeInCellsX(), master_grid.getSizeInCellsY());
}

} // namespace rmcs::navigation::behaviors

PLUGINLIB_EXPORT_CLASS(rmcs::navigation::behaviors::LocalMapIgnoreLayer, nav2_costmap_2d::Layer)
