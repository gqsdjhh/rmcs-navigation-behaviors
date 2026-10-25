#pragma once

#include <atomic>

#include <nav2_costmap_2d/layer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

namespace rmcs::navigation::behaviors {

/// 临时忽略局部地图的代价地图层。
///
/// 该层应放在局部代价地图插件列表的最后，确保清掉 static_layer 与
/// inflation_layer 在本周期已写入的代价。收到 true 时清空主代价地图，使
/// 局部控制器不再使用 local_map 的障碍；收到 false 后不再干预，
/// static_layer 会随下一帧地图恢复障碍。
class LocalMapIgnoreLayer final : public nav2_costmap_2d::Layer {
public:
    LocalMapIgnoreLayer() = default;
    ~LocalMapIgnoreLayer() override = default;

    auto onInitialize() -> void override;
    auto reset() -> void override;
    auto isClearable() -> bool override;
    auto updateBounds(
        double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
        double* max_x, double* max_y) -> void override;
    auto updateCosts(
        nav2_costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
        -> void override;

private:
    std::atomic<bool> ignore_local_map_{false};
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ignore_subscription_;
};

} // namespace rmcs::navigation::behaviors
