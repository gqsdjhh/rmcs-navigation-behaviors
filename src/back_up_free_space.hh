#pragma once

#include "nav2_behaviors/plugins/drive_on_heading.hpp"
#include "nav2_msgs/action/back_up.hpp"
#include "nav2_msgs/srv/get_costmap.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace rmcs::navigation::behaviors {

/// BackUp 行为：采样代价地图各方向，选择最自由方向后按全向方式移动。
/// 复用 nav2_msgs::action::BackUp，可直接被现有 <BackUp> BT 节点驱动。
class BackUpFreeSpace final : public nav2_behaviors::DriveOnHeading<nav2_msgs::action::BackUp> {
public:
    using Action = nav2_msgs::action::BackUp;
    using Costmap = nav2_msgs::msg::Costmap;

    BackUpFreeSpace() = default;
    ~BackUpFreeSpace() override = default;

    auto onConfigure() -> void override;
    auto onCleanup() -> void override;

    auto onRun(const std::shared_ptr<const Action::Goal> command)
        -> nav2_behaviors::ResultStatus override;
    auto onCycleUpdate() -> nav2_behaviors::ResultStatus override;

    auto getResourceInfo() -> nav2_core::CostmapInfoType override {
        return nav2_core::CostmapInfoType::NONE;
    }

private:
    using ServiceResponseFuture = rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedFuture;

    auto requestCostmap() -> void;
    auto snapshotCostmap() -> std::shared_ptr<const Costmap>;
    auto queryCostAt(const Costmap & costmap, double x, double y) const -> int;
    auto sampleDirectionCosts(const Costmap & costmap, double x, double y, double direction) const
        -> std::vector<float>;
    auto countLeadingObstacles(const std::vector<float> & costs) const -> int;
    auto findBestDirection(const Costmap & costmap, double x, double y) const
        -> std::optional<double>;
    auto visualize(double x, double y, double direction) -> void;

    rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedPtr costmap_client_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    std::string service_name_;
    double sample_radius_{3.0};
    int sample_directions_{18};
    double obstacle_threshold_{150.0};
    double stop_cost_{150.0};
    double refresh_interval_{0.5};
    bool visualize_{false};

    std::shared_ptr<const Costmap> latest_costmap_;
    bool has_costmap_{false};
    bool costmap_request_in_flight_{false};
    std::mutex costmap_mutex_;
    rclcpp::Time last_costmap_request_{0, 0, RCL_ROS_TIME};

    geometry_msgs::msg::Pose2D start_pose_{};
    std::optional<double> heading_;
    bool started_{false};
};

} // namespace rmcs::navigation::behaviors
