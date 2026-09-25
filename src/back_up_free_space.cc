#include "back_up_free_space.hh"

#include "pluginlib/class_list_macros.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <numbers>
#include <stdexcept>

namespace rmcs::navigation::behaviors {

auto BackUpFreeSpace::onConfigure() -> void {
    auto node = node_.lock();
    if (!node) {
        throw std::runtime_error{"Failed to lock node"};
    }

    const auto prefix = behavior_name_ + ".";
    nav2_util::declare_parameter_if_not_declared(
        node, prefix + "service_name",
        rclcpp::ParameterValue{std::string{"/global_costmap/get_costmap"}});
    nav2_util::declare_parameter_if_not_declared(
        node, prefix + "sample_radius", rclcpp::ParameterValue{3.0});
    nav2_util::declare_parameter_if_not_declared(
        node, prefix + "sample_directions", rclcpp::ParameterValue{18});
    nav2_util::declare_parameter_if_not_declared(
        node, prefix + "obstacle_threshold", rclcpp::ParameterValue{150.0});
    nav2_util::declare_parameter_if_not_declared(
        node, prefix + "stop_cost", rclcpp::ParameterValue{150.0});
    nav2_util::declare_parameter_if_not_declared(
        node, prefix + "refresh_interval", rclcpp::ParameterValue{0.5});
    nav2_util::declare_parameter_if_not_declared(
        node, prefix + "visualize", rclcpp::ParameterValue{false});

    node->get_parameter(prefix + "service_name", service_name_);
    node->get_parameter(prefix + "sample_radius", sample_radius_);
    node->get_parameter(prefix + "sample_directions", sample_directions_);
    node->get_parameter(prefix + "obstacle_threshold", obstacle_threshold_);
    node->get_parameter(prefix + "stop_cost", stop_cost_);
    node->get_parameter(prefix + "refresh_interval", refresh_interval_);
    node->get_parameter(prefix + "visualize", visualize_);

    if (sample_directions_ <= 0) {
        sample_directions_ = 1;
    }
    if (sample_radius_ <= 0.0) {
        sample_radius_ = 1.0;
    }
    if (refresh_interval_ < 0.0) {
        refresh_interval_ = 0.0;
    }

    costmap_client_ = node->create_client<nav2_msgs::srv::GetCostmap>(service_name_);

    if (visualize_) {
        marker_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
            "back_up_free_space_markers", 1);
    }

    RCLCPP_INFO(
        logger_, "BackUpFreeSpace configured: service=%s, radius=%.2f, directions=%d",
        service_name_.c_str(), sample_radius_, sample_directions_);
}

auto BackUpFreeSpace::onCleanup() -> void {
    costmap_client_.reset();
    marker_pub_.reset();

    std::lock_guard<std::mutex> lock{costmap_mutex_};
    latest_costmap_.reset();
    has_costmap_ = false;
    costmap_request_in_flight_ = false;
    heading_.reset();
    started_ = false;
}

auto BackUpFreeSpace::onRun(const std::shared_ptr<const Action::Goal> command)
    -> nav2_behaviors::ResultStatus {
    if (command->target.y != 0.0 || command->target.z != 0.0) {
        RCLCPP_WARN(logger_, "BackUpFreeSpace only supports motion in the XY plane.");
        return {nav2_behaviors::Status::FAILED, Action::Result::INVALID_INPUT};
    }

    command_x_ = command->target.x;
    command_speed_ = command->speed;
    command_time_allowance_ = command->time_allowance;
    end_time_ = clock_->now() + command_time_allowance_;

    heading_.reset();
    started_ = false;
    last_costmap_request_ = rclcpp::Time{0, 0, RCL_ROS_TIME};

    RCLCPP_INFO(
        logger_, "BackUpFreeSpace started: distance=%.2f m, speed=%.2f m/s", command_x_,
        command_speed_);

    return {nav2_behaviors::Status::SUCCEEDED, Action::Result::NONE};
}

auto BackUpFreeSpace::onCycleUpdate() -> nav2_behaviors::ResultStatus {
    const auto time_remaining = end_time_ - clock_->now();
    if (time_remaining.seconds() < 0.0 && command_time_allowance_.seconds() > 0.0) {
        stopRobot();
        RCLCPP_WARN(logger_, "Exceeded time allowance before reaching the BackUp goal.");
        return {nav2_behaviors::Status::FAILED, Action::Result::TIMEOUT};
    }

    requestCostmap();
    const auto costmap = snapshotCostmap();
    if (!costmap) {
        RCLCPP_WARN_THROTTLE(
            logger_, *clock_, 1000, "No costmap received yet, waiting...");
        return {nav2_behaviors::Status::RUNNING, Action::Result::NONE};
    }

    auto current_pose = geometry_msgs::msg::PoseStamped{};
    if (!nav2_util::getCurrentPose(
            current_pose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_)) {
        RCLCPP_ERROR(logger_, "Current robot pose is not available.");
        return {nav2_behaviors::Status::FAILED, Action::Result::TF_ERROR};
    }

    const auto x = current_pose.pose.position.x;
    const auto y = current_pose.pose.position.y;

    if (!started_) {
        start_pose_.x = x;
        start_pose_.y = y;
        heading_ = findBestDirection(*costmap, x, y);
        if (!heading_) {
            stopRobot();
            RCLCPP_WARN(logger_, "BackUpFreeSpace failed to find a safe direction.");
            return {nav2_behaviors::Status::FAILED, Action::Result::UNKNOWN};
        }
        started_ = true;
        RCLCPP_INFO(
            logger_, "BackUpFreeSpace selected direction %.2f rad", *heading_);
    }

    const auto distance = std::hypot(x - start_pose_.x, y - start_pose_.y);
    feedback_->distance_traveled = static_cast<float>(distance);
    action_server_->publish_feedback(feedback_);

    const auto cell_cost = queryCostAt(*costmap, x, y);
    if (distance >= std::fabs(command_x_) || (cell_cost >= 0 && cell_cost <= stop_cost_)) {
        stopRobot();
        return {nav2_behaviors::Status::SUCCEEDED, Action::Result::NONE};
    }

    const auto yaw = tf2::getYaw(current_pose.pose.orientation);
    const auto base_heading = *heading_ - yaw;

    auto cmd_vel = geometry_msgs::msg::TwistStamped{};
    cmd_vel.header.stamp = clock_->now();
    cmd_vel.header.frame_id = robot_base_frame_;
    cmd_vel.twist.linear.x = std::cos(base_heading) * std::fabs(command_speed_);
    cmd_vel.twist.linear.y = std::sin(base_heading) * std::fabs(command_speed_);
    cmd_vel.twist.angular.z = 0.0;
    vel_pub_->publish(std::make_unique<geometry_msgs::msg::TwistStamped>(cmd_vel));

    if (visualize_) {
        visualize(x, y, *heading_);
    }

    return {nav2_behaviors::Status::RUNNING, Action::Result::NONE};
}

auto BackUpFreeSpace::requestCostmap() -> void {
    if (!costmap_client_ || !costmap_client_->service_is_ready()) {
        RCLCPP_WARN_THROTTLE(
            logger_, *clock_, 2000, "GetCostmap service is not ready.");
        return;
    }

    {
        std::lock_guard<std::mutex> lock{costmap_mutex_};
        if (costmap_request_in_flight_) {
            return;
        }
        if (has_costmap_ &&
            (clock_->now() - last_costmap_request_) <
                rclcpp::Duration::from_seconds(refresh_interval_)) {
            return;
        }
        costmap_request_in_flight_ = true;
        last_costmap_request_ = clock_->now();
    }

    auto request = std::make_shared<nav2_msgs::srv::GetCostmap::Request>();

    costmap_client_->async_send_request(
        request, [this](ServiceResponseFuture future) {
            try {
                auto response = future.get();
                auto costmap = std::make_shared<Costmap>(std::move(response->map));

                std::lock_guard<std::mutex> lock{costmap_mutex_};
                latest_costmap_ = std::move(costmap);
                has_costmap_ = true;
                costmap_request_in_flight_ = false;
            } catch (const std::exception & e) {
                std::lock_guard<std::mutex> lock{costmap_mutex_};
                costmap_request_in_flight_ = false;
                RCLCPP_ERROR(logger_, "GetCostmap failed: %s", e.what());
            }
        });
}

auto BackUpFreeSpace::snapshotCostmap() -> std::shared_ptr<const Costmap> {
    std::lock_guard<std::mutex> lock{costmap_mutex_};
    if (!has_costmap_) {
        return nullptr;
    }
    return latest_costmap_;
}

auto BackUpFreeSpace::queryCostAt(const Costmap & costmap, double x, double y) const -> int {
    const auto & meta = costmap.metadata;
    if (meta.resolution <= 0.0F || meta.size_x == 0 || meta.size_y == 0) {
        return 256;
    }

    const auto grid_x = static_cast<long>((x - meta.origin.position.x) / meta.resolution);
    const auto grid_y = static_cast<long>((y - meta.origin.position.y) / meta.resolution);
    if (grid_x < 0 || grid_y < 0 || grid_x >= static_cast<long>(meta.size_x) ||
        grid_y >= static_cast<long>(meta.size_y)) {
        return 256;
    }

    const auto index =
        static_cast<size_t>(grid_y) * static_cast<size_t>(meta.size_x) + static_cast<size_t>(grid_x);
    if (index >= costmap.data.size()) {
        return 256;
    }

    return static_cast<int>(costmap.data[index]);
}

auto BackUpFreeSpace::sampleDirectionCosts(
    const Costmap & costmap, double x, double y, double direction) const -> std::vector<float> {
    std::vector<float> costs;
    const auto resolution = costmap.metadata.resolution;
    if (resolution <= 0.0F) {
        return costs;
    }

    const auto step_count = std::max(1, static_cast<int>(sample_radius_ / resolution));
    costs.reserve(static_cast<size_t>(step_count));

    for (int step = 1; step <= step_count; ++step) {
        const auto distance = step * resolution;
        const auto sample_x = x + distance * std::cos(direction);
        const auto sample_y = y + distance * std::sin(direction);
        const auto cost = queryCostAt(costmap, sample_x, sample_y);
        costs.push_back(cost > 255 ? 255.0F : static_cast<float>(cost));
    }

    return costs;
}

auto BackUpFreeSpace::countLeadingObstacles(const std::vector<float> & costs) const -> int {
    int count = 0;
    for (const auto cost : costs) {
        if (cost < obstacle_threshold_) {
            break;
        }
        ++count;
    }
    return count;
}

auto BackUpFreeSpace::findBestDirection(const Costmap & costmap, double x, double y) const
    -> std::optional<double> {
    auto best_direction = std::optional<double>{};
    auto min_leading_obstacles = std::numeric_limits<int>::max();
    auto min_avg_cost = std::numeric_limits<float>::max();

    for (int i = 0; i < sample_directions_; ++i) {
        const auto direction =
            2.0 * std::numbers::pi * static_cast<double>(i) / sample_directions_;
        const auto costs = sampleDirectionCosts(costmap, x, y, direction);
        if (costs.empty()) {
            continue;
        }

        const auto leading_obstacles = countLeadingObstacles(costs);
        const auto avg_cost =
            std::accumulate(costs.begin(), costs.end(), 0.0F) / static_cast<float>(costs.size());

        if (!best_direction || leading_obstacles < min_leading_obstacles ||
            (leading_obstacles == min_leading_obstacles && avg_cost < min_avg_cost)) {
            best_direction = direction;
            min_leading_obstacles = leading_obstacles;
            min_avg_cost = avg_cost;
        }
    }

    if (best_direction) {
        RCLCPP_INFO(
            logger_, "BackUpFreeSpace best direction %.2f rad, leading obstacles=%d, avg cost=%.2f",
            *best_direction, min_leading_obstacles, min_avg_cost);
    }

    return best_direction;
}

auto BackUpFreeSpace::visualize(double x, double y, double direction) -> void {
    if (!marker_pub_) {
        return;
    }

    auto arrow = visualization_msgs::msg::Marker{};
    arrow.header.frame_id = global_frame_;
    arrow.header.stamp = clock_->now();
    arrow.ns = "back_up_free_space";
    arrow.id = 0;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;
    arrow.scale.x = 0.05;
    arrow.scale.y = 0.1;
    arrow.scale.z = 0.1;
    arrow.color.r = 0.0F;
    arrow.color.g = 1.0F;
    arrow.color.b = 0.0F;
    arrow.color.a = 1.0F;

    auto start = geometry_msgs::msg::Point{};
    start.x = x;
    start.y = y;
    start.z = 0.0;

    auto end = geometry_msgs::msg::Point{};
    end.x = x + sample_radius_ * std::cos(direction);
    end.y = y + sample_radius_ * std::sin(direction);
    end.z = 0.0;

    arrow.points = {start, end};

    auto markers = visualization_msgs::msg::MarkerArray{};
    markers.markers.push_back(arrow);
    marker_pub_->publish(markers);
}

} // namespace rmcs::navigation::behaviors

PLUGINLIB_EXPORT_CLASS(rmcs::navigation::behaviors::BackUpFreeSpace, nav2_core::Behavior)
