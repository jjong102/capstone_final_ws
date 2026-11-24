#include <mutex>

#include "nav2_navfn_planner/navfn_planner.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "third_impact/patrol_path.hpp"

namespace third_impact
{
class PatrolPlanner : public nav2_navfn_planner::NavfnPlanner
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override
  {
    NavfnPlanner::configure(parent, name, tf, costmap);
    costmap_ros_ = costmap;
    clock_ = parent.lock()->get_clock();
    cached_.clear();
  }

  void cleanup() override
  {
    std::lock_guard<std::mutex> guard(planner_mutex_);
    cached_.clear();
    clock_.reset();
    costmap_ros_.reset();
    NavfnPlanner::cleanup();
  }

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override
  {
    namespace pp = patrol_path;
    std::lock_guard<std::mutex> guard(planner_mutex_);
    const pp::Point a{start.pose.position.x,start.pose.position.y};
    const pp::Point b{goal.pose.position.x,goal.pose.position.y};
    const double yaw = tf2::getYaw(start.pose.orientation);
    if (start.header.frame_id != goal.header.frame_id ||
      start.header.frame_id != costmap_ros_->getGlobalFrameID() ||
      !std::isfinite(pp::distance(a,b)) || !std::isfinite(yaw))
    {
      return nav_msgs::msg::Path();
    }

    const auto footprint = costmap_ros_->getRobotFootprint();
    double length = 0., width = 0.;
    for (const auto & p : footprint) {
      length = std::max(length,std::abs(p.x));
      width = std::max(width,std::abs(p.y));
    }
    if (length <= 0. || width <= 0.) {
      cached_.clear();
      return NavfnPlanner::createPlan(start,goal);
    }

    auto snapshot = copyCostmap();
    if (snapshot.getResolution() <= 0.) {return nav_msgs::msg::Path();}
    const pp::Clearance check(snapshot,length,width);
    const auto publishable = [&](const pp::Route & route) {
        nav_msgs::msg::Path path;
        path.header = start.header;
        path.header.stamp = clock_->now();
        for (size_t i = 1; i < route.size(); ++i) {
          const auto count = std::max(1,static_cast<int>(std::ceil(pp::distance(route[i-1],route[i])/.05)));
          tf2::Quaternion orientation;
          orientation.setRPY(0.,0.,pp::heading(route[i-1],route[i]));
          for (int j = 0; j < count; ++j) {
            auto pose = start;
            const auto p = pp::interpolate(route[i-1],route[i],static_cast<double>(j)/count);
            pose.header = path.header;
            pose.pose.position.x = p.x;
            pose.pose.position.y = p.y;
            pose.pose.orientation = tf2::toMsg(orientation);
            path.poses.push_back(pose);
          }
        }
        auto endpoint = goal;
        endpoint.header = path.header;
        path.poses.push_back(endpoint);
        cached_goal_ = b;
        cached_ = route;
        return path;
      };

    // A replan may keep the already checked suffix of the same mission path.
    // Validate from the current robot pose on a fresh costmap before reusing it.
    if (!cached_.empty() && pp::distance(cached_goal_,b) < 1e-6) {
      auto tail = pp::remaining(cached_,a);
      if (check.route(tail,yaw)) {return publishable(tail);}
    }
    cached_.clear();
    if (check.segment(a,b,yaw)) {return publishable({a,b});}

    auto original = NavfnPlanner::createPlan(start,goal);
    if (original.poses.size() < 2) {return original;}
    const auto & endpoint = original.poses.back().pose.position;
    if (pp::distance({endpoint.x,endpoint.y},b) > .05) {return original;}
    pp::Route raw{a};
    for (const auto & pose : original.poses) {
      raw.push_back({pose.pose.position.x,pose.pose.position.y});
    }
    raw.back() = b;
    // Navfn may update the start cell; use the unmodified snapshot for shortcuts.
    const auto simpler = pp::simplify(raw,yaw,check);
    return simpler.empty() ? original : publishable(simpler);
  }

private:
  nav2_costmap_2d::Costmap2D copyCostmap()
  {
    auto * map = costmap_ros_->getCostmap();
    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
    return *map;
  }

  std::mutex planner_mutex_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Clock::SharedPtr clock_;
  patrol_path::Point cached_goal_{};
  patrol_path::Route cached_;
};
}  // namespace third_impact

PLUGINLIB_EXPORT_CLASS(third_impact::PatrolPlanner, nav2_core::GlobalPlanner)
