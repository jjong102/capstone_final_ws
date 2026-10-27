#include <iostream>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <pluginlib/class_loader.hpp>
#include <nav2_core/goal_checker.hpp>
int main(int argc, char **argv) {
 rclcpp::init(argc,argv);
 try {
  auto node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("controller_server");
  pluginlib::ClassLoader<nav2_core::GoalChecker> loader("nav2_core","nav2_core::GoalChecker");
  auto patrol=loader.createSharedInstance("nav2_controller::SimpleGoalChecker");
  patrol->initialize(node,"patrol_goal_checker",nullptr);
  geometry_msgs::msg::Pose goal, query;
  goal.orientation.w=1.; query.orientation.z=1.;  // 180 degree heading error
  geometry_msgs::msg::Twist velocity; velocity.linear.x=.4; velocity.angular.z=.5;
  query.position.x=.049;
  if(!patrol->isGoalReached(query,goal,velocity)) throw std::runtime_error("4.9cm with opposite heading/motion must succeed");
  query.position.x=.051;
  if(patrol->isGoalReached(query,goal,velocity)) throw std::runtime_error("5.1cm must not succeed, including after prior success");
  query.position.x=.04; query.position.y=.04;
  if(patrol->isGoalReached(query,goal,velocity)) throw std::runtime_error("Euclidean distance outside 5cm must fail");
  auto normal=loader.createSharedInstance("nav2_controller::SimpleGoalChecker");
  normal->initialize(node,"goal_checker",nullptr);
  query.position.x=.01; query.position.y=0.;
  if(normal->isGoalReached(query,goal,velocity)) throw std::runtime_error("Return checker heading policy changed");
  std::cout<<"PASS: installed Nav2 checker accepts patrol at 4.9cm while moving with 180deg error; rejects 5.1cm; return heading policy retained\n";
 } catch(const std::exception &e) {std::cerr<<e.what()<<"\n"; rclcpp::shutdown(); return 1;}
 rclcpp::shutdown(); return 0;
}
