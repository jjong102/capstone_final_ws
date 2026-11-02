#include <iostream>
#include <random>
#include <set>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <dwb_core/trajectory_critic.hpp>
#include <pluginlib/class_loader.hpp>
#include <std_msgs/msg/string.hpp>
#include <QJsonDocument>
#include <QJsonObject>

bool allowed(const dwb_core::TrajectoryCritic::Ptr &c,double x,double y,double w) {
 dwb_msgs::msg::Trajectory2D t;t.velocity.x=x;t.velocity.y=y;t.velocity.theta=w;
 try {c->scoreTrajectory(t);return true;}catch(const std::exception &) {return false;}
}
int main(int argc,char **argv) {
 rclcpp::init(argc,argv);
 try {
  auto node=std::make_shared<nav2_util::LifecycleNode>("oscillation_test");
  pluginlib::ClassLoader<dwb_core::TrajectoryCritic> loader("dwb_core","dwb_core::TrajectoryCritic");
  auto original=loader.createSharedInstance("dwb_critics::OscillationCritic");
  auto instrumented=loader.createSharedInstance("third_impact::LoggedOscillationCritic");
  original->initialize(node,"Oscillation","Stock",nullptr);
  instrumented->initialize(node,"Oscillation","Patrol",nullptr);
  std::mt19937 rng(4217);std::uniform_real_distribution<double> step(-.03,.03);
  geometry_msgs::msg::Pose2D p,goal;nav_2d_msgs::msg::Twist2D cmd;nav_2d_msgs::msg::Path2D path;
  size_t comparisons=0;
  for(int i=0;i<1000;++i) {
   if(i%71==0){original->reset();instrumented->reset();}
   p.x+=step(rng);p.y+=step(rng);p.theta+=step(rng)*3;
   original->prepare(p,nav_2d_msgs::msg::Twist2D(),goal,path);instrumented->prepare(p,nav_2d_msgs::msg::Twist2D(),goal,path);
   for(double x:{-.1,0.,.03,.1})for(double y:{-.1,0.,.1})for(double w:{-.3,0.,.3}) {
    if(allowed(original,x,y,w)!=allowed(instrumented,x,y,w))throw std::runtime_error("Policy parity mismatch");
    ++comparisons;
   }
   cmd.x=(i%3==0)?.1:0.;cmd.y=0.;cmd.theta=(i%5<2)?-.2:.2;
   original->debrief(cmd);instrumented->debrief(cmd);
  }
  auto reader=std::make_shared<rclcpp::Node>("oscillation_reader");
  if(reader->count_publishers("/third_impact/oscillation")!=0) throw std::runtime_error("Logging false must not create a publisher");
  std::set<std::string> reasons;
  bool schema=false;
  auto sub=reader->create_subscription<std_msgs::msg::String>("/third_impact/oscillation",100,
   [&](std_msgs::msg::String::ConstSharedPtr msg) {
    auto doc=QJsonDocument::fromJson(QByteArray::fromStdString(msg->data));
    if(doc.isNull())throw std::runtime_error("Invalid JSON diagnostics");
    auto o=doc.object();reasons.insert(o["reset_reason"].toString().toStdString());
    schema=o.contains("reference_pose")&&o.contains("flags_before")&&o.contains("distance_ready")&&o.contains("selected_command");
   });
  node->declare_parameter("Logged.Oscillation.debug_logging",true);
  auto logged=loader.createSharedInstance("third_impact::LoggedOscillationCritic");
  logged->initialize(node,"Oscillation","Logged",nullptr);
  auto spin=[&](){for(int i=0;i<20;++i){rclcpp::spin_some(reader);std::this_thread::sleep_for(std::chrono::milliseconds(5));}};
  spin();
  auto flip=[&](auto &critic){p=geometry_msgs::msg::Pose2D();critic->reset();critic->prepare(p,nav_2d_msgs::msg::Twist2D(),goal,path);cmd=nav_2d_msgs::msg::Twist2D();cmd.theta=.2;critic->debrief(cmd);cmd.theta=-.2;critic->debrief(cmd);};
  flip(logged);if(allowed(logged,0,0,.2))throw std::runtime_error("Expected positive turn blocked");
  p.x=.051;logged->prepare(p,nav_2d_msgs::msg::Twist2D(),goal,path);logged->debrief(cmd);
  if(!allowed(logged,0,0,.2))throw std::runtime_error("Distance reset failed");
  spin();
  flip(logged);p.theta=.201;logged->prepare(p,nav_2d_msgs::msg::Twist2D(),goal,path);logged->debrief(cmd);spin();
  node->declare_parameter("Timed.Oscillation.debug_logging",true);
  node->declare_parameter("Timed.Oscillation.oscillation_reset_time",.01);
  auto timed=loader.createSharedInstance("third_impact::LoggedOscillationCritic");timed->initialize(node,"Oscillation","Timed",nullptr);spin();
  flip(timed);std::this_thread::sleep_for(std::chrono::milliseconds(20));timed->debrief(cmd);spin();
  if(!schema || !reasons.count("distance") || !reasons.count("angle") || !reasons.count("time") || !reasons.count("controller_reset"))throw std::runtime_error("Missing diagnostics/reset reason");
  std::cout<<"PASS: "<<comparisons<<" stock/instrumented decisions match; distance, angle, time, controller reset diagnostics received\n";
 }catch(const std::exception &e){std::cerr<<e.what()<<"\n";rclcpp::shutdown();return 1;}
 rclcpp::shutdown();return 0;
}
