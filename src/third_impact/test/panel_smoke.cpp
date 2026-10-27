#include <QApplication>
#include <QPushButton>
#include <QThread>
#include <QLabel>
#include <QTableWidget>
#include <std_msgs/msg/string.hpp>
#include <pluginlib/class_loader.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/display.hpp>
#include <rclcpp/rclcpp.hpp>
#include <iostream>
int main(int argc, char **argv) {
 QApplication app(argc, argv);
 rclcpp::init(argc, argv);
 try {
  pluginlib::ClassLoader<rviz_common::Panel> loader("rviz_common", "rviz_common::Panel");
  auto panel=loader.createSharedInstance("third_impact/GoalPanel");
  auto status_panel=loader.createSharedInstance("third_impact/StatusPanel");
  if(!status_panel->findChildren<QPushButton*>().empty()) return 5;
  auto state=status_panel->findChild<QLabel*>("mission_state");
  if(!state) return 6;
  auto source=std::make_shared<rclcpp::Node>("isolated_status_source");
  auto publisher=source->create_publisher<std_msgs::msg::String>("/third_impact/status",10);
  auto check=[&](const std::string &json, const QString &expected){
   std_msgs::msg::String msg; msg.data=json;
   for(int i=0;i<30;++i) { publisher->publish(msg); QThread::msleep(10); app.processEvents(); }
   if(state->text()!=expected) throw std::runtime_error("Status label mismatch: "+state->text().toStdString());
  };
  check(R"({"mode":"mission","state":"PATROL","manual":false,"ready":true,"has_goal":true,"has_rectangle":true})", "정찰 중");
  check(R"({"mode":"mission","state":"PATROL","manual":false,"ready":true,"has_goal":true,"has_rectangle":true,"detail":"Navigation rejected; retrying"})", "정찰 요청 재시도 중");
  auto table=status_panel->findChild<QTableWidget*>("mission_table");
  if(!table || table->columnCount()!=3 || table->editTriggers()!=QAbstractItemView::NoEditTriggers) return 8;
  auto contains=[&](const QString &text) {
   for(int r=0;r<table->rowCount();++r) for(int c=0;c<table->columnCount();++c)
    if(table->item(r,c) && table->item(r,c)->text().contains(text)) return true;
   return false;
  };
  if(!contains("이동 요청을 거절")) return 8;
  check(R"({"mode":"mission","state":"PATROL","manual":false,"ready":true,"has_goal":true,"has_rectangle":true,"detail":"Navigation accepted: patrol"})", "정찰 중");
  if(!contains("요청을 수락") || contains("거절")) return 9;
  check(R"({"mode":"mission","state":"PATROL","intent":"patrol","manual":false,"ready":true,"has_goal":true,"has_rectangle":true,"detail":"Navigation action finished: purpose=patrol, status=4, cancellation=False"})", "정찰 중");
  if(!contains("성공 (4)") || !contains("요청 안 함") || contains("cancellation=False")) return 10;
  check(R"({"mode":"mission","state":"PATROL","intent":"patrol","manual":false,"ready":true,"has_goal":true,"has_rectangle":true,"navigation":{"phase":"idle","last_result":{"purpose":"patrol","status":5,"cancel_requested":true,"patrol_arrived":true},"patrol_visits":2,"patrol_tolerance_m":0.05}})", "정찰 중");
  if(!contains("취소됨 (5)") || !contains("정상 전환") || !contains("5 cm 이내")) return 11;
  check(R"({"mode":"mission","state":"RETURN","manual":false,"ready":true,"has_goal":true,"has_rectangle":true})", "목표로 복귀 중");
  check(R"({"mode":"mission","state":"ALIGN","manual":false,"ready":true,"has_goal":true,"has_rectangle":true})", "목표 위치 정밀 정렬 중");
  check(R"({"mode":"mission","state":"RETURN","manual":true,"ready":true})", "수동 조작 중");
  check(R"({"mode":"mission","state":"PATROL","manual":false,"ready":false})", "위치·센서 준비 대기");
  check(R"({"mode":"mission","state":"WAIT","manual":false,"ready":false,"localization":{"fresh":true,"state":"SEARCHING","ready":false,"reason":"현재 LiDAR로 지도 전체를 탐색합니다."}})", "자동 위치 탐색 중");
  if(!contains("지도 전체를 탐색")) return 13;
  check(R"({"mode":"mission","state":"WAIT","manual":false,"ready":false,"localization":{"fresh":true,"state":"AMBIGUOUS","ready":false,"reason":"비슷한 위치 후보가 여러 개입니다."}})", "위치 후보가 여러 개");
  check(R"({"mode":"mission","state":"HOLD","manual":false,"ready":true})", "도착 완료 · 대기 중");
  for(int i=0;i<320;++i) { QThread::msleep(10); app.processEvents(); }
  if(state->text()!="상태 수신 끊김") throw std::runtime_error("Timeout label mismatch: "+state->text().toStdString());
  if(table->rowCount()!=0) return 12;
  std::cout << "PASS: read-only status panel transitions, manual/readiness overrides and stale timeout\n";
  if(panel->findChildren<QPushButton*>().size()!=2) return 2;
  app.processEvents();
  std::cout << "PASS: RViz plugin loaded; Save Goal and Save Rectangle buttons constructed\n";
  pluginlib::ClassLoader<rviz_common::Display> displays("rviz_common", "rviz_common::Display");
  auto markers=displays.createSharedInstance("rviz_default_plugins/MarkerArray");
  auto editor=displays.createSharedInstance("rviz_default_plugins/InteractiveMarkers");
  bool topic=false, marker_namespace=false;
  for(int i=0;i<markers->numChildren();++i)
   topic |= markers->childAt(i)->getName()=="Topic";
  for(int i=0;i<editor->numChildren();++i)
   marker_namespace |= editor->childAt(i)->getName()=="Interactive Markers Namespace";
  if(!topic || !marker_namespace) return 3;
  std::cout << "PASS: installed RViz display property names match corrected topic and editor namespace keys\n";
  // Reproduce RViz SIGINT: Qt timers may fire before the panel is destroyed.
  rclcpp::shutdown();
  QThread::msleep(80);
  app.processEvents();
  for (auto *button: panel->findChildren<QPushButton*>())
   if (button->isEnabled()) return 4;
  panel.reset();
  status_panel.reset();
  std::cout << "PASS: Qt timer after ROS shutdown does not create a guard condition or crash\n";
 } catch(const std::exception &e) {std::cerr<<e.what()<<"\n"; rclcpp::shutdown(); return 1;}
 rclcpp::shutdown(); return 0;
}
