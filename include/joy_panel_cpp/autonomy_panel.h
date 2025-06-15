#ifndef AUTONOMY_PANEL_H
#define AUTONOMY_PANEL_H

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <sensor_msgs/msg/joy.hpp>

// Forward declarations for Qt classes
class QCheckBox;
class QLabel;

namespace joy_panel_cpp
{

// All panels in RViz are plugins, and inherit from rviz_common::Panel.
class AutonomyPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  // The constructor must have this signature.
  explicit AutonomyPanel(QWidget* parent = nullptr);

  // Override the onInitialize() function from the base class.
  // This is called by RViz when the panel is first created.
  void onInitialize() override;

// Qt slots are special callback functions.
protected Q_SLOTS:
  // This function will be called whenever the checkbox state changes.
  void onAutonomyStateChanged(int state);

protected:
  // ROS 2 node handle and publisher for the /joy topic.
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_publisher_;

  // UI elements
  QCheckBox* autonomy_switch_;
  QLabel* status_label_;
};

} // end namespace joy_panel_cpp

#endif // AUTONOMY_PANEL_H