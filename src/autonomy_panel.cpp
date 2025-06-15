#include "joy_panel_cpp/autonomy_panel.h"
#include <rviz_common/display_context.hpp>
#include <rclcpp/rclcpp.hpp>

#include <QVBoxLayout>
#include <QCheckBox>
#include <QLabel>

namespace joy_panel_cpp
{

// Constructor: create the UI and connect signals to slots.
AutonomyPanel::AutonomyPanel(QWidget* parent)
  : rviz_common::Panel(parent)
{
  // Create a vertical layout for the panel
  QVBoxLayout* layout = new QVBoxLayout(this);

  // Create the UI elements
  layout->addWidget(new QLabel("Autonomy Control"));
  autonomy_switch_ = new QCheckBox("Enable Autonomy");
  autonomy_switch_->setChecked(true); // Start in autonomy mode
  layout->addWidget(autonomy_switch_);

  status_label_ = new QLabel("Status: Awaiting Initialization");
  layout->addWidget(status_label_);
  
  // Set the layout for this widget
  setLayout(layout);
  
  // Connect the stateChanged signal of the checkbox to our slot
  connect(autonomy_switch_, &QCheckBox::stateChanged, this, &AutonomyPanel::onAutonomyStateChanged);
}

// onInitialize() is called by RViz after the constructor.
// It is the place to create ROS 2 objects.
void AutonomyPanel::onInitialize()
{
  // Get the node handle using the recommended method from the display context.
  // Add .lock() to convert the weak_ptr to a shared_ptr before calling methods.
  node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  joy_publisher_ = node_->create_publisher<sensor_msgs::msg::Joy>("/joy", 10);
  
  status_label_->setText("Status: Ready. Toggling switch will publish to /joy.");
  
  // Trigger an initial publish to set a default state
  onAutonomyStateChanged(autonomy_switch_->checkState());
}


// Callback function for when the checkbox state changes.
void AutonomyPanel::onAutonomyStateChanged(int state)
{
  auto joy_msg = std::make_unique<sensor_msgs::msg::Joy>();
  
  // Initialize axes and buttons arrays to a common size to avoid errors
  joy_msg->axes.resize(8, 0.0f);
  joy_msg->buttons.resize(12, 0);

  // Qt::Checked corresponds to the checked state
  if (static_cast<Qt::CheckState>(state) == Qt::Checked) {
    joy_msg->axes[4] = 1.0f; // Enable autonomy
    RCLCPP_INFO(node_->get_logger(), "Autonomy Switch ON: Publishing axes[4] = 1.0");
  } else {
    joy_msg->axes[4] = 0.0f; // Disable autonomy
    RCLCPP_INFO(node_->get_logger(), "Autonomy Switch OFF: Publishing axes[4] = 0.0");
  }
  
  joy_publisher_->publish(std::move(joy_msg));
}

} // end namespace joy_panel_cpp

// This is the macro that registers this class as a plugin.
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(joy_panel_cpp::AutonomyPanel, rviz_common::Panel)