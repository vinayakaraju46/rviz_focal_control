
#pragma once

#include <memory>

#include <QTimer>
#include <QLabel>
#include <Ogre.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <rviz_common/panel.hpp>

namespace rviz_focal_control
{

class FocalPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit FocalPanel(QWidget * parent = nullptr);

  void onInitialize() override;

private:
  // Process incoming ROS 2 messages
  void processMessages();

  // Update the RViz2 focal point
  void applyOffset(
    const geometry_msgs::msg::Vector3 & offset);

  // Publish a marker at the focal point
  void publishFocalMarker(
    const Ogre::Vector3 & position);

  void resetFocalPoint(const std::string & frame);
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr reset_sub_;

  // ROS 2 node
  rclcpp::Node::SharedPtr node_;

  // Subscriber for focal point offsets
  rclcpp::Subscription<
    geometry_msgs::msg::Vector3>::SharedPtr sub_;

  // Publisher for focal point marker
  rclcpp::Publisher<
    visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  // Qt timer for processing ROS 2 callbacks
  QTimer * timer_;
  QLabel * status_;
};

}  // namespace rviz_focal_control
