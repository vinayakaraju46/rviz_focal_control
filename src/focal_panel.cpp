
#include "rviz_focal_control/focal_panel.hpp"

#include <cmath>

#include <QLabel>
#include <QVBoxLayout>
#include <QString>
#include <OgreCamera.h>
#include <OgreSceneNode.h>
#include <rviz_common/frame_position_tracking_view_controller.hpp>

#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_common/view_controller.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/properties/vector_property.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace rviz_focal_control
{

FocalPanel::FocalPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * layout = new QVBoxLayout();

  layout->addWidget(
    new QLabel("RViz Focal Point Controller"));

  status_ = new QLabel("Waiting for RViz view...");
  status_->setWordWrap(true);
  layout->addWidget(status_);

  setLayout(layout);

  timer_ = new QTimer(this);

  connect(
    timer_,
    &QTimer::timeout,
    this,
    &FocalPanel::processMessages);
}

void FocalPanel::onInitialize()
{
  node_ = std::make_shared<rclcpp::Node>(
    "rviz_focal_controller");

  sub_ = node_->create_subscription<
    geometry_msgs::msg::Vector3>(
    "/rviz/focal_point_offset",
    10,
    [this](
      geometry_msgs::msg::Vector3::SharedPtr msg)
    {
      applyOffset(*msg);
    });

  marker_pub_ = node_->create_publisher<
    visualization_msgs::msg::Marker>(
      "/rviz/focal_point_marker",
      rclcpp::QoS(1).transient_local()
  );

  reset_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/rviz/focal_point_reset", 10,
    [this](std_msgs::msg::String::SharedPtr msg) {resetFocalPoint(msg->data);});

  timer_->start(20);
}

void FocalPanel::publishFocalMarker(
    const Ogre::Vector3 & position)
{
    if (!marker_pub_) {
        return;
    }

    auto * controller = getDisplayContext()->getViewManager()->getCurrent();
    Ogre::Vector3 world_position = position;
    if (dynamic_cast<rviz_common::FramePositionTrackingViewController *>(controller)) {
        auto * camera = controller->getCamera();
        auto * camera_node = camera ? camera->getParentSceneNode() : nullptr;
        auto * target_node = camera_node ? camera_node->getParentSceneNode() : nullptr;
        if (!target_node) {
            status_->setText("Waiting for the view camera...");
            return;
        }
        // Focal Point is local to the tracked target, not the fixed frame.
        world_position = target_node->convertLocalToWorldPosition(position);
    }

    visualization_msgs::msg::Marker marker;

    // Header
    marker.header.frame_id = getDisplayContext()->getFixedFrame().toStdString();
    marker.header.stamp = node_->now();

    // Marker identity
    marker.ns = "focal_crosshair";
    marker.id = 0;

    // Crosshair using line segments
    marker.type =
        visualization_msgs::msg::Marker::LINE_LIST;

    marker.action =
        visualization_msgs::msg::Marker::ADD;

    // Keep the exact goal position in the pose, independent of reticle geometry.
    marker.pose.position.x = world_position.x;
    marker.pose.position.y = world_position.y;
    marker.pose.position.z = world_position.z;
    auto * camera = controller->getCamera();
    const auto orientation = camera ? camera->getDerivedOrientation() :
        Ogre::Quaternion::IDENTITY;
    marker.pose.orientation.x = orientation.x;
    marker.pose.orientation.y = orientation.y;
    marker.pose.orientation.z = orientation.z;
    marker.pose.orientation.w = orientation.w;

    // Face the camera so the target remains legible at oblique viewing angles.
    marker.scale.x = 0.035;
    marker.color.r = 0.1;
    marker.color.g = 1.0;
    marker.color.b = 0.85;
    marker.color.a = 1.0;
    auto accent = marker.color;
    accent.r = 1.0;
    accent.g = 0.75;
    accent.b = 0.15;

    auto segment = [&](double x1, double y1, double x2, double y2, bool bright)
    {
        geometry_msgs::msg::Point p;
        p.x = x1;
        p.y = y1;
        marker.points.push_back(p);
        p.x = x2;
        p.y = y2;
        marker.points.push_back(p);
        const auto color = bright ? accent : marker.color;
        marker.colors.push_back(color);
        marker.colors.push_back(color);
    };
    constexpr double pi = 3.141592653589793;
    auto ring = [&](double radius, bool broken)
    {
        constexpr int segments = 96;
        for (int i = 0; i < segments; ++i) {
            // Four small gaps in the outer ring separate the aiming ticks.
            if (broken && (i % 24 < 2 || i % 24 >= 22)) {
                continue;
            }
            const double a = 2.0 * pi * i / segments;
            const double b = 2.0 * pi * (i + 1) / segments;
            segment(radius * std::cos(a), radius * std::sin(a),
                radius * std::cos(b), radius * std::sin(b), false);
        }
    };
    ring(0.55, true);
    ring(0.30, false);
    for (int i = 0; i < 4; ++i) {
        const double a = i * pi / 2.0;
        segment(0.40 * std::cos(a), 0.40 * std::sin(a),
            0.78 * std::cos(a), 0.78 * std::sin(a), true);
    }
    segment(-0.07, 0.0, 0.07, 0.0, true);
    segment(0.0, -0.07, 0.0, 0.07, true);

    // Keep marker visible indefinitely
    marker.lifetime = rclcpp::Duration(0, 0);

    // Publish crosshair
    marker_pub_->publish(marker);
    status_->setText(QString("Publishing /rviz/focal_point_marker\nFrame: %1\nPosition: %2, %3, %4\nMarker subscribers: %5")
        .arg(QString::fromStdString(marker.header.frame_id))
        .arg(world_position.x, 0, 'f', 2).arg(world_position.y, 0, 'f', 2)
        .arg(world_position.z, 0, 'f', 2)
        .arg(static_cast<qulonglong>(marker_pub_->get_subscription_count())));
}



void FocalPanel::processMessages()
{
    if (node_) {
        rclcpp::spin_some(node_);
    }

    auto * manager =
        getDisplayContext()->getViewManager();

    if (!manager) {
        return;
    }

    auto * controller = manager->getCurrent();

    if (!controller) {
        return;
    }

    auto * property = controller->subProp("Focal Point");

    auto * vector_property =
        dynamic_cast<
            rviz_common::properties::VectorProperty *>(
                property);

    if (!vector_property) {
        status_->setText("No Focal Point in this view. Select Views → Type → Orbit or XYOrbit.");
        return;
    }

    publishFocalMarker(vector_property->getVector());
}


void FocalPanel::resetFocalPoint(const std::string & frame)
{
  auto * context = getDisplayContext();
  auto * manager = context ? context->getViewManager() : nullptr;
  auto * controller = manager ? manager->getCurrent() : nullptr;
  auto * focal = controller ? dynamic_cast<rviz_common::properties::VectorProperty *>(
    controller->subProp("Focal Point")) : nullptr;
  if (!focal) {
    RCLCPP_WARN(node_->get_logger(), "Cannot reset: active view has no Focal Point");
    return;
  }

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (frame.empty() ||
    !context->getFrameManager()->getTransform(frame, position, orientation))
  {
    RCLCPP_WARN(node_->get_logger(), "Cannot reset: TF for '%s' unavailable", frame.c_str());
    return;
  }

  auto * target = controller->subProp("Target Frame");
  if (!dynamic_cast<rviz_common::FramePositionTrackingViewController *>(controller) ||
    !target || !target->getValue().isValid())
  {
    RCLCPP_WARN(node_->get_logger(), "Cannot reset: select an Orbit or XYOrbit view");
    return;
  }

  // Changing target may preserve the old world-space focal point. Clear that
  // compensating offset afterwards so the camera actually returns to the car.
  target->setValue(QString::fromStdString(frame));
  position = Ogre::Vector3::ZERO;
  focal->setVector(position);
  publishFocalMarker(position);
  context->queueRender();
}

void FocalPanel::applyOffset(
  const geometry_msgs::msg::Vector3 & offset)
{
  auto * manager =
    getDisplayContext()->getViewManager();

  if (!manager) {
    return;
  }

  auto * controller = manager->getCurrent();

  if (!controller) {
    return;
  }

  // Search the active view's property tree
  auto * property =
    controller->subProp("Focal Point");

  if (!property) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Active view has no Focal Point property");
    return;
  }

  auto * vector_property =
    dynamic_cast<
      rviz_common::properties::VectorProperty *>(
      property);

  if (!vector_property) {
    return;
  }

  Ogre::Vector3 current =
    vector_property->getVector();

  Ogre::Vector3 updated(
    current.x + offset.x,
    current.y + offset.y,
    current.z + offset.z);

  vector_property->setVector(updated);
  // Update visible marker
  publishFocalMarker(updated);
}

}  // namespace rviz_focal_control

PLUGINLIB_EXPORT_CLASS(
  rviz_focal_control::FocalPanel,
  rviz_common::Panel)
