// Copyright (c) 2008, Willow Garage, Inc.
// Copyright (c) 2018, Bosch Software Innovations GmbH.
// All rights reserved.

#include "rviz_located_vector_tool.h"

#include <QString>  // NOLINT: cpplint is unable to handle the include order here
#include <OgreSceneNode.h>
#include <rviz_common/display_context.hpp>
#include <rviz_common/load_resource.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_common/interaction/view_picker_iface.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/parse_color.hpp>
#include <rviz_rendering/objects/line.hpp>

namespace noether_ros
{
LocatedVectorTool::LocatedVectorTool() : is_line_started_(false), length_(-1)
{
  shortcut_key_ = 'l';

  color_property_ = new rviz_common::properties::ColorProperty("Line color",
                                                               Qt::darkYellow,
                                                               "The topic on which to publish points.",
                                                               getPropertyContainer(),
                                                               SLOT(updateLineColor()),
                                                               this);
}

void LocatedVectorTool::onInitialize()
{
  // Create the publisher
  rclcpp::Node::SharedPtr node = context_->getRosNodeAbstraction().lock()->get_raw_node();
  rclcpp::QoS qos(1);
  qos.transient_local();
  qos.reliable();
  publisher_ = node->create_publisher<noether_ros::msg::LocatedVector>("located_vector", qos);

  // Create the line display
  line_ = std::make_shared<rviz_rendering::Line>(context_->getSceneManager());
  updateLineColor();

  std_cursor_ = rviz_common::getDefaultCursor();
  hit_cursor_ = rviz_common::makeIconCursor("package://rviz_common/icons/crosshair.svg");
}

void LocatedVectorTool::activate() { is_line_started_ = false; }

void LocatedVectorTool::deactivate() {}

int LocatedVectorTool::processMouseEvent(rviz_common::ViewportMouseEvent& event)
{
  Ogre::Vector3 pos;
  bool success = context_->getViewPicker()->get3DPoint(event.panel, event.x, event.y, pos);
  setCursor(success ? hit_cursor_ : std_cursor_);

  if (is_line_started_ && success)
  {
    line_->setPoints(start_, pos);
    length_ = (start_ - pos).length();
  }

  if (event.leftUp() && success)
  {
    processLeftButton(pos);
    return Render;
  }
  if (event.rightUp())
  {
    processRightButton();
  }

  return 0;
}

void LocatedVectorTool::updateLineColor()
{
  Ogre::ColourValue color = rviz_common::properties::qtToOgre(color_property_->getColor());
  line_->setColor(color);
}

void LocatedVectorTool::processLeftButton(const Ogre::Vector3& pos)
{
  if (is_line_started_)
  {
    end_ = pos;
    line_->setPoints(start_, end_);
    is_line_started_ = false;

    // Publish a located vector message
    msg::LocatedVector msg;
    msg.source.header.frame_id = context_->getFixedFrame().toStdString();
    msg.source.point.x = start_.x;
    msg.source.point.y = start_.y;
    msg.source.point.z = start_.z;
    msg.target.header.frame_id = context_->getFixedFrame().toStdString();
    msg.target.point.x = end_.x;
    msg.target.point.y = end_.y;
    msg.target.point.z = end_.z;
    publisher_->publish(msg);
  }
  else
  {
    start_ = pos;
    is_line_started_ = true;
  }
}

void LocatedVectorTool::processRightButton()
{
  is_line_started_ = false;
  line_->setVisible(false);
}

}  // namespace noether_ros

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(noether_ros::LocatedVectorTool, rviz_common::Tool)
