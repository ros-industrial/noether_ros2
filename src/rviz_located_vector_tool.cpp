// Copyright (c) 2008, Willow Garage, Inc.
// Copyright (c) 2018, Bosch Software Innovations GmbH.
// All rights reserved.

#include "rviz_located_vector_tool.h"

#include <QString>  // NOLINT: cpplint is unable to handle the include order here
#include <OgreManualObject.h>
#include <OgreRenderQueue.h>
#include <OgreSceneNode.h>
#include <rviz_common/display_context.hpp>
#include <rviz_common/load_resource.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_common/interaction/view_picker_iface.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/parse_color.hpp>
#include <rviz_rendering/objects/line.hpp>

namespace noether_ros
{
Ogre::ManualObject* Line::getManualObject() const { return manual_object_; }

Ogre::MaterialPtr Line::getManualObjectMaterial() const { return manual_object_material_; }

LocatedVectorTool::LocatedVectorTool() : rviz_common::Tool(), start_(nullptr), end_(nullptr)
{
  shortcut_key_ = 'l';

  color_property_ = new rviz_common::properties::ColorProperty("Line color",
                                                               Qt::darkYellow,
                                                               "The topic on which to publish points.",
                                                               getPropertyContainer(),
                                                               SLOT(updateLineColor()),
                                                               this);

  render_as_overlay_property_ = new rviz_common::properties::BoolProperty("Render as overlay",
                                                                          false,
                                                                          "Render the drawn lines on top of all other "
                                                                          "geometry",
                                                                          getPropertyContainer(),
                                                                          SLOT(updateRenderAsOverlay()),
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
  line_ = std::make_shared<Line>(context_->getSceneManager());
  updateLineColor();
  updateRenderAsOverlay();

  std_cursor_ = rviz_common::getDefaultCursor();
  hit_cursor_ = rviz_common::makeIconCursor("package://rviz_common/icons/crosshair.svg");
}

void LocatedVectorTool::activate() {}

void LocatedVectorTool::deactivate() {}

int LocatedVectorTool::processMouseEvent(rviz_common::ViewportMouseEvent& event)
{
  Ogre::Vector3 pos;
  bool success = context_->getViewPicker()->get3DPoint(event.panel, event.x, event.y, pos);
  setCursor(success ? hit_cursor_ : std_cursor_);

  // Update the line display
  if (success && start_ && !end_)
    line_->setPoints(*start_, pos);

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

void LocatedVectorTool::updateRenderAsOverlay()
{
  const bool render_as_overlay = render_as_overlay_property_->getBool();
  const Ogre::RenderQueueGroupID render_id =
      render_as_overlay ? Ogre::RenderQueueGroupID::RENDER_QUEUE_OVERLAY : Ogre::RenderQueueGroupID::RENDER_QUEUE_MAIN;

  line_->getManualObjectMaterial()->setDepthCheckEnabled(!render_as_overlay);
  line_->getManualObjectMaterial()->setDepthWriteEnabled(!render_as_overlay);
  line_->getManualObject()->setRenderQueueGroup(render_id);
}

void LocatedVectorTool::processLeftButton(const Ogre::Vector3& pos)
{
  if (start_)
  {
    if (end_)
    {
      // Both the start and end points are defined -> reset the start to the current position
      start_ = std::make_shared<Ogre::Vector3>(pos);
      end_.reset();
    }
    else
    {
      // Only the start point is defined -> set the current position as the end
      end_ = std::make_shared<Ogre::Vector3>(pos);

      // Publish a located vector message
      msg::LocatedVector msg;
      msg.source.header.frame_id = context_->getFixedFrame().toStdString();
      msg.source.point.x = start_->x;
      msg.source.point.y = start_->y;
      msg.source.point.z = start_->z;
      msg.target.header.frame_id = context_->getFixedFrame().toStdString();
      msg.target.point.x = end_->x;
      msg.target.point.y = end_->y;
      msg.target.point.z = end_->z;
      publisher_->publish(msg);
    }
  }
  else
  {
    start_ = std::make_shared<Ogre::Vector3>(pos);
  }
}

void LocatedVectorTool::processRightButton()
{
  start_.reset();
  end_.reset();
  line_->setVisible(false);
}

}  // namespace noether_ros

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(noether_ros::LocatedVectorTool, rviz_common::Tool)
