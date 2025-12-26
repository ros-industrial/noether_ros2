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
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/properties/parse_color.hpp>
#include <rviz_rendering/objects/line.hpp>

#if __has_include(<rclcpp/version.h>)
#include <rclcpp/version.h>

#if (RCLCPP_VERSION_MAJOR >= 28)
#define QOS_REQUIRED_IN_SERVICE
#endif

#endif

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

  service_name_property_ = new rviz_common::properties::StringProperty(
      "Service name", "get_located_vector", "Desc", getPropertyContainer(), SLOT(updateServiceName()), this);
}

LocatedVectorTool::~LocatedVectorTool()
{
  executor_.cancel();
  executor_thread_.join();
}

void LocatedVectorTool::onInitialize()
{
  // Create the line display
  line_ = std::make_shared<Line>(context_->getSceneManager());
  updateLineColor();
  updateRenderAsOverlay();

  std_cursor_ = rviz_common::getDefaultCursor();
  hit_cursor_ = rviz_common::makeIconCursor("package://rviz_common/icons/crosshair.svg");

  updateServiceName();
}

void LocatedVectorTool::updateServiceName()
{
  const std::string service_name = service_name_property_->getStdString();

  rclcpp::Node::SharedPtr node = context_->getRosNodeAbstraction().lock()->get_raw_node();

  executor_.cancel();
  if (executor_thread_.joinable())
    executor_thread_.join();

  executor_callback_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  executor_.add_callback_group(executor_callback_group_, node->get_node_base_interface());

#ifdef QOS_REQUIRED_IN_SERVICE
  rclcpp::QoS qos(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT, 1),
                  rmw_qos_profile_services_default);
  server_ = node->create_service<srv::GetLocatedVector>(
      service_name,
      std::bind(&LocatedVectorTool::callback, this, std::placeholders::_1, std::placeholders::_2),
      qos,
      executor_callback_group_);
#else
  server_ = node->create_service<srv::GetLocatedVector>(
      service_name,
      std::bind(&LocatedVectorTool::callback, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default,
      executor_callback_group_);
#endif

  executor_thread_ = std::thread([&]() { executor_.spin(); });
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

void LocatedVectorTool::callback(srv::GetLocatedVector::Request::ConstSharedPtr req,
                                 srv::GetLocatedVector::Response::SharedPtr res) const
{
  if (!start_)
  {
    res->success = false;
    res->message = "Start point has not been set";
    return;
  }

  if (!end_)
  {
    res->success = false;
    res->message = "End point has not been set";
    return;
  }

  res->source.header.frame_id = context_->getFixedFrame().toStdString();
  res->source.point.x = start_->x;
  res->source.point.y = start_->y;
  res->source.point.z = start_->z;

  res->target.header.frame_id = context_->getFixedFrame().toStdString();
  res->target.point.x = end_->x;
  res->target.point.y = end_->y;
  res->target.point.z = end_->z;

  res->success = true;
}

}  // namespace noether_ros

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(noether_ros::LocatedVectorTool, rviz_common::Tool)
