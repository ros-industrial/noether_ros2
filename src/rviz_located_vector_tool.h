// Copyright (c) 2008, Willow Garage, Inc.
// Copyright (c) 2018, Bosch Software Innovations GmbH.
// All rights reserved.

#pragma once

#include <noether_ros/srv/get_located_vector.hpp>

#include <memory>
#include <QCursor>  // NOLINT cpplint cannot handle include order
#include <OgreVector.h>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/service.hpp>
#include <rviz_common/tool.hpp>
#include <rviz_rendering/objects/line.hpp>

namespace rviz_common
{
namespace properties
{
class BoolProperty;
class ColorProperty;
class StringProperty;
}  // namespace properties
}  // namespace rviz_common

namespace noether_ros
{
class Line : public rviz_rendering::Line
{
public:
  using rviz_rendering::Line::Line;

  Ogre::ManualObject* getManualObject() const;
  Ogre::MaterialPtr getManualObjectMaterial() const;
};

class LocatedVectorTool : public rviz_common::Tool
{
  Q_OBJECT

public:
  LocatedVectorTool();
  virtual ~LocatedVectorTool();

  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  int processMouseEvent(rviz_common::ViewportMouseEvent& event) override;

public Q_SLOTS:
  void updateLineColor();
  void updateRenderAsOverlay();
  void updateServiceName();

private:
  void processLeftButton(const Ogre::Vector3& pos);
  void processRightButton();
  void callback(srv::GetLocatedVector::Request::ConstSharedPtr req,
                srv::GetLocatedVector::Response::SharedPtr res) const;

  rviz_common::properties::ColorProperty* color_property_;
  rviz_common::properties::BoolProperty* render_as_overlay_property_;
  rviz_common::properties::StringProperty* service_name_property_;

  rclcpp::Service<srv::GetLocatedVector>::SharedPtr server_;
  std::thread executor_thread_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::CallbackGroup::SharedPtr executor_callback_group_;

  std::shared_ptr<Line> line_;
  std::shared_ptr<Ogre::Vector3> start_;
  std::shared_ptr<Ogre::Vector3> end_;

  QCursor std_cursor_;
  QCursor hit_cursor_;
};

}  // namespace noether_ros
