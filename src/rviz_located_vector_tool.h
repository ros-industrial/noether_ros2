// Copyright (c) 2008, Willow Garage, Inc.
// Copyright (c) 2018, Bosch Software Innovations GmbH.
// All rights reserved.

#pragma once

#include <noether_ros/msg/located_vector.hpp>

#include <memory>
#include <QCursor>  // NOLINT cpplint cannot handle include order
#include <OgreVector.h>
#include <rclcpp/publisher.hpp>
#include <rviz_common/tool.hpp>
#include <rviz_rendering/objects/line.hpp>

namespace rviz_common
{
namespace properties
{
class BoolProperty;
class ColorProperty;
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

  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  int processMouseEvent(rviz_common::ViewportMouseEvent& event) override;

public Q_SLOTS:
  void updateLineColor();
  void updateRenderAsOverlay();

private:
  void processLeftButton(const Ogre::Vector3& pos);
  void processRightButton();

  rclcpp::Publisher<noether_ros::msg::LocatedVector>::SharedPtr publisher_;

  rviz_common::properties::ColorProperty* color_property_;
  rviz_common::properties::BoolProperty* render_as_overlay_property_;

  std::shared_ptr<Line> line_;
  std::shared_ptr<Ogre::Vector3> start_;
  std::shared_ptr<Ogre::Vector3> end_;

  QCursor std_cursor_;
  QCursor hit_cursor_;
};

}  // namespace noether_ros
