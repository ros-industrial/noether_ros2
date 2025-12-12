// Copyright (c) 2008, Willow Garage, Inc.
// Copyright (c) 2018, Bosch Software Innovations GmbH.
// All rights reserved.

#pragma once

#include <memory>
#include <QCursor>  // NOLINT cpplint cannot handle include order
#include <OgreVector.h>
#include <rviz_common/tool.hpp>

namespace rviz_rendering
{
class Line;
}

namespace rviz_common
{
namespace properties
{
class ColorProperty;
}
}  // namespace rviz_common

namespace noether_ros
{
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

private:
  // void setStatusMessage();
  void processLeftButton(const Ogre::Vector3& pos);
  void processRightButton();

  rviz_common::properties::ColorProperty* color_property_;

  std::shared_ptr<rviz_rendering::Line> line_;
  Ogre::Vector3 start_;
  Ogre::Vector3 end_;
  bool is_line_started_;
  float length_;

  QCursor std_cursor_;
  QCursor hit_cursor_;
};

}  // namespace noether_ros
