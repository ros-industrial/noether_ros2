#pragma once

#include <noether_gui/widgets.h>

namespace noether_ros
{
struct LocatedVectorDirectionGeneratorWidget : public noether::BaseWidget
{
  using noether::BaseWidget::BaseWidget;
  using noether::BaseWidget::configure;

  void save(YAML::Node&) const override;
};

struct LocatedVectorOriginGeneratorWidget : public noether::BaseWidget
{
  using noether::BaseWidget::BaseWidget;
  using noether::BaseWidget::configure;

  void save(YAML::Node&) const override;
};

}  // namespace noether_ros
