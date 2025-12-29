#pragma once

#include <noether_gui/widgets.h>

class QLineEdit;

namespace noether_ros
{
class LocatedVectorDirectionGeneratorWidget : public noether::BaseWidget
{
public:
  LocatedVectorDirectionGeneratorWidget(QWidget* parent = nullptr);

  void configure(const YAML::Node&) override;
  void save(YAML::Node&) const override;

protected:
  QLineEdit* line_edit_service_name_;
};

struct LocatedVectorOriginGeneratorWidget : public noether::BaseWidget
{
public:
  LocatedVectorOriginGeneratorWidget(QWidget* parent = nullptr);

  void configure(const YAML::Node&) override;
  void save(YAML::Node&) const override;

protected:
  QLineEdit* line_edit_service_name_;
};

}  // namespace noether_ros
