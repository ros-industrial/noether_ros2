#include <noether_ros/located_vector_generator_widgets.h>

namespace noether_ros
{
void LocatedVectorDirectionGeneratorWidget::save(YAML::Node& config) const
{
  config["name"] = "LocatedVectorDirection";
}

void LocatedVectorOriginGeneratorWidget::save(YAML::Node& config) const { config["name"] = "LocatedVectorOrigin"; }

}  // namespace noether_ros
