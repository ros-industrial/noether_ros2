#include <noether_ros/located_vector_generator_widgets.h>

#include <noether_gui/plugin_interface.h>

using namespace noether;

namespace noether_ros
{
EXPORT_SIMPLE_DIRECTION_GENERATOR_WIDGET_PLUGIN(LocatedVectorDirectionGeneratorWidget, LocatedVectorDirection)
EXPORT_SIMPLE_ORIGIN_GENERATOR_WIDGET_PLUGIN(LocatedVectorOriginGeneratorWidget, LocatedVectorOrigin)
}  // namespace noether_ros
