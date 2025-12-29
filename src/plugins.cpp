#include <noether_ros/located_vector_generators.h>

#include <noether_tpp/plugin_interface.h>

using namespace noether;

namespace noether_ros
{
EXPORT_SIMPLE_DIRECTION_GENERATOR_PLUGIN(LocatedVectorDirectionGenerator, LocatedVectorDirection)
EXPORT_SIMPLE_ORIGIN_GENERATOR_PLUGIN(LocatedVectorOriginGenerator, LocatedVectorOrigin)
}  // namespace noether_ros
