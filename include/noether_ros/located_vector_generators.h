#pragma once

#include <noether_tpp/macros.h>
#include <noether_tpp/tool_path_planners/raster/raster_planner.h>

FWD_DECLARE_YAML_STRUCTS()

namespace noether_ros
{
class LocatedVectorDirectionGenerator : public noether::DirectionGenerator
{
public:
  LocatedVectorDirectionGenerator() = default;

  Eigen::Vector3d generate(const pcl::PolygonMesh& mesh) const override;
};

class LocatedVectorOriginGenerator : public noether::OriginGenerator
{
public:
  LocatedVectorOriginGenerator() = default;

  Eigen::Vector3d generate(const pcl::PolygonMesh& mesh) const override;
};

}  // namespace noether_ros

FWD_DECLARE_YAML_CONVERT(noether_ros::LocatedVectorOriginGenerator)
FWD_DECLARE_YAML_CONVERT(noether_ros::LocatedVectorDirectionGenerator)
