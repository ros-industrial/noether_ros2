#pragma once

#include <noether_tpp/macros.h>
#include <noether_tpp/tool_path_planners/raster/raster_planner.h>

FWD_DECLARE_YAML_STRUCTS()

namespace noether_ros
{
class LocatedVectorDirectionGenerator : public noether::DirectionGenerator
{
public:
  LocatedVectorDirectionGenerator(const std::string& service_name);

  Eigen::Vector3d generate(const pcl::PolygonMesh& mesh) const override;

protected:
  LocatedVectorDirectionGenerator() = default;
  DECLARE_YAML_FRIEND_CLASSES(LocatedVectorDirectionGenerator)

  std::string service_name_;
};

class LocatedVectorOriginGenerator : public noether::OriginGenerator
{
public:
  LocatedVectorOriginGenerator(const std::string& service_name);

  Eigen::Vector3d generate(const pcl::PolygonMesh& mesh) const override;

protected:
  LocatedVectorOriginGenerator() = default;
  DECLARE_YAML_FRIEND_CLASSES(LocatedVectorOriginGenerator)

  std::string service_name_;
};

}  // namespace noether_ros

FWD_DECLARE_YAML_CONVERT(noether_ros::LocatedVectorOriginGenerator)
FWD_DECLARE_YAML_CONVERT(noether_ros::LocatedVectorDirectionGenerator)
