#pragma once

#include <noether_tpp/macros.h>
#include <noether_tpp/tool_path_planners/raster/raster_planner.h>

FWD_DECLARE_YAML_STRUCTS()

namespace noether_ros
{
/**
 * @brief Calls a `noether_ros/srv/GetLocatedVector` service to generate the direction vector
 */
class LocatedVectorDirectionGenerator : public noether::DirectionGenerator
{
public:
  /**
   * @brief Constructor
   * @param service_name Name of the service providing `noether_ros/srv/GetLocatedVector`
   */
  LocatedVectorDirectionGenerator(const std::string& service_name);

  Eigen::Vector3d generate(const pcl::PolygonMesh& mesh) const override;

protected:
  LocatedVectorDirectionGenerator() = default;
  DECLARE_YAML_FRIEND_CLASSES(LocatedVectorDirectionGenerator)

  std::string service_name_;
};

/**
 * @brief Calls a `noether_ros/srv/GetLocatedVector` service to generate the origin location
 */
class LocatedVectorOriginGenerator : public noether::OriginGenerator
{
public:
  /**
   * @brief Constructor
   * @param service_name Name of the service providing `noether_ros/srv/GetLocatedVector`
   */
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
