#pragma once

#include <noether_ros/msg/located_vector.hpp>

#include <noether_tpp/macros.h>
#include <noether_tpp/tool_path_planners/raster/raster_planner.h>
#include <optional>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

FWD_DECLARE_YAML_STRUCTS()

namespace noether_ros
{
class LocatedVectorSubscriber
{
public:
  LocatedVectorSubscriber();
  virtual ~LocatedVectorSubscriber() = default;

  Eigen::Isometry3d lookupTransform(const std::string& source, const std::string& target) const;
  std::optional<noether_ros::msg::LocatedVector> getMessage() const;

protected:
  mutable std::mutex mutex_;
  noether_ros::msg::LocatedVector::ConstSharedPtr msg_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<noether_ros::msg::LocatedVector>::SharedPtr sub_;
  tf2_ros::Buffer::SharedPtr buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
};

class LocatedVectorDirectionGenerator : public noether::DirectionGenerator
{
public:
  LocatedVectorDirectionGenerator() = default;

  Eigen::Vector3d generate(const pcl::PolygonMesh& mesh) const override;
};

using LocatedVectorDirectionGeneratorWidget = noether::BaseWidget;

class LocatedVectorOriginGenerator : public noether::OriginGenerator
{
public:
  LocatedVectorOriginGenerator() = default;

  Eigen::Vector3d generate(const pcl::PolygonMesh& mesh) const override;
};

}  // namespace noether_ros

FWD_DECLARE_YAML_CONVERT(noether_ros::LocatedVectorOriginGenerator)
FWD_DECLARE_YAML_CONVERT(noether_ros::LocatedVectorDirectionGenerator)
