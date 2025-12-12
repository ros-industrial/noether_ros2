#pragma once

#include <noether_ros/msg/located_vector.hpp>

#include <noether_tpp/macros.h>
#include <noether_tpp/tool_path_planners/raster/raster_planner.h>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

FWD_DECLARE_YAML_STRUCTS()

namespace noether_ros
{
class LocatedVectorDirectionGenerator : public noether::DirectionGenerator
{
public:
  LocatedVectorDirectionGenerator();

  Eigen::Vector3d generate(const pcl::PolygonMesh& mesh) const override;

protected:
  noether_ros::msg::LocatedVector::ConstSharedPtr msg_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<noether_ros::msg::LocatedVector>::SharedPtr sub_;
  tf2_ros::Buffer::SharedPtr buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
};

class LocatedVectorOriginGenerator : public noether::OriginGenerator
{
public:
  LocatedVectorOriginGenerator();

  Eigen::Vector3d generate(const pcl::PolygonMesh& mesh) const override;

protected:
  noether_ros::msg::LocatedVector::ConstSharedPtr msg_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<noether_ros::msg::LocatedVector>::SharedPtr sub_;
  tf2_ros::Buffer::SharedPtr buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
};

}  // namespace noether_ros

FWD_DECLARE_YAML_CONVERT(noether_ros::LocatedVectorOriginGenerator)
FWD_DECLARE_YAML_CONVERT(noether_ros::LocatedVectorDirectionGenerator)
