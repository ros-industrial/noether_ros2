#include <noether_ros/located_vector_generators.h>

#include <noether_tpp/serialization.h>
#include <tf2_eigen/tf2_eigen.hpp>

namespace noether_ros
{
LocatedVectorDirectionGenerator::LocatedVectorDirectionGenerator() : msg_(nullptr)
{
  // Initialize RCLPP if not already
  if (!rclcpp::ok())
    rclcpp::init(0, nullptr);

  node_ = std::make_shared<rclcpp::Node>("located_vector_direction_generator");

  // Create the subscriber
  rclcpp::QoS qos(1);
  qos.reliable();
  qos.transient_local();

  auto cb = [this](const noether_ros::msg::LocatedVector::ConstSharedPtr& msg) { msg_ = msg; };
  sub_ = node_->create_subscription<noether_ros::msg::LocatedVector>("located_vector", qos, cb);

  // Create the transform listener
  buffer_ =
      std::make_shared<tf2_ros::Buffer>(node_->get_clock(), tf2::Duration(tf2::BUFFER_CORE_DEFAULT_CACHE_TIME), node_);
  listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
}

Eigen::Vector3d LocatedVectorDirectionGenerator::generate(const pcl::PolygonMesh& mesh) const
{
  if (msg_ == nullptr)
    throw std::runtime_error("No located vector message received yet");

  // Transform the source point into the mesh frame
  Eigen::Vector3d source;
  {
    tf2::fromMsg(msg_->source.point, source);
    Eigen::Isometry3d mesh_to_source = tf2::transformToEigen(buffer_->lookupTransform(
        mesh.header.frame_id, msg_->source.header.frame_id, rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0)));
    source = mesh_to_source * source;
  }

  // Transform the target point into the mesh frame
  Eigen::Vector3d target;
  {
    tf2::fromMsg(msg_->target.point, target);
    Eigen::Isometry3d mesh_to_target = tf2::transformToEigen(buffer_->lookupTransform(
        mesh.header.frame_id, msg_->target.header.frame_id, rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0)));
    target = mesh_to_target * target;
  }

  return (target - source).normalized();
}

LocatedVectorOriginGenerator::LocatedVectorOriginGenerator() : msg_(nullptr)
{
  // Initialize RCLPP if not already
  if (!rclcpp::ok())
    rclcpp::init(0, nullptr);

  node_ = std::make_shared<rclcpp::Node>("located_vector_origin_generator");

  // Create the subscriber
  rclcpp::QoS qos(1);
  qos.reliable();
  qos.transient_local();

  auto cb = [this](const noether_ros::msg::LocatedVector::ConstSharedPtr& msg) { msg_ = msg; };
  sub_ = node_->create_subscription<noether_ros::msg::LocatedVector>("located_vector", qos, cb);

  // Create the transform listener
  buffer_ =
      std::make_shared<tf2_ros::Buffer>(node_->get_clock(), tf2::Duration(tf2::BUFFER_CORE_DEFAULT_CACHE_TIME), node_);
  listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
}

Eigen::Vector3d LocatedVectorOriginGenerator::generate(const pcl::PolygonMesh& mesh) const
{
  if (msg_ == nullptr)
    throw std::runtime_error("No located vector message received yet");

  // Convert to Eigen
  Eigen::Vector3d source;
  tf2::fromMsg(msg_->source.point, source);

  // Look up the transform from the mesh frame to the source frame
  Eigen::Isometry3d mesh_to_source = tf2::transformToEigen(buffer_->lookupTransform(
      mesh.header.frame_id, msg_->source.header.frame_id, rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0)));

  // Transform the source point into the mesh frame
  return mesh_to_source * source;
}

}  // namespace noether_ros

namespace YAML
{
/** @cond */
Node convert<noether_ros::LocatedVectorDirectionGenerator>::encode(
    const noether_ros::LocatedVectorDirectionGenerator& val)
{
  return {};
}

bool convert<noether_ros::LocatedVectorDirectionGenerator>::decode(const Node& node,
                                                                   noether_ros::LocatedVectorDirectionGenerator& val)
{
  return true;
}

Node convert<noether_ros::LocatedVectorOriginGenerator>::encode(const noether_ros::LocatedVectorOriginGenerator& val)
{
  return {};
}

bool convert<noether_ros::LocatedVectorOriginGenerator>::decode(const Node& node,
                                                                noether_ros::LocatedVectorOriginGenerator& val)
{
  return true;
}
/** @endcond */
}  // namespace YAML
