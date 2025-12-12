#include <noether_ros/located_vector_generators.h>

#include <chrono>
#include <noether_tpp/serialization.h>
#include <tf2_eigen/tf2_eigen.hpp>

using namespace std::chrono_literals;

namespace noether_ros
{
LocatedVectorSubscriber::LocatedVectorSubscriber()
{
  // Initialize RCLPP if not already
  if (!rclcpp::ok())
    rclcpp::init(0, nullptr);

  node_ = std::make_shared<rclcpp::Node>("located_vector_direction_generator");

  // Create the subscriber
  rclcpp::QoS qos(1);
  qos.reliable();
  qos.transient_local();

  auto cb = [this](const noether_ros::msg::LocatedVector::ConstSharedPtr& msg) {
    std::lock_guard lock{ mutex_ };
    msg_ = msg;
  };
  sub_ = node_->create_subscription<noether_ros::msg::LocatedVector>("located_vector", qos, cb);

  // Create the transform listener
  buffer_ =
      std::make_shared<tf2_ros::Buffer>(node_->get_clock(), tf2::Duration(tf2::BUFFER_CORE_DEFAULT_CACHE_TIME), node_);
  listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_, node_);
}

Eigen::Isometry3d LocatedVectorSubscriber::lookupTransform(const std::string& source, const std::string& target) const
{
  rclcpp::spin_some(node_);
  return tf2::transformToEigen(
      buffer_->lookupTransform(source, target, rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0)));
}

std::optional<noether_ros::msg::LocatedVector> LocatedVectorSubscriber::getMessage() const
{
  auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < 3s)
  {
    rclcpp::spin_some(node_);
    std::lock_guard lock{ mutex_ };
    if (msg_)
      return *msg_;
    std::this_thread::sleep_for(100ms);
  }
  return {};
}

Eigen::Vector3d LocatedVectorDirectionGenerator::generate(const pcl::PolygonMesh& mesh) const
{
  LocatedVectorSubscriber sub_;
  const std::optional<msg::LocatedVector> msg = sub_.getMessage();

  if (!msg)
    throw std::runtime_error("No located vector message received yet");

  // Transform the source point into the mesh frame
  Eigen::Vector3d source;
  {
    tf2::fromMsg(msg.value().source.point, source);
    const Eigen::Isometry3d mesh_to_source =
        sub_.lookupTransform(mesh.header.frame_id, msg.value().source.header.frame_id);
    source = mesh_to_source * source;
  }

  // Transform the target point into the mesh frame
  Eigen::Vector3d target;
  {
    tf2::fromMsg(msg.value().target.point, target);
    const Eigen::Isometry3d mesh_to_target =
        sub_.lookupTransform(mesh.header.frame_id, msg.value().target.header.frame_id);
    target = mesh_to_target * target;
  }

  return (target - source).normalized();
}

Eigen::Vector3d LocatedVectorOriginGenerator::generate(const pcl::PolygonMesh& mesh) const
{
  LocatedVectorSubscriber sub_;
  std::optional<msg::LocatedVector> msg = sub_.getMessage();

  if (!msg)
    throw std::runtime_error("No located vector message received yet");

  // Convert to Eigen
  Eigen::Vector3d source;
  tf2::fromMsg(msg.value().source.point, source);

  // Look up the transform from the mesh frame to the source frame
  Eigen::Isometry3d mesh_to_source = sub_.lookupTransform(mesh.header.frame_id, msg.value().source.header.frame_id);

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
