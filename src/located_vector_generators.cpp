#include <noether_ros/located_vector_generators.h>

#include <chrono>
#include <noether_tpp/serialization.h>
#include <tf2_eigen/tf2_eigen.hpp>

using namespace std::chrono_literals;

namespace noether_ros
{
LocatedVectorClient::LocatedVectorClient()
{
  // Initialize RCLPP if not already
  if (!rclcpp::ok())
    rclcpp::init(0, nullptr);

  node_ = std::make_shared<rclcpp::Node>("located_vector_node");

  // Create the client
  client_ = node_->create_client<srv::GetLocatedVector>("get_located_vector");

  // Create the transform listener
  buffer_ =
      std::make_shared<tf2_ros::Buffer>(node_->get_clock(), tf2::Duration(tf2::BUFFER_CORE_DEFAULT_CACHE_TIME), node_);
  listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_, node_);
}

Eigen::Isometry3d LocatedVectorClient::lookupTransform(const std::string& source, const std::string& target) const
{
  rclcpp::spin_some(node_);
  return tf2::transformToEigen(
      buffer_->lookupTransform(source, target, rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0)));
}

LocatedVector LocatedVectorClient::getLocatedVector() const
{
  // Check if the service is ready
  rclcpp::spin_some(node_);
  using namespace std::chrono_literals;
  if (!client_->wait_for_service(1s))
    throw std::runtime_error("Get located vector service is not available");

  // Call the ROI selection service
  auto request = std::make_shared<srv::GetLocatedVector::Request>();
  auto future = client_->async_send_request(request);

  switch (rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(3)))
  {
    case rclcpp::FutureReturnCode::SUCCESS:
      break;
    case rclcpp::FutureReturnCode::TIMEOUT:
      throw std::runtime_error("Service call to '" + std::string(client_->get_service_name()) + "' timed out");
    default:
      throw std::runtime_error("Service call to '" + std::string(client_->get_service_name()) + "' failed");
  }

  srv::GetLocatedVector::Response::ConstSharedPtr response = future.get();
  if (response->success)
    return std::make_pair(response->source, response->target);

  throw std::runtime_error(response->message);
}

Eigen::Vector3d LocatedVectorDirectionGenerator::generate(const pcl::PolygonMesh& mesh) const
{
  LocatedVectorClient client;
  const LocatedVector lv = client.getLocatedVector();

  // Transform the source point into the mesh frame
  Eigen::Vector3d source;
  {
    tf2::fromMsg(lv.first.point, source);
    const Eigen::Isometry3d mesh_to_source = client.lookupTransform(mesh.header.frame_id, lv.first.header.frame_id);
    source = mesh_to_source * source;
  }

  // Transform the target point into the mesh frame
  Eigen::Vector3d target;
  {
    tf2::fromMsg(lv.second.point, target);
    const Eigen::Isometry3d mesh_to_target = client.lookupTransform(mesh.header.frame_id, lv.second.header.frame_id);
    target = mesh_to_target * target;
  }

  return (target - source).normalized();
}

Eigen::Vector3d LocatedVectorOriginGenerator::generate(const pcl::PolygonMesh& mesh) const
{
  LocatedVectorClient client;
  LocatedVector lv = client.getLocatedVector();

  // Convert to Eigen
  Eigen::Vector3d source;
  tf2::fromMsg(lv.first.point, source);

  // Look up the transform from the mesh frame to the source frame
  Eigen::Isometry3d mesh_to_source = client.lookupTransform(mesh.header.frame_id, lv.first.header.frame_id);

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
