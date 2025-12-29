#include <noether_ros/located_vector_generators.h>
#include <noether_ros/srv/get_located_vector.hpp>

#include <chrono>
#include <noether_tpp/serialization.h>
#include <rclcpp/node.hpp>
#include <rclcpp/service.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

static const char* SERVICE_NAME_KEY = "service_name";
static const char* DEFAULT_SERVICE_NAME = "get_located_vector";

namespace noether_ros
{
using LocatedVector = std::pair<geometry_msgs::msg::PointStamped, geometry_msgs::msg::PointStamped>;

class LocatedVectorClient
{
public:
  LocatedVectorClient(const std::string& service_name)
  {
    // Initialize RCLPP if not already
    if (!rclcpp::ok())
      rclcpp::init(0, nullptr);

    node_ = std::make_shared<rclcpp::Node>("located_vector_client_node");

    // Create the client
    client_ = node_->create_client<srv::GetLocatedVector>(service_name);

    // Create the transform listener
    buffer_ = std::make_shared<tf2_ros::Buffer>(
        node_->get_clock(), tf2::Duration(tf2::BUFFER_CORE_DEFAULT_CACHE_TIME), node_);
    listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_, node_);
  }

  virtual ~LocatedVectorClient() = default;

  Eigen::Isometry3d lookupTransform(const std::string& source, const std::string& target) const
  {
    rclcpp::spin_some(node_);
    return tf2::transformToEigen(
        buffer_->lookupTransform(source, target, rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0)));
  }

  LocatedVector getLocatedVector() const
  {
    // Check if the service is ready
    rclcpp::spin_some(node_);
    using namespace std::chrono_literals;
    if (!client_->wait_for_service(1s))
    {
      std::stringstream ss;
      ss << "Service '" << client_->get_service_name() << "' is not available";
      throw std::runtime_error(ss.str());
    }

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

protected:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<srv::GetLocatedVector>::SharedPtr client_;
  tf2_ros::Buffer::SharedPtr buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
};

LocatedVectorDirectionGenerator::LocatedVectorDirectionGenerator(const std::string& service_name)
  : service_name_(service_name)
{
}

Eigen::Vector3d LocatedVectorDirectionGenerator::generate(const pcl::PolygonMesh& mesh) const
{
  LocatedVectorClient client(service_name_);
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

LocatedVectorOriginGenerator::LocatedVectorOriginGenerator(const std::string& service_name)
  : service_name_(service_name)
{
}

Eigen::Vector3d LocatedVectorOriginGenerator::generate(const pcl::PolygonMesh& mesh) const
{
  LocatedVectorClient client(service_name_);
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
  Node node;
  node[SERVICE_NAME_KEY] = val.service_name_;
  return node;
}

bool convert<noether_ros::LocatedVectorDirectionGenerator>::decode(const Node& node,
                                                                   noether_ros::LocatedVectorDirectionGenerator& val)
{
  if (node[SERVICE_NAME_KEY])
    val.service_name_ = getMember<std::string>(node, SERVICE_NAME_KEY);
  else
    val.service_name_ = DEFAULT_SERVICE_NAME;
  return true;
}

Node convert<noether_ros::LocatedVectorOriginGenerator>::encode(const noether_ros::LocatedVectorOriginGenerator& val)
{
  Node node;
  node[SERVICE_NAME_KEY] = val.service_name_;
  return node;
}

bool convert<noether_ros::LocatedVectorOriginGenerator>::decode(const Node& node,
                                                                noether_ros::LocatedVectorOriginGenerator& val)
{
  if (node[SERVICE_NAME_KEY])
    val.service_name_ = getMember<std::string>(node, SERVICE_NAME_KEY);
  else
    val.service_name_ = DEFAULT_SERVICE_NAME;
  return true;
}
/** @endcond */
}  // namespace YAML
