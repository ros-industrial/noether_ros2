#include <noether_ros/conversions.h>
#include <noether_ros/srv/plan_tool_path.hpp>

#include <filesystem>
#include <noether_tpp/core/tool_path_planner_pipeline.h>
#include <noether_tpp/utils.h>
#include <noether_tpp/plugin_interface.h>
#include <pcl/io/vtk_lib_io.h>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

class ToolPathPlanningServer : public rclcpp::Node
{
public:
  ToolPathPlanningServer() : Node("tool_path_planning_server")
  {
    server_ = this->create_service<noether_ros::srv::PlanToolPath>(
        "plan_tool_path",
        std::bind(&ToolPathPlanningServer::callback, this, std::placeholders::_1, std::placeholders::_2));
  }

protected:
  void callback(const std::shared_ptr<noether_ros::srv::PlanToolPath::Request> request,
                std::shared_ptr<noether_ros::srv::PlanToolPath::Response> response)
  {
    try
    {
      // Convert input configuration string/file to YAML node
      // Load the configuration directly as a YAML-formatted string
      YAML::Node config = YAML::Load(request->config);

      // If the config is a scalar (and not a map), it's likely that the config is actually a file path and not a
      // YAML-formatted string
      if (config.IsScalar())
      {
        // Check if the configuration is an existing file
        if (std::filesystem::is_regular_file(request->config) || std::filesystem::is_symlink(request->config))
        {
          // Reload the configuration from file
          config = YAML::LoadFile(request->config);
        }
        else
        {
          throw std::runtime_error("The request TPP configuration looks like a file path but does not exist: '" +
                                   request->config + "'");
        }
      }

      // Check that the loaded request configuration is a map type
      if (!config.IsMap())
        throw std::runtime_error("The request TPP configuration must be a map type: '" + request->config + "'");

      // Load the mesh file
      pcl::PolygonMesh mesh;
      if (pcl::io::loadPolygonFile(request->mesh_file, mesh) <= 0)
        throw std::runtime_error("Failed to load mesh from file: " + request->mesh_file);

      // Set the mesh frame from the request
      mesh.header.frame_id = request->mesh_frame;

      // Create and run the tool path planning pipeline
      noether::Factory factory;
      noether::ToolPathPlannerPipeline pipeline(factory, config);
      std::vector<noether::ToolPaths> tool_paths = pipeline.plan(mesh);

      // Fill in the response
      response->success = true;
      response->tool_paths = noether_ros::toMsg(tool_paths, request->mesh_frame);
    }
    catch (const std::exception& ex)
    {
      response->success = false;
      std::stringstream ss;
      noether::printException(ex, ss);
      response->message = ss.str();
    }
  }

  rclcpp::Service<noether_ros::srv::PlanToolPath>::SharedPtr server_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ToolPathPlanningServer>();
  RCLCPP_INFO_STREAM(node->get_logger(), "Started tool path planning server");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
