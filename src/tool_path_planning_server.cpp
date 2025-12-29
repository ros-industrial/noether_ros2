#include <noether_ros/conversions.h>
#include <noether_ros/srv/plan_tool_path.hpp>

#include <noether_tpp/core/tool_path_planner_pipeline.h>
#include <noether_tpp/utils.h>
#include <noether_tpp/plugin_interface.h>

#include <pcl/io/vtk_lib_io.h>

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <filesystem>

class ToolPathPlanningServer : public rclcpp::Node
{
public:
  ToolPathPlanningServer() : Node("tool_path_planning_server")
  {
    RCLCPP_INFO(get_logger(), "[INIT] Creating tool_path_planning_server service");
    server_ = this->create_service<noether_ros::srv::PlanToolPath>(
        "plan_tool_path",
        std::bind(&ToolPathPlanningServer::callback, this, std::placeholders::_1, std::placeholders::_2));
  }

protected:
  void callback(const std::shared_ptr<noether_ros::srv::PlanToolPath::Request> request,
                std::shared_ptr<noether_ros::srv::PlanToolPath::Response> response)
  {
    auto t_start = std::chrono::steady_clock::now();

    RCLCPP_INFO(get_logger(), "-----------------------------------------------------");
    RCLCPP_INFO(get_logger(), "[CALLBACK] Received plan_tool_path request");
    RCLCPP_INFO(get_logger(), "  mesh_file   : %s", request->mesh_file.c_str());
    RCLCPP_INFO(get_logger(), "  mesh_frame  : %s", request->mesh_frame.c_str());
    RCLCPP_INFO(get_logger(), "  config (raw): %s", request->config.c_str());
    RCLCPP_INFO(get_logger(), "-----------------------------------------------------");

    try
    {
      RCLCPP_INFO(get_logger(), "[CONFIG] Parsing configuration");

      YAML::Node config;

      // Try file first
      try
      {
        if (std::filesystem::exists(request->config))
        {
          RCLCPP_INFO(get_logger(), "[CONFIG] Loading config from file: %s", request->config.c_str());
          config = YAML::LoadFile(request->config);
        }
        else
        {
          throw std::runtime_error("config path does not exist, falling back to parsing raw string");
        }
      }
      catch (...)
      {
        RCLCPP_WARN(get_logger(), "[CONFIG] Failed to load as file — parsing as raw YAML string");
        try
        {
          config = YAML::Load(request->config);
        }
        catch (const YAML::Exception& e)
        {
          throw std::runtime_error(std::string("Failed to parse YAML config: ") + e.what());
        }
      }

      RCLCPP_INFO(get_logger(), "[CONFIG] Successfully loaded YAML config");

      // Print top-level YAML keys
      if (config.IsMap())
      {
        RCLCPP_INFO(get_logger(), "[CONFIG] Top-level keys:");
        for (auto it : config)
        {
          RCLCPP_INFO(get_logger(), "  - %s", it.first.as<std::string>().c_str());
        }
      }
      else
      {
        RCLCPP_WARN(get_logger(), "[CONFIG] YAML root is not a map!");
      }

      // ---------------------------------------------------------------------
      // Load mesh
      // ---------------------------------------------------------------------
      RCLCPP_INFO(get_logger(), "[MESH] Attempting to load mesh from file: %s", request->mesh_file.c_str());

      if (!std::filesystem::exists(request->mesh_file))
      {
        throw std::runtime_error("Mesh file does not exist: " + request->mesh_file);
      }

      pcl::PolygonMesh mesh;
      mesh.header.frame_id = request->mesh_frame;

      int load_result = pcl::io::loadPolygonFile(request->mesh_file, mesh);
      RCLCPP_INFO(get_logger(), "[MESH] pcl::io::loadPolygonFile result = %d", load_result);

      if (load_result <= 0)
        throw std::runtime_error("Failed to load mesh from file: " + request->mesh_file);

      // Print mesh stats
      RCLCPP_INFO(get_logger(), "[MESH] Mesh successfully loaded");
      RCLCPP_INFO(get_logger(), "[MESH]   #Polygons : %zu", mesh.polygons.size());
      RCLCPP_INFO(get_logger(), "[MESH]   Data size : %zu bytes", mesh.cloud.data.size());

      // ---------------------------------------------------------------------
      // Factory and pipeline construction
      // ---------------------------------------------------------------------
      RCLCPP_INFO(get_logger(), "[FACTORY] Building plugin factory");
      noether::Factory factory;

      // List registered plugin types
      RCLCPP_INFO(get_logger(), "[FACTORY] Factory created (cannot list plugins in this version)");

      RCLCPP_INFO(get_logger(), "[PIPELINE] Creating ToolPathPlannerPipeline");
      noether::ToolPathPlannerPipeline pipeline(factory, config);

      RCLCPP_INFO(get_logger(), "[PIPELINE] Pipeline created (cannot list stages in this version)");

      // ---------------------------------------------------------------------
      // Run the pipeline
      // ---------------------------------------------------------------------
      RCLCPP_INFO(get_logger(), "[PIPELINE] Starting planning...");

      auto t_plan_start = std::chrono::steady_clock::now();

      std::vector<noether::ToolPaths> tool_paths = pipeline.plan(mesh);

      auto t_plan_end = std::chrono::steady_clock::now();
      double plan_ms = std::chrono::duration<double, std::milli>(t_plan_end - t_plan_start).count();

      RCLCPP_INFO(get_logger(), "[PIPELINE] Planning complete in %.2f ms", plan_ms);
      RCLCPP_INFO(get_logger(), "[PIPELINE] #ToolPaths groups = %zu", tool_paths.size());

      for (size_t i = 0; i < tool_paths.size(); ++i)
      {
        RCLCPP_INFO(get_logger(), "  Group %zu: contains %zu paths", i, tool_paths[i].size());
      }

      // ---------------------------------------------------------------------
      // Fill the response
      // ---------------------------------------------------------------------
      RCLCPP_INFO(get_logger(), "[RESPONSE] Converting tool paths to ROS message");

      response->success = true;
      response->tool_paths = noether_ros::toMsg(tool_paths, request->mesh_frame);

      RCLCPP_INFO(get_logger(), "[RESPONSE] Success = true");
    }
    catch (const std::exception& ex)
    {
      RCLCPP_ERROR(get_logger(), "[ERROR] Exception caught during planning:");
      RCLCPP_ERROR(get_logger(), "  %s", ex.what());

      response->success = false;

      std::stringstream ss;
      noether::printException(ex, ss);
      response->message = ss.str();

      RCLCPP_ERROR(get_logger(), "[RESPONSE] Returning error message");
    }

    auto t_end = std::chrono::steady_clock::now();
    double total_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
    RCLCPP_INFO(get_logger(), "[CALLBACK] Finished request in %.2f ms", total_ms);
    RCLCPP_INFO(get_logger(), "-----------------------------------------------------");
  }

  rclcpp::Service<noether_ros::srv::PlanToolPath>::SharedPtr server_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ToolPathPlanningServer>();
  RCLCPP_INFO_STREAM(node->get_logger(), "Started tool_path_planning_server");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
