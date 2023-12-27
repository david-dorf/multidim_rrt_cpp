#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "Marker.h"
#include "TreeNode.h"

using namespace std::chrono_literals;

class RRT2DNode : public rclcpp::Node
{
public:
  RRT2DNode()
      : Node("RRT2DNode"), start_x(0), start_y(0), goal_x(0),
        goal_y(0), map_sub_mode(false), obstacle_sub_mode(false),
        step_size(0.1), node_limit(10000), goal_tolerance(0.1),
        wall_confidence(0), count_(0)
  {
    marker_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("rrt_markers", 10);
    path_publisher = this->create_publisher<nav_msgs::msg::Path>("rrt_path", 10);
  }

private:
  float start_x;
  float start_y;
  float goal_x;
  float goal_y;
  bool map_sub_mode;
  bool obstacle_sub_mode;
  float step_size;
  int node_limit;
  float goal_tolerance;
  int wall_confidence;
  // void timer_callback()
  // {
  // }
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_subscription;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RRT2DNode>());
  rclcpp::shutdown();
  return 0;
}