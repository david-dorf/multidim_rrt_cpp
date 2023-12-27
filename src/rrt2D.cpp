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

TreeNode::TreeNode(const std::vector<float> &val, std::shared_ptr<TreeNode> parent)
    : val(val), parent(parent) {}

class RRT2DNode : public rclcpp::Node
{
public:
  RRT2DNode()
      : Node("RRT2DNode"), start_x(0), start_y(0), goal_x(1),
        goal_y(-1), map_sub_mode(false), obstacle_sub_mode(false),
        step_size(0.1), node_limit(10000), goal_tolerance(0.1),
        wall_confidence(0), count_(0)
  {
    marker_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("rrt_markers", 10);
    path_publisher = this->create_publisher<nav_msgs::msg::Path>("rrt_path", 10);
    // occupancy_grid_subscription = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    //     "map", 10, std::bind(&RRT2DNode::occupancyGridCallback, this, std::placeholders::_1));
    // obstacle_subscription = this->create_subscription<visualization_msgs::msg::MarkerArray>(
    //     "obstacles", 10, std::bind(&RRT2DNode::obstacleCallback, this, std::placeholders::_1));
    // timer_ = this->create_wall_timer(
    //     500ms, std::bind(&RRT2DNode::timerCallback, this));
  }

private:
  float start_x;
  float start_y;
  float goal_x;
  float goal_y;
  bool map_sub_mode;
  bool obstacle_sub_mode;
  float step_size;
  size_t node_limit;
  float goal_tolerance;
  int wall_confidence;
  std::vector<float> start_position = {start_x, start_y};
  std::vector<float> goal_position = {goal_x, goal_y};
  std::vector<int> map_size = {100, 100};
  std::shared_ptr<TreeNode> root_node = std::make_shared<TreeNode>(start_position, nullptr);
  std::vector<std::shared_ptr<TreeNode>> node_list = {root_node};
  std::shared_ptr<TreeNode> goal_node = std::make_shared<TreeNode>(goal_position, nullptr);

  // ROS Objects
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_subscription;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_subscription;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;

  void runRRT()
  {
    bool completed = false;
    while (node_list.size() < node_limit)
    {
      float random_position_x = static_cast<float>(rand()) / static_cast<float>(RAND_MAX / (map_size[0] * step_size));
      float random_position_y = static_cast<float>(rand()) / static_cast<float>(RAND_MAX / (map_size[1] * step_size));
      std::vector<float> random_position = {random_position_x, random_position_y};
      float min_distance = INFINITY;
      std::vector<float> min_node_vec;
      std::shared_ptr<TreeNode> nearest_node;
      for (auto node : node_list)
      {
        std::vector<float> goal_vec = {goal_position[0] - node->val[0], goal_position[1] - node->val[1]};
        float goal_distance = sqrt(pow(goal_vec[0], 2) + pow(goal_vec[1], 2));
        if (goal_distance < goal_tolerance)
        {
          node->addChild(goal_node);
          node_list.push_back(goal_node);
          completed = true;
          break;
        }
        std::vector<float> node_vec = {node->val[0] - random_position[0], node->val[1] - random_position[1]};
        float distance = sqrt(pow(node_vec[0], 2) + pow(node_vec[1], 2));
        if (distance < min_distance)
        {
          min_node_vec = node_vec;
          min_distance = distance;
          nearest_node = node;
        }
        if (min_distance != 0)
        {
          std::vector<float> new_node_position = {nearest_node->val[0] - (min_node_vec[0] / min_distance) * step_size,
                                                  nearest_node->val[1] - (min_node_vec[1] / min_distance) * step_size};
          std::shared_ptr<TreeNode> new_node = std::make_shared<TreeNode>(new_node_position, nearest_node);
          nearest_node->addChild(new_node);
          node_list.push_back(new_node);
        }
      }
    }
    if (completed)
    {
      RCLCPP_INFO(this->get_logger(), "Path found");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Path not found");
    }
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RRT2DNode>());
  rclcpp::shutdown();
  return 0;
}