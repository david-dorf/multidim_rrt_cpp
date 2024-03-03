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
      : Node("RRT2DNode"), start_x(0), start_y(0), goal_x(10),
        goal_y(10), map_sub_mode(false), obstacle_sub_mode(false),
        step_size(1.0), node_limit(10000), goal_tolerance(10.0),
        wall_confidence(0)
  {
    marker_publisher = create_publisher<visualization_msgs::msg::MarkerArray>("rrt_markers", 10);
    runRRT();
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
  TreeNode *root_node = new TreeNode(start_position, nullptr);
  TreeNode *goal_node = new TreeNode(goal_position, nullptr);
  std::vector<TreeNode *> node_list = {root_node};
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher;

  void runRRT()
  {
    RCLCPP_INFO(get_logger(), "Running RRT");
    while (node_list.size() < node_limit)
    {
      float random_position_x = rand() % map_size[0];
      float random_position_y = rand() % map_size[1];
      std::vector<float> random_position = {random_position_x, random_position_y};
      float min_distance = INFINITY;
      std::vector<float> min_node_vec;
      TreeNode *nearest_node = nullptr;
      for (TreeNode *node : node_list)
      {
        std::vector<float> goal_vec = {goal_position[0] - node->val[0],
                                       goal_position[1] - node->val[1]};
        const float goal_distance = sqrt(pow(goal_vec[0], 2) + pow(goal_vec[1], 2));
        if (goal_distance < goal_tolerance)
        {
          node->add_child(goal_node);
          node_list.push_back(goal_node);
          RCLCPP_INFO(get_logger(), "Path found");
          return;
        }
        std::vector<float> node_vec = {node->val[0] - random_position[0],
                                       node->val[1] - random_position[1]};
        const float distance = sqrt(pow(node_vec[0], 2) + pow(node_vec[1], 2));
        if (distance < min_distance)
        {
          min_node_vec = node_vec;
          min_distance = distance;
          nearest_node = node;
        }
      }
      const float new_node_x = nearest_node->val[0] - (min_node_vec[0] / min_distance) * step_size;
      const float new_node_y = nearest_node->val[1] - (min_node_vec[1] / min_distance) * step_size;
      std::vector<float> new_node_position = {new_node_x, new_node_y};
      TreeNode *new_node = new TreeNode(new_node_position, nearest_node);
      nearest_node->add_child(new_node);
      node_list.push_back(new_node);
    }
    RCLCPP_INFO(get_logger(), "Path not found");
    return;
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RRT2DNode>());
  rclcpp::shutdown();
  return 0;
}