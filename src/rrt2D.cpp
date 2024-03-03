#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "TreeNode.h"

class RRT2DNode : public rclcpp::Node
{
public:
  RRT2DNode()
      : Node("RRT2DNode"), start_x(50), start_y(50), goal_x(90), goal_y(90), step_size(1),
        goal_tolerance(1), map_sub_mode(false), obstacle_sub_mode(false), node_limit(10000),
        wall_confidence(50)
  {
    path_publisher = create_publisher<nav_msgs::msg::Path>("path", 10);
    run_rrt();
  }

private:
  const float start_x;
  const float start_y;
  const float goal_x;
  const float goal_y;
  const float step_size;
  const float goal_tolerance;
  const bool map_sub_mode;
  const bool obstacle_sub_mode;
  const size_t node_limit;
  const int wall_confidence;
  std::vector<float> start_position = {start_x, start_y};
  std::vector<float> goal_position = {goal_x, goal_y};
  std::vector<int> map_size = {100, 100};
  TreeNode *root_node = new TreeNode(start_position, nullptr);
  std::vector<TreeNode *> node_list = {root_node};
  nav_msgs::msg::Path path;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher;

  void publish_path()
  {
    path.header.frame_id = "map";
    path.header.stamp = get_clock()->now();
    TreeNode *current_node = node_list.back();
    while (current_node->parent != nullptr)
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = current_node->val[0];
      pose.pose.position.y = current_node->val[1];
      path.poses.push_back(pose);
      current_node = current_node->parent;
    }
    path_publisher->publish(path);
  }

  void run_rrt()
  {
    RCLCPP_INFO(get_logger(), "Running RRT");
    while (node_list.size() < node_limit)
    {
      const float random_position_x = rand() % map_size[0];
      const float random_position_y = rand() % map_size[1];
      float min_distance = INFINITY;
      std::vector<float> random_position = {random_position_x, random_position_y};
      std::vector<float> min_node_vec;
      TreeNode *nearest_node;
      for (TreeNode *node : node_list)
      {
        std::vector<float> goal_vec = {goal_position[0] - node->val[0],
                                       goal_position[1] - node->val[1]};
        const float goal_distance = sqrt(pow(goal_vec[0], 2) + pow(goal_vec[1], 2));
        if (goal_distance < goal_tolerance)
        {
          TreeNode *goal_node = new TreeNode(goal_position, node);
          node->add_child(goal_node);
          node_list.push_back(goal_node);
          RCLCPP_INFO(get_logger(), "Path found");
          publish_path();
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