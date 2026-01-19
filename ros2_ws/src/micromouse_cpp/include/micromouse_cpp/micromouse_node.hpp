/**
 * @file micromouse_node.hpp
 * @brief MicroMouse ROS2 Node
 * 
 * Provides:
 * - Action server: /navigate_to_goal
 * - Service: /get_robot_status
 * - Publisher: /robot_positon
 * 
 *  Can run in standalone mode (immediate navigation) or wait for action client.
 */
#pragma once
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <geometry_msgs/msg/point.hpp>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "micromouse_cpp/maze_control_api.hpp"
#include "micromouse_interfaces/action/navigate_to_goal.hpp"
#include "micromouse_interfaces/srv/get_robot_status.hpp"

using NavigateToGoal = micromouse_interfaces::action::NavigateToGoal;
using GoalHandleNavigate = rclcpp_action::ServerGoalHandle<NavigateToGoal>;
using GetRobotStatus = micromouse_interfaces::srv::GetRobotStatus;

namespace micromouse {

/**
 * @brief MicroMouse ROS2 Node
 *
 * Provides:
 * - Action server: /navigate_to_goal
 * - Service: /get_robot_status
 * - Publisher: /robot_position
 *
 * Can run in standalone mode (immediate navigation) or wait for action client.
 */
class MicroMouseNode : public rclcpp::Node {
 public:
  MicroMouseNode();

/**
 * @brief Run navigation immediately (standalone MMS mode)
 */
  void run_navigation();


/**
 * @brief Check if running in standalone mode
 * @return true if standalone mode (immediate navigation), false if waiting
 * for action
 */
  bool is_standalone_mode() const;

 private:
  // Maze dimensions
  int W_{16};
  int H_{16};
  int goal_x_{7};
  int goal_y_{7};

  // Visualization colors
  char path_color_{'c'};
  char goal_color_{'g'};

  // Mode
  bool standalone_mode_{true};

  // Internal wall representation
  std::vector<std::vector<std::array<bool, 4>>> walls_;
  std::set<Cell> explored_cells_;

  // Robot state
  Cell robot_{0, 0};
  Dir facing_{Dir::North};
  int steps_{0};

  // Timing
  std::chrono::steady_clock::time_point nav_start_time_;
  bool is_running_{false};
  std::mutex state_mutex_;

  // ROS2 interfaces
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr robot_position_pub_;
  rclcpp::Service<GetRobotStatus>::SharedPtr status_srv_;
  rclcpp_action::Server<NavigateToGoal>::SharedPtr action_server_;

  // Utility
  void log(const std::string& msg);
  bool in_bounds(const Cell& c) const;

  // Wall management
/**
 * @brief Initialize internal wall storage and set perimeter walls
 */
  void init_walls();


/**
 * @brief Mark wall on both sides (current cell and neighbor)
 */
  void mark_wall(const Cell& c, Dir d);


/**
 * @brief Sense walls and update internal map
 */
  void sense_and_update(const Cell& pos, Dir facing);


/**
 * @brief Check if edge between cell and direction is free (no wall)
 */
  bool edge_free(const Cell& c, Dir d) const;

  // Movement
/**
 * @brief Turn robot to face desired direction
 */
  void face_direction(Dir want);

  // Planning
  /**
   * @brief Compute path using Depth-First Search
   *
   * Uses iterative DFS with a stack. Explores neighbors in order:
   * North, East, South, West. Does NOT guarantee shortest path.
   *
   * @param start Starting cell
   * @param goal Target cell
   * @return Path from start to goal, or nullopt if no path exists
   */
  std::optional<std::vector<Cell>> dfs_plan(const Cell& start,
                                            const Cell& goal);


  // Visualization
  /**
   * @brief Color cells to display travelled path
   * 
   * @param path Travelled path 
   * @param goal Goal Position
   */
  void color_path(const std::vector<Cell>& path, const Cell& goal);


  // Navigation
/**
 * @brief Execute navigation with dynamic replanning
 *
 * Algorithm:
 * 1. Sense walls at current position
 * 2. Plan path using DFS
 * 3. Follow path, sensing before each move
 * 4. If wall blocks path, replan from current position
 * 5. Repeat until goal reached or no path exists
 *
 * @param start Starting cell
 * @param goal Target cell
 * @param goal_handle Optional action goal handle for feedback (nullptr for
 * standalone)
 * @return true if goal reached, false otherwise
 */
  bool execute_with_replanning(
      const Cell& start, const Cell& goal,
      std::shared_ptr<GoalHandleNavigate> goal_handle = nullptr);

  
      // ROS callbacks
  /**
   * @brief ROS callback for publisher that publishes position of the robot 
   * 
   */
  void publish_position();


  /**
   * @brief Get the elapsed seconds object
   * 
   * @return double 
   */
  double get_elapsed_seconds() const;


  /**
   * @brief Get the status callback object
   * 
   * @param request 
   * @param response 
   */
  void get_status_callback(
      const std::shared_ptr<GetRobotStatus::Request> request,
      std::shared_ptr<GetRobotStatus::Response> response);


  /**
   * @brief action callback to handle goal
   * 
   * @param uuid 
   * @param goal 
   * @return rclcpp_action::GoalResponse 
   */
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const NavigateToGoal::Goal> goal);


  /**
   * @brief action callback to handle goal cancellation
   * 
   * @param goal_handle 
   * @return rclcpp_action::CancelResponse 
   */
  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleNavigate> goal_handle);


  /**
   * @brief action callback to handle goal acceptance
   * 
   * @param goal_handle 
   * @return * void 
   */
  void handle_accepted(
      const std::shared_ptr<GoalHandleNavigate> goal_handle);


 /**
  * @brief execute action
  * 
  * @param goal_handle 
  */
  void execute_action(
      const std::shared_ptr<GoalHandleNavigate> goal_handle);
};

}  // namespace micromouse