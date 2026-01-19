#include <iostream>
#include <string>

#include "robot.hpp"
#include "dfs_algorithm.hpp"
#include "bfs_algorithm.hpp"

#include "maze_api.hpp"

using MMS = micro_mouse::MazeControlAPI;

static void log(const std::string& text) {
  std::cerr << "[MAIN] " << text << std::endl;
}

int main() {
  log("Micromouse DFS solver starting.");

  // Clear any previous run visualization
  MMS::clear_all_color();
  MMS::clear_all_text();

  // Mark start cell
  MMS::set_color(0, 0, 'G');
  MMS::set_text(0, 0, "S");

  // Mark four center goal cells (visual only)
  MMS::set_text(7, 7, "(7,7)");
  MMS::set_text(7, 8, "(7,8)");
  MMS::set_text(8, 7, "(8,7)");
  MMS::set_text(8, 8, "(8,8)");
  MMS::set_color(7, 7, 'y');
  MMS::set_color(7, 8, 'y');
  MMS::set_color(8, 7, 'y');
  MMS::set_color(8, 8, 'y');
  log("Start and center goal cells marked.");

  Robot robot;

  // Create DFS algorithm and robot
  DFSAlgorithm dfs_algo;
  robot.setAlgorithm(&dfs_algo);

  /**
   * @brief uncomment lines 49 and 50 and comment lines 41 & 42 for BFS algorithm implementation 
   * 
   */

  // BFSAlgorithm bfs_algo;
  // robot.setAlgorithm(&bfs_algo);

  log("Robot and Algorithm initialized and linked.");

  // Handle reset once before exploration
  if (MMS::was_reset()) {
    log("Simulator reports reset = true; acknowledging...");
    MMS::ack_reset();
    log("Reset acknowledged.");
  } else {
    log("Simulator reports reset = false; continuing.");
  }

  log("Starting exploration to center goal...");
  try {
    robot.startExploration();
    log("Exploration completed.");
  } catch (const std::exception& ex) {
    log(std::string("Exception during exploration: ") + ex.what());
  } catch (...) {
    log("Unknown exception during exploration.");
  }

  log("Micromouse program finished. Exiting.");
  return 0;
}