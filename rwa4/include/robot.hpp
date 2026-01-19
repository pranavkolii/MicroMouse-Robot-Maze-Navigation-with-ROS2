/**
 * @file robot.hpp
 * @author Akshun Sharma (asharm41@umd.edu), Pranav Koli (pkoli@umd.edu), 
 *         Anish Gupta (agupta96@umd.edu), Sidharth Mathur (sidmat03@umd.edu)
 * @brief Declaration of the Micromouse robot controller.
 * @version 1.0
 * @date 2025-11-29
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

#include <utility>
#include "maze.hpp"
#include "direction.hpp"

class Algorithm;

/**
 * @brief Micromouse Robot controller.
 *
 * Owns a Maze (composition) and holds a non-owning pointer to an Algorithm
 * (aggregation). At runtime we perform an online DFS with full backtracking
 * using the Maze and MazeControlAPI sensors.
 */

class Robot {
public:

    /**
     * @brief Construct a new Robot object
     * 
     * Initializes the internal maze model and sets the robot's
     * starting pose (position and orientation) at the maze origin.
     */

    Robot();

    /**
     * @brief Set the maze-solving algorithm used by the robot.
     * 
     * @param algo Non-owning pointer to pathfinding algorithm
     */

    void setAlgorithm(Algorithm* algo);

    /**
     * @brief Select one of the four center cells as the goal.
     * 
     * Randomly choose one of the four center goals (7,7), (7,8), (8,7), (8,8)
     */

    void selectRandomGoal();

    /**
     * @brief Start online DFS exploration until goal is reached or all reachable cells are explored.
     * 
     */

    void startExploration();

private:

    /**
     * @brief Move exactly one cell to the target 
     * 
     * @param targetX X coordinate of the target neighbor cell.
     * @param targetY Y coordinate of the target neighbor cell.
     * @return true on success, false if a wall (according to sensors) blocks it.
     */

    bool moveOneStep(int targetX, int targetY);

    Maze maze;                       // composition: Robot owns Maze
    Algorithm* algorithm;            // aggregation: non-owning
    int x;
    int y;
    Direction dir;
    std::pair<int,int> selectedGoal; // currently chosen center goal
};