/**
 * @file bfs_algorithm.hpp
 * @author Akshun Sharma (asharm41@umd.edu), Pranav Koli (pkoli@umd.edu), 
 *         Anish Gupta (agupta96@umd.edu), Sidharth Mathur (sidmat03@umd.edu)
 * @brief Declaration of a stateful breadth-first search maze-solving algorithm.
 * @version 1.0
 * @date 2025-11-29
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

#include "algorithm.hpp"
#include <utility>
#include <vector>

/**
 * @brief forward declaration of class Maze
 * 
 */

class Maze; 

/**
 * @brief Maze-solving algorithm using breadth-first search (BFS).
 *        This class implements the Algorithm interface by performing 
 *        a breadth-first search over the maze grid to find a shortest 
 *        path between a start and goal cell.
 */

class BFSAlgorithm : public Algorithm {
public:

    /**
     * @brief Compute the shortest path from start to goal using BFS.
     *        Explores the maze in a breadth-first manner and returns 
     *        a path with the minimum number of steps from start 
     *        to goal, if such a path exists.
     * 
     * @param maze Reference to the maze model used for planning.
     * @param start Starting cell coordinates (x, y).
     * @param goal Goal cell coordinates (x, y).
     * @return A vector of (x, y) coordinates representing the 
     *         shortest path from start to goal (inclusive), 
               or an empty vector if no path can be found.
     */

    std::vector<std::pair<int,int>> solve(
        const Maze& maze,
        std::pair<int,int> start,
        std::pair<int,int> goal
    ) override;
};
