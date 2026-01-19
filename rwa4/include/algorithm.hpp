/**
 * @file algorithm.hpp
 * @author Akshun Sharma (asharm41@umd.edu), Pranav Koli (pkoli@umd.edu), 
 *         Anish Gupta (agupta96@umd.edu), Sidharth Mathur (sidmat03@umd.edu)
 * @brief Abstract base class for maze-solving algorithms
 * @version 1.0
 * @date 2025-11-29
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

#include <utility>
#include <vector>

class Maze;
/**
 * @brief  This class defines the common interface used by all maze-solving 
 * strategies (DFS & BFS). Concrete algorithms must implement the solve() 
 * function to compute a path between two cells in a given maze.
 * 
 */

class Algorithm {
public:

    /**
     * @brief Destroy the Algorithm object (Virtual destructor for safe polymorphic deletion.)
     * 
     */

    virtual ~Algorithm() = default;
    
    /**
     * @brief Compute a path from start to goal in the given maze.
     * 
     * Implementations should return a sequence of grid cells
     * representing a path from start to goal (inclusive).
     * If no path exists, the returned vector should be empty.
     * 
     * @param maze Reference to the maze model used for planning.
     * @param start Starting cell coordinates (x, y).
     * @param goal Goal cell coordinates (x, y).
     * @return Return a sequence of (x,y) cells from start to goal, or an empty vector if no path is found.
     */

    virtual std::vector<std::pair<int,int>> solve(
        const Maze& maze,
        std::pair<int,int> start,
        std::pair<int,int> goal
    ) = 0;
};