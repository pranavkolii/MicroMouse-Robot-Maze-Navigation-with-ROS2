/**
 * @file dfs_algorithm.hpp
 * @author Akshun Sharma (asharm41@umd.edu), Pranav Koli (pkoli@umd.edu), 
 *         Anish Gupta (agupta96@umd.edu), Sidharth Mathur (sidmat03@umd.edu)
 * @brief Declaration of a stateful depth-first search maze-solving algorithm.
 * @version 1.0
 * @date 2025-11-29
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#pragma once

#include "algorithm.hpp"
#include "maze.hpp"

#include <vector>
#include <set>
#include <map>
#include <utility>

/**
 * @brief Maze-solving algorithm using a stateful depth-first search (DFS).
 * 
 * The DFSAlgorithm class implements the Algorithm interface by exploring
 * the maze with depth-first search. It maintains internal search state
 * (DFS tree and stack) across successive calls to solve(), allowing the
 * robot to reuse previously explored information as it moves.
 * 
 */

class DFSAlgorithm : public Algorithm {
public:

    /**
     * @brief Alias for a maze coordinate (x, y).
     * 
     */

    using Coord = std::pair<int,int>;

    /**
     * @brief Construct a new DFSAlgorithm object
     * 
     * The internal search state is initialized to an empty DFS tree
     * with no visited cells.
     */

    DFSAlgorithm();

    /**
     * @brief Plan a path from start to goal using DFS.
     * 
     * Uses the current internal DFS tree and stack to continue
     * exploration from the given start cell toward goal.
     * Previously discovered nodes and parent relationships are
     * reused across calls, which allows incremental replanning
     * as the robot uncovers new walls.
     * 
     * @param maze Reference to the maze model used for planning.
     * @param start Starting cell coordinates (x, y).
     * @param goal Goal cell coordinates (x, y).
     * @return A vector of coordinates representing a path from
     *         start to goal (inclusive), or an empty vector if no
     *         path can currently be found.
     */

    std::vector<Coord> solve(
        const Maze& maze,
        Coord start,
        Coord goal
    ) override;
    
    /**
     * @brief Clear all internal DFS state.
     * 
     * Resets the search tree, stack, and visited sets so that the
     * next call to solve() behaves as if no prior exploration had
     * taken place.
     */

    void reset();

    /**
     * @brief Record that the robot has physically visited a cell.
     * 
     * This method is called by the Robot whenever it moves
     * into a new cell. The visited information is used to keep the
     * DFS search tree consistent with the robot's actual traversal.
     * 
     * @param c Coordinate of the cell that the robot has visited.
     */

    void addRobotVisit(const Coord& c);

private:
    bool  initialized_;
    Coord root_;         // root of the global DFS tree
    Coord currentGoal_;  // last goal requested

    // Cells the robot has physically visited (NOT search-only exploration).
    std::set<Coord> robotVisited_;

    // DFS search tree and stack.
    std::map<Coord, Coord> parent_;   // DFS parent tree (root_ as root)
    std::vector<Coord>     stack_;    // DFS stack: path from root_ to current frontier

    /**
     * @brief Rebuild the DFS stack from root_ to the given start cell.
     * 
     * Reconstructs stack_ so that it contains the path [root_, ..., start]
     * using the existing parent_ map. All cells along this path are also
     * inserted into visitedLocal.
     * 
     * @param start Coordinate of the new start cell.
     * @param visitedLocal Local set of visited nodes for the current search.
     */

    void rebuildStackToStart(const Coord& start, std::set<Coord>& visitedLocal);

    /**
     * @brief Build the path from the root to a given node.
     * 
     * Follows parent_ links from node_ back to root_ and returns
     * the resulting sequence of coordinates.
     * @param node Target node for which the path from the root is requested.
     * @return Vector of coordinates from root_ to node, or an empty
     *         vector if node is not present in the DFS tree.
     */

    [[nodiscard]] std::vector<Coord> buildPathRootTo(const Coord& node) const;

    /**
     * @brief Build a path from start to goal using the DFS tree.
     * 
     * Uses the stored DFS tree to construct a path from start to
     * goal, typically by finding the lowest common ancestor (LCA)
     * of the two nodes in the tree and concatenating the corresponding
     * segments.
     * 
     * @param start Starting cell coordinates.
     * @param goal Goal cell coordinates.
     * @return Vector of coordinates representing a path from start
     *         to goal, or an empty vector if either node is not in
     *         the DFS tree or they are disconnected.
     */
    
    [[nodiscard]] std::vector<Coord> buildPathStartToGoal(const Coord& start, const Coord& goal) const;
};