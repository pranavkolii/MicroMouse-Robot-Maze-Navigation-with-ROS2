/**
 * @file cell.hpp
 * @author Akshun Sharma (asharm41@umd.edu), Pranav Koli (pkoli@umd.edu), 
 *         Anish Gupta (agupta96@umd.edu), Sidharth Mathur (sidmat03@umd.edu)
 * @brief Declaration of the Cell class used to store wall information for a maze cell.
 * @version 1.0
 * @date 2025-11-29
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#pragma once
#include <stdexcept>

/**
 * @brief Represents a single cell in the maze.
 * 
 * Each cell stores whether there is a wall on its four sides
 * (north, south, east, west). The cell provides methods to
 * mark walls and to query the presence of a wall in a given
 * direction.
 */

class Cell {
private:
    bool wallN = false;
    bool wallS = false;
    bool wallE = false;
    bool wallW = false;

public:

     /**
      * @brief Construct a new Cell object
      * 
      */

    Cell() = default;

    /**
     * @brief Mark a wall on the given side of the cell.
     * 
     *   The direction character is interpreted case-insensitively:
     * - 'n' or 'N' for the north wall
     * - 's' or 'S' for the south wall
     * - 'e' or 'E' for the east wall
     * - 'w' or 'W' for the west wall
     * 
     * @param dir Direction character specifying which wall to mark.
     */

    void markWall(char dir) noexcept;

    /**
     * @brief Check whether a wall exists on the given side of the cell.
     * 
     * The direction character is interpreted case-insensitively using
     * the same mapping as markWall().
     * 
     * @param dir Direction character specifying which wall to query.
     * @return true if a wall is present on the requested side, false otherwise.
     */
    
    [[nodiscard]] bool hasWall(char dir) const noexcept;
};