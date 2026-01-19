/**
 * @file maze.hpp
 * @author Akshun Sharma (asharm41@umd.edu), Pranav Koli (pkoli@umd.edu), 
 *         Anish Gupta (agupta96@umd.edu), Sidharth Mathur (sidmat03@umd.edu)
 * @brief Internal maze representation and simulator wrapper.
 * @version 1.0
 * @date 2025-11-29
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once
#include <vector>
#include <string>
#include <utility>
#include "cell.hpp"
#include "maze_api.hpp"  

/**
 * @brief Internal model of the micromouse maze.
 * 
 * The Maze class stores a grid of Cell objects and provides a high-level
 * interface for querying and updating walls, sensing the environment,
 * and controlling visualization. All interaction with the underlying
 * simulator API is funneled through this class.
 */

class Maze {
private:
    int width;
    int height;
    std::vector<std::vector<Cell>> grid;

public:

    /**
     * @brief Construct a new Maze object
     * 
     * The constructor queries the simulator for the maze width and height
     * and allocates the corresponding grid of Cell objects.
     * 
     */

    Maze();

    /**
     * @brief Check whether coordinates are inside the maze bounds.
     * 
     * @param x X coordinate of the cell.
     * @param y Y coordinate of the cell.
     * @return true if (x, y) lies within the maze grid, false otherwise.
     */

    [[nodiscard]] bool inBounds(int x, int y) const noexcept;

    /**
     * @brief Query whether a wall exists on a given side of a cell.
     * 
     * @param x X coordinate of the cell.
     * @param y Y coordinate of the cell.
     * @param dir Direction character ('n', 'e', 's', or 'w').
     * @return true if a wall is present in the requested direction, false otherwise.
     */

    [[nodiscard]] bool hasWall(int x, int y, char dir) const noexcept;

    /**
     * @brief Set the Wall object
     * 
     * Updates the corresponding Cell locally and calls the API
     * to synchronize the wall state visually.
     * @param x X coordinate of the cell.
     * @param y Y coordinate of the cell.
     * @param dir Direction character ('n', 'e', 's', or 'w').
     */

    void setWall(int x, int y, char dir);

    /**
    * @brief Clear a wall at the specified position and direction
    * 
    * @param x X coordinate of the cell
    * @param y Y coordinate of the cell
    * @param direction Direction of the wall ('n', 's', 'e', 'w' for north, south, east, west)
    */

    void clearWall(int x, int y, char dir);
    /**
     * @brief Update local wall information using robot sensors.
     * 
     * Queries the simulator for walls in front, left, and right of the
     * robot (relative to robotDir) and converts them into absolute
     * directions to update the corresponding Cell.
     * @param x  Robot's current X coordinate.
     * @param y Robot's current Y coordinate.
     * @param robotDir Robot's current heading ('N', 'E', 'S', or 'W').
     */

    void updateFromSensors(int x, int y, char robotDir);
    /**
     * @brief Set the color of a cell in the maze
     * 
     * @param x X coordinate of the cell
     * @param y Y coordinate of the cell
     * @param color Color character identifier
     */

    void setColor(int x, int y, char color) noexcept;
    /**
     * @brief Clear the color of a cell in the maze
     * 
     * @param x X coordinate of the cell
     * @param y Y coordinate of the cell
     */

    void clearColor(int x, int y) noexcept;

    /**
     * @brief Set text at the specified cell in the maze
     * 
     * @param x X coordinate of the cell
     * @param y Y coordinate of the cell
     * @param text Text string to display at the cell
     */

    void setText(int x, int y, const std::string& text) noexcept;

    /**
     * @brief Clear text from the specified cell in the maze
     * 
     * @param x X coordinate of the cell
     * @param y Y coordinate of the cell
     */

    void clearText(int x, int y) noexcept;

    /**
     * @brief Get all free neighbor cells of a given cell.
     * 
     * Returns the list of 4-connected neighbors (north, east, south, west)
     * that are within bounds and not blocked by a wall from the current cell.
     * @param x X coordinate of the current cell.
     * @param y Y coordinate of the current cell.
     * @return Vector of (x, y) coordinates of reachable neighbor cells.
     */

    [[nodiscard]] std::vector<std::pair<int,int>> getNeighbors(int x, int y) const; 

    /**
     * @brief Get the maze width in cells.
     * 
     * @return int Number of columns in the maze grid.
     */

    [[nodiscard]] int getWidth() const noexcept { return width; }

    /**
     * @brief Get the maze height in cells.
     * 
     * @return int Number of rows in the maze grid.
     */
    
    [[nodiscard]] int getHeight() const noexcept { return height; }
};