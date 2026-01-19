/**
 * @file direction.hpp
 * @author Akshun Sharma (asharm41@umd.edu), Pranav Koli (pkoli@umd.edu), 
 *         Anish Gupta (agupta96@umd.edu), Sidharth Mathur (sidmat03@umd.edu)
 * @brief Definition of basic compass directions used for maze navigation.
 * @version 1.0
 * @date 2025-11-29
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#pragma once

/**
 * @brief Cardinal directions for robot orientation and movement.
 * 
 * Basic direction enum used by Robot and others
 */

enum class Direction {
    NORTH = 0,
    EAST  = 1,
    SOUTH = 2,
    WEST  = 3
};