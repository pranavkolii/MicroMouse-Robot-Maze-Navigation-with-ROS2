#include "maze.hpp"
#include "maze_api.hpp"

#include <cctype>
#include <string>
#include <vector>

using micro_mouse::MazeControlAPI;

namespace {

    // Normalize to 'n','e','s','w' (default 'n')
    char normalize_dir(char d) {
        d = static_cast<char>(std::tolower(static_cast<unsigned char>(d)));
        switch (d) {
            case 'n': case 'e': case 's': case 'w':
                return d;
            default:
                return 'n';
        }
    }

    char left_of(char d) {
        d = normalize_dir(d);
        switch (d) {
            case 'n': return 'w';
            case 'w': return 's';
            case 's': return 'e';
            case 'e': return 'n';
            default:  return 'n';
        }
    }

    char right_of(char d) {
        d = normalize_dir(d);
        switch (d) {
            case 'n': return 'e';
            case 'e': return 's';
            case 's': return 'w';
            case 'w': return 'n';
            default:  return 'n';
        }
    }

} 

Maze::Maze()
: width(0)
, height(0)
{
    width  = MazeControlAPI::get_maze_width();
    height = MazeControlAPI::get_maze_height();

    grid.clear();
    grid.resize(height, std::vector<Cell>(width));
}

bool Maze::inBounds(int x, int y) const noexcept {
    return x >= 0 && x < width &&
    y >= 0 && y < height;
}

bool Maze::hasWall(int x, int y, char dir) const noexcept {
    if (!inBounds(x, y)) {
        return true; // treat out-of-bounds as walled
    }
    return grid[y][x].hasWall(dir);
}

void Maze::setWall(int x, int y, char dir) {
    if (!inBounds(x, y)) return;
    char d = normalize_dir(dir);

    grid[y][x].markWall(d);
    MazeControlAPI::set_wall(x, y, d);
}

void Maze::clearWall(int x, int y, char dir) {
    if (!inBounds(x, y)) return;

    char d = normalize_dir(dir);

    MazeControlAPI::clear_wall(x, y, d);
}

void Maze::updateFromSensors(int x, int y, char robotDir) {
    if (!inBounds(x, y)) return;

    // Relative walls from sensors
    bool front = MazeControlAPI::has_wall_front();
    bool left  = MazeControlAPI::has_wall_left();
    bool right = MazeControlAPI::has_wall_right();

    char facing = normalize_dir(robotDir);

    if (front) {
        setWall(x, y, facing);
    }
    if (left) {
        char absLeft = left_of(facing);
        setWall(x, y, absLeft);
    }
    if (right) {
        char absRight = right_of(facing);
        setWall(x, y, absRight);
    }
}

void Maze::setColor(int x, int y, char color) noexcept {
    if (!inBounds(x, y)) return;
    MazeControlAPI::set_color(x, y, color);
}

void Maze::clearColor(int x, int y) noexcept {
    if (!inBounds(x, y)) return;
    MazeControlAPI::clear_color(x, y);
}

void Maze::setText(int x, int y, const std::string& text) noexcept {
    if (!inBounds(x, y)) return;
    MazeControlAPI::set_text(x, y, text);
}

void Maze::clearText(int x, int y) noexcept {
    if (!inBounds(x, y)) return;
    MazeControlAPI::clear_text(x, y);
}

std::vector<std::pair<int,int>> Maze::getNeighbors(int x, int y) const {
    std::vector<std::pair<int,int>> neighbors;
    if (!inBounds(x, y)) return neighbors;

    // Enforce N, E, S, W search order
    struct DirInfo {
        char d;
        int dx;
        int dy;
    };

    static const DirInfo dirs[4] = {
        { 'n',  0,  1 }, // North: y+1
        { 'e',  1,  0 }, // East:  x+1
        { 's',  0, -1 }, // South: y-1
        { 'w', -1,  0 }  // West:  x-1
    };

    for (const auto& info : dirs) {
        int nx = x + info.dx;
        int ny = y + info.dy;

        if (!inBounds(nx, ny)) continue;
        if (hasWall(x, y, info.d)) continue; // blocked in that direction

        neighbors.emplace_back(nx, ny);
    }

    return neighbors;
}