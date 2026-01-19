#include "robot.hpp"
#include "maze_api.hpp"
#include "direction.hpp"
#include "algorithm.hpp"
#include "dfs_algorithm.hpp"

#include <random>
#include <vector>
#include <iostream>
#include <string>
#include <cstdlib>

using micro_mouse::MazeControlAPI;

namespace {

    Direction turnRight(Direction d) {
        switch (d) {
            case Direction::NORTH: return Direction::EAST;
            case Direction::EAST:  return Direction::SOUTH;
            case Direction::SOUTH: return Direction::WEST;
            case Direction::WEST:  return Direction::NORTH;
        }
        return Direction::NORTH;
    }

    char directionToChar(Direction d) {
        switch (d) {
            case Direction::NORTH: return 'N';
            case Direction::EAST:  return 'E';
            case Direction::SOUTH: return 'S';
            case Direction::WEST:  return 'W';
        }
        return 'N';
    }

    std::string straightSymbol(int dx, int dy, bool isFinalVerticalStep = false) {
        // east
        if (dx == 1  && dy == 0) return "-->";
        // west
        if (dx == -1 && dy == 0) return "<--";
        // north (dy = +1)
        if (dx == 0 && dy == 1) {
            return isFinalVerticalStep ? "^" : "|";
        }
        // south (dy = -1)
        if (dx == 0 && dy == -1) {
            return isFinalVerticalStep ? "v" : "|";
        }
        return "?";
    }

    // ASCII corner symbols
    std::string cornerSymbol(int pdx, int pdy, int dx, int dy) {

        // EAST (1,0) - NORTH (0,1)
        if (pdx == 1 && pdy == 0 && dx == 0 && dy == 1) return "_|";

        // NORTH (0,1) - EAST (1,0)
        if (pdx == 0 && pdy == 1 && dx == 1 && dy == 0) return "|-";


        // EAST (1,0) - SOUTH (0,-1)
        if (pdx == 1 && pdy == 0 && dx == 0 && dy == -1) return "-|";

        // SOUTH (0,-1) - EAST (1,0)
        if (pdx == 0 && pdy == -1 && dx == 1 && dy == 0) return "|_";


        // WEST (-1,0) - NORTH (0,1)
        if (pdx == -1 && pdy == 0 && dx == 0 && dy == 1) return "|_";

        // NORTH (0,1) - WEST (-1,0)
        if (pdx == 0 && pdy == 1 && dx == -1 && dy == 0) return "-|";


        // WEST (-1,0) - SOUTH (0,-1)
        if (pdx == -1 && pdy == 0 && dx == 0 && dy == -1) return "|-";

        // SOUTH (0,-1) - WEST (-1,0)
        if (pdx == 0 && pdy == -1 && dx == -1 && dy == 0) return "_|";

        return "?";
    }

} 


Robot::Robot(): maze(), algorithm(nullptr),x(0), y(0), dir(Direction::NORTH), selectedGoal({7,7}){}

void Robot::setAlgorithm(Algorithm* algo) { algorithm = algo; }

void Robot::selectRandomGoal() {
    static const std::pair<int,int> goals[4] = {
        {7,7},{7,8},{8,7},{8,8}
    };

    std::random_device rd;
    std::mt19937 gen(rd());
    int idx = std::uniform_int_distribution<int>(0,3)(gen);

    selectedGoal = goals[idx];

    maze.setColor(selectedGoal.first, selectedGoal.second,'r');
    maze.setText(selectedGoal.first, selectedGoal.second,"G");
}

// moving Robot

bool Robot::moveOneStep(int tx, int ty) {
    int dx = tx - x;
    int dy = ty - y;
    if (abs(dx)+abs(dy)!=1) return false;

    Direction desired = dir;
    if (dx==0 && dy==1)  desired = Direction::NORTH;
    if (dx==1 && dy==0)  desired = Direction::EAST;
    if (dx==0 && dy==-1) desired = Direction::SOUTH;
    if (dx==-1 && dy==0) desired = Direction::WEST;

    int turncount=0;
    while (dir != desired) {
        MazeControlAPI::turn_right();
        dir = turnRight(dir);
        if (++turncount > 4) return false;
    }

    if (MazeControlAPI::has_wall_front()) {
        maze.updateFromSensors(x,y,directionToChar(dir));
        return false;
    }

    MazeControlAPI::move_forward(1);

    switch (dir) {
        case Direction::NORTH: y++; break;
        case Direction::EAST:  x++; break;
        case Direction::SOUTH: y--; break;
        case Direction::WEST:  x--; break;
    }

    maze.updateFromSensors(x,y,directionToChar(dir));
    maze.setColor(x,y,'y');

    if (auto dfs = dynamic_cast<DFSAlgorithm*>(algorithm))
        dfs->addRobotVisit({x,y});

    return true;
}

// Exploration Loop

void Robot::startExploration() {
    if (!algorithm) {
        std::cerr << "[Robot] No algorithm.\n";
        return;
    }

    selectRandomGoal();
    auto goal = selectedGoal;
    maze.updateFromSensors(x,y,directionToChar(dir));
    maze.setColor(x,y,'b');
    maze.setText(x,y,"S");

    if (auto dfs = dynamic_cast<DFSAlgorithm*>(algorithm))
        dfs->addRobotVisit({x,y});

    while (true) {
       
        // clearing all symbols and colors each iteration

        maze.setColor(x, y, 'y');
        maze.setColor(goal.first, goal.second, 'g');
        maze.setColor(0,0, 'b');

        if (x == goal.first && y == goal.second) return;

        auto path = algorithm->solve(maze,{x,y},goal);

        if (path.empty()) {
            std::cerr << "[Robot] No path.\n";
            return;
        }

        // drawing path symbols (straight + corner)
        
        maze.setColor(goal.first,goal.second,'r');
        maze.setText(goal.first,goal.second,"G");

        for (size_t i=0; i+1<path.size(); ++i) {
            int x0 = path[i].first;
            int y0 = path[i].second;
            int x1 = path[i+1].first;
            int y1 = path[i+1].second;

            int dx = x1 - x0;
            int dy = y1 - y0;

            if (i > 0) {
                int px = path[i-1].first;
                int py = path[i-1].second;
                int pdx = x0 - px;
                int pdy = y0 - py;

                bool turning = !(pdx==dx && pdy==dy);
                if (turning) {
                    maze.setText(x0,y0, cornerSymbol(pdx,pdy,dx,dy));
                    continue;
                }
            }

            // For vertical segments, we mark the last vertical step differently:
            //   - non-final: "|"
            //   - final: "^" (north) or "v" (south)
            
            bool isFinalVerticalStep = (i + 1 == path.size() - 1);
            maze.setText(x0,y0, straightSymbol(dx,dy,isFinalVerticalStep));
        }

        bool replan = false;

        for (size_t i=1; i<path.size(); ++i) {
            if (!moveOneStep(path[i].first, path[i].second)) {
                std::cerr << "[Robot] Obstacle found.\n";
                replan = true;
                break;
            }
            if (x==goal.first && y==goal.second) return;
        }

        if (!replan && !(x==goal.first && y==goal.second)) {
            std::cerr << "[Robot] Ended early; replanning...\n";
        }
    }
}