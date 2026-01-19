#include "bfs_algorithm.hpp"
#include "maze.hpp"

#include <queue>
#include <map>
#include <vector>
#include <algorithm> // std::reverse

std::vector<std::pair<int,int>> BFSAlgorithm::solve(
    const Maze& maze,
    std::pair<int,int> start,
    std::pair<int,int> goal
) {
    std::queue<std::pair<int,int>> q;
    std::map<std::pair<int,int>, std::pair<int,int>> parent;

    q.push(start);
    parent[start] = start;

    bool found = false;

    while (!q.empty()) {
        auto current = q.front();
        q.pop();

        if (current == goal) {
            found = true;
            break;
        }

        auto neighbors = maze.getNeighbors(current.first, current.second);
        for (const auto& nb : neighbors) {
            if (parent.find(nb) != parent.end()) {
                continue; // already visited
            }
            parent[nb] = current;
            q.push(nb);
        }
    }

    std::vector<std::pair<int,int>> path;
    if (!found) {
        return path; // empty path if no route
    }

    // Reconstructing path from goal back to start
    std::pair<int,int> node = goal;
    while (true) {
        path.push_back(node);
        auto it = parent.find(node);
        if (it == parent.end() || it->second == node) {
            break;
        }
        node = it->second;
    }

    std::reverse(path.begin(), path.end());
    return path;
}