#include "dfs_algorithm.hpp"

#include <algorithm>
#include <iostream>

DFSAlgorithm::DFSAlgorithm()
: initialized_(false)
, root_{0, 0}
, currentGoal_{0, 0}
{
}

void DFSAlgorithm::reset() {
    initialized_   = false;
    robotVisited_.clear();
    parent_.clear();
    stack_.clear();
}

// Robot tells DFS "I have physically visited this cell".
void DFSAlgorithm::addRobotVisit(const Coord& c) {
    robotVisited_.insert(c);
}

// Helper: build path from root_ to 'node' using parent_.
std::vector<DFSAlgorithm::Coord>
DFSAlgorithm::buildPathRootTo(const Coord& node) const {
    std::vector<Coord> path;
    auto it = parent_.find(node);
    if (it == parent_.end()) {
        return path; // empty: node not in tree
    }

    Coord cur = node;
    while (true) {
        path.push_back(cur);
        auto pit = parent_.find(cur);
        if (pit == parent_.end() || pit->second == cur) {
            break; // reached root (or self-parented)
        }
        cur = pit->second;
    }
    std::reverse(path.begin(), path.end()); // root_ ... node
    return path;
}

// rebuild stack_ so it is [root_, ..., start] using parent_ and also adds that path into visitedLocal.
void DFSAlgorithm::rebuildStackToStart(const Coord& start, std::set<Coord>& visitedLocal) {

    std::cerr << "\n\n[DFS] rebuildStackToStart("
    << start.first << "," << start.second << ")\n";

    // If we have no parent info for 'start', we treat it as a new root.
    if (!initialized_ || parent_.find(start) == parent_.end()) {
        std::cerr << "[DFS] start not in parent tree. Resetting root.\n";

        root_        = start;
        initialized_ = true;

        stack_.clear();
        parent_.clear();

        parent_[start] = start;
        stack_.push_back(start);

        // visitedLocal seeded by caller from robotVisited_
        visitedLocal.insert(start);

        std::cerr << "[DFS] new root = (" << start.first
        << "," << start.second << ")\n";
        return;
    }

    // built path from root_ to start using existing tree.
    std::vector<Coord> pathRootToStart = buildPathRootTo(start);

    // printing path root→start
    std::cerr << "[DFS] path root→start: ";
    for (auto &c : pathRootToStart) {
        std::cerr << "(" << c.first << "," << c.second << ") ";
    }
    std::cerr << "\n";

    if (pathRootToStart.empty()) {
        std::cerr << "[DFS] pathRootToStart empty. Resetting root.\n";
        root_        = start;
        stack_.clear();
        parent_.clear();

        parent_[start] = start;
        stack_.push_back(start);
        visitedLocal.insert(start);
        return;
    }

    // ensuring all nodes on this path are in visitedLocal
    for (const auto& c : pathRootToStart) {
        visitedLocal.insert(c);
    }

    // set stack_ to pathRootToStart
    stack_.assign(pathRootToStart.begin(), pathRootToStart.end());

    std::cerr << "[DFS] stack rebuilt.\n";
}

// build path from start to goal using tree & LCA.
std::vector<DFSAlgorithm::Coord>
DFSAlgorithm::buildPathStartToGoal(const Coord& start,
                                    const Coord& goal) const {
    std::vector<Coord> empty;

    std::vector<Coord> pathRS = buildPathRootTo(start);
    std::vector<Coord> pathRG = buildPathRootTo(goal);

    if (pathRS.empty() || pathRG.empty()) return empty;

    std::size_t i = 0;
    std::size_t n = std::min(pathRS.size(), pathRG.size());
    while (i < n && pathRS[i] == pathRG[i]) {
        ++i;
    }
    if (i == 0) return empty;

    std::size_t lcaIndex = i - 1;

    std::vector<Coord> path;

    // start → ... → childOfLCA (reverse along pathRS)
    for (std::size_t k = pathRS.size(); k-- > lcaIndex + 1; ) {
        path.push_back(pathRS[k]);
    }

    // LCA → ... → goal (forward along pathRG)
    for (std::size_t k = lcaIndex; k < pathRG.size(); ++k) {
        path.push_back(pathRG[k]);
    }

    return path;
}

std::vector<std::pair<int,int>> DFSAlgorithm::solve( const Maze& maze, std::pair<int,int> start,std::pair<int,int> goal){
    using Coord = std::pair<int,int>;

    // First-time initialization: root = first start.
    if (!initialized_) {
        initialized_   = true;
        root_          = start;
        currentGoal_   = goal;
        parent_.clear();
        stack_.clear();

        parent_[start] = start;
        stack_.push_back(start);

        // Robot will call addRobotVisit(start) externally; here we just use whatever is in robotVisited_.
    } 
    else {
        currentGoal_ = goal;
    }

    // Local DFS visited set = cells that robot has actually visited so far.
    std::set<Coord> visitedLocal = robotVisited_;

    // ensuring stack_ matches root→start and that this path is in visitedLocal.
    rebuildStackToStart(start, visitedLocal);

    // PRINT robotVisited_ and stack_ AFTER REBUILD
    std::cerr << "[DFS] robotVisited_ (nodes actually visited by robot): ";
    for (const auto& c : robotVisited_) {
        std::cerr << "(" << c.first << "," << c.second << ") ";
    }
    std::cerr << "\n";

    std::cerr << "[DFS] stack_ after rebuild: ";
    for (const auto& c : stack_) {
        std::cerr << "(" << c.first << "," << c.second << ") ";
    }
    std::cerr << "\n\n";

    bool found = false;

    struct Dir { int dx, dy; char dchar; };
    static const Dir dirs[4] = {
        {  0,  1, 'n' },
        {  1,  0, 'e' },
        {  0, -1, 's' },
        { -1,  0, 'w' }
    };

    // DFS LOOP
    while (!stack_.empty()) {
        Coord n = stack_.back();
        int x = n.first;
        int y = n.second;


        if (n == goal) {
            found = true;
            break;
        }

        bool advanced = false;

        for (const auto& d : dirs) {
            int nx = x + d.dx;
            int ny = y + d.dy;
            Coord child = {nx, ny};

            bool inb  = maze.inBounds(nx, ny);
            bool wall = (!inb || maze.hasWall(x, y, d.dchar));
            bool vis  = (visitedLocal.count(child) != 0);



            if (!inb || wall || vis) continue;

            // Search discovers a new node
            visitedLocal.insert(child);    // local search visited
            parent_[child] = n;           // extend DFS tree
            stack_.push_back(child);
            advanced = true;
            break;
        }

        if (!advanced) {
            // No unvisited neighbors from n: backtrack.
            stack_.pop_back();
        }
    }

    if (!found) {
        return {};
    }

    std::vector<Coord> path = buildPathStartToGoal(start, goal);
    return path; // start - ... - goal
}