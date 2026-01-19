
GROUP 4

Group Members - 

(1) Akshun Sharma
(2) Pranav Koli
(3) Anish Gupta
(4) Sidharth Mathur

Note: By default our code runs DFS, for switching to BFS implementation please uncomment lines 49 and 50 and comment lines 41 & 42 in main.cpp.

Comparison between DFS and BFS on the following features - 

(1) Completeness - If a path exists in a finite graph, both BFS and DFS guarantee finding it.

(2) Optimality   - BFS is optimal (since it finds the optimal path and explores all nodes layer by layer) while DFS is non optimal since it tends to explore one path to its maximum depth,
                   potentially resulting in finding a very long path first.
                   
(3) Memory Usage - BFS requires more memory than DFS. BFS requires storing all nodes in the current layer of the search frontier in the queue. DFS only requires to store the current path
                   from the root to the frontier on the stack, making it more memory efficient.
                   
(4) Exploration  - BFS used broad exploration i.e. it explores neighbors evenly, expanding the search frontier in a uniform circle from the start whereas DFS uses deep exploration and 
                   explores a single branch until a dead end is reached before backtracking.

In the case of finding any path (for low memory usage) DFS is a better solution. BFS is preferred for solving the maze if our goal is to find the shortest path but it can be memory intensive
if used for very large mazes.
