This project centers on the design and evaluation of an AI-controlled Snake Game that uses classical and heuristic search algorithms to navigate a dynamic environment. The objective is to simulate intelligent behavior in guiding the snake to reach food while avoiding obstacles, thereby eliminating the need for manual control. The project integrates various pathfinding strategies including Breadth-First Search (BFS), Depth-First Search (DFS), Uniform Cost Search (UCS), Iterative Deepening Search (IDS), Greedy Best-First Search (GBFS), A* Search, and a Random Move approach.

Each algorithm was tested within a Python-based game simulation, and performance was analyzed in terms of time taken, efficiency of the path (number of moves), and failure rate. The data, summarized in the report, shows that:

A* consistently outperformed all other algorithms with optimal paths and minimal failure.

BFS and UCS were reliable but slower due to their exhaustive nature.

GBFS was fast but sometimes suboptimal due to local optima traps.

DFS and Random Move showed high failure rates and inefficient paths, making them unreliable for real-time AI applications.

IDS provided accuracy similar to BFS but was computationally expensive.

The game simulation visually demonstrated how AI can autonomously make decisions in uncertain and constrained environments. The findings underline the practical use of AI and search algorithms in game development, robotics, and other fields where autonomous navigation is critical.

In conclusion, this project not only deepened the understanding of AI search techniques but also applied them meaningfully in a real-time interactive system—laying a foundation for more advanced AI-controlled systems in gaming and beyond.
