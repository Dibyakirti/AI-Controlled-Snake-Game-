import random
from collections import deque
import heapq

# Directions (Up, Down, Left, Right)
DIRECTIONS = [(-1, 0), (1, 0), (0, -1), (0, 1)]


def reconstruct_path(came_from, start, goal):
    """Helper function to reconstruct the list of moves from start to goal."""
    path = []
    current = goal
    while current != start:
        prev, action = came_from[current]
        path.append(action)
        current = prev
    path.reverse()
    return path


# Breadth First Search (BFS) Algorithm
def bfs(start, goal, obstacles, rows, cols):
    frontier = deque([start])
    came_from = {start: None}

    while frontier:
        current = frontier.popleft()
        if current == goal:
            return reconstruct_path(came_from, start, goal)
        for direction in DIRECTIONS:
            next_node = (current[0] + direction[0], current[1] + direction[1])
            if (0 <= next_node[0] < rows and 0 <= next_node[1] < cols and 
                next_node not in obstacles and next_node not in came_from):
                frontier.append(next_node)
                came_from[next_node] = (current, direction)
    return []  # No path found


# Depth First Search (DFS) Algorithm
def dfs(start, goal, obstacles, rows, cols):
    stack = [start]
    came_from = {start: None}

    while stack:
        current = stack.pop()
        if current == goal:
            return reconstruct_path(came_from, start, goal)
        for direction in DIRECTIONS:
            next_node = (current[0] + direction[0], current[1] + direction[1])
            if (0 <= next_node[0] < rows and 0 <= next_node[1] < cols and 
                next_node not in obstacles and next_node not in came_from):
                stack.append(next_node)
                came_from[next_node] = (current, direction)
    return []  # No path found


# Iterative Deepening Search (IDS) Algorithm
def ids(start, goal, obstacles, rows, cols):
    def depth_limited_search(current, goal, depth, visited):
        if current == goal:
            return []
        if depth == 0:
            return None
        for direction in DIRECTIONS:
            next_node = (current[0] + direction[0], current[1] + direction[1])
            if (0 <= next_node[0] < rows and 0 <= next_node[1] < cols and
                next_node not in obstacles):
                if next_node in visited:
                    continue
                visited.add(next_node)
                result = depth_limited_search(next_node, goal, depth - 1, visited)
                if result is not None:
                    return [direction] + result
                visited.remove(next_node)
        return None

    # Set a maximum depth (worst-case scenario: visiting every cell)
    max_depth = rows * cols
    for depth in range(max_depth):
        visited = set([start])
        result = depth_limited_search(start, goal, depth, visited)
        if result is not None:
            return result
    return []  # No path found


# Uniform Cost Search (UCS) Algorithm
def ucs(start, goal, obstacles, rows, cols):
    frontier = []
    heapq.heappush(frontier, (0, start))
    came_from = {start: None}
    cost_so_far = {start: 0}

    while frontier:
        cost, current = heapq.heappop(frontier)
        if current == goal:
            return reconstruct_path(came_from, start, goal)
        for direction in DIRECTIONS:
            next_node = (current[0] + direction[0], current[1] + direction[1])
            if (0 <= next_node[0] < rows and 0 <= next_node[1] < cols and
                next_node not in obstacles):
                new_cost = cost + 1  # every move costs 1
                if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                    cost_so_far[next_node] = new_cost
                    heapq.heappush(frontier, (new_cost, next_node))
                    came_from[next_node] = (current, direction)
    return []  # No path found


def heuristic(a, b):
    """Manhattan distance heuristic for grid-based pathfinding."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


# Greedy Best First Search Algorithm
def greedy_bfs(start, goal, obstacles, rows, cols):
    frontier = []
    heapq.heappush(frontier, (heuristic(start, goal), start))
    came_from = {start: None}
    visited = set([start])

    while frontier:
        _, current = heapq.heappop(frontier)
        if current == goal:
            return reconstruct_path(came_from, start, goal)
        for direction in DIRECTIONS:
            next_node = (current[0] + direction[0], current[1] + direction[1])
            if (0 <= next_node[0] < rows and 0 <= next_node[1] < cols and 
                next_node not in obstacles and next_node not in visited):
                visited.add(next_node)
                heapq.heappush(frontier, (heuristic(next_node, goal), next_node))
                came_from[next_node] = (current, direction)
    return []  # No path found


# A* Search Algorithm
def astar(start, goal, obstacles, rows, cols):
    frontier = []
    heapq.heappush(frontier, (heuristic(start, goal), 0, start))
    came_from = {start: None}
    cost_so_far = {start: 0}

    while frontier:
        _, cost, current = heapq.heappop(frontier)
        if current == goal:
            return reconstruct_path(came_from, start, goal)
        for direction in DIRECTIONS:
            next_node = (current[0] + direction[0], current[1] + direction[1])
            if (0 <= next_node[0] < rows and 0 <= next_node[1] < cols and
                next_node not in obstacles):
                new_cost = cost + 1
                if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                    cost_so_far[next_node] = new_cost
                    priority = new_cost + heuristic(next_node, goal)
                    heapq.heappush(frontier, (priority, new_cost, next_node))
                    came_from[next_node] = (current, direction)
    return []  # No path found

def random_move(start, goal, obstacles, rows, cols):
    """Returns a random valid move for the snake."""
    valid_moves = []
    
    for direction in DIRECTIONS:
        next_node = (start[0] + direction[0], start[1] + direction[1])
        if (0 <= next_node[0] < rows and 0 <= next_node[1] < cols and next_node not in obstacles):
            valid_moves.append(direction)
    
    return [random.choice(valid_moves)] if valid_moves else []
