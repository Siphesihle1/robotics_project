import heapq
import math
import numpy as np

# Calculate the heuristic distance between two points (L2 norm)
# def distance(p1, p2):
#     x1, y1 = p1
#     x2, y2 = p2
#     return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

# L1 norm
def distance(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    return 0.5 * (abs(x2 - x1) + abs(y2 - y1))

# Chebyshev distance (L_inf norm)
# def distance(p1, p2):
#     x1, y1 = p1
#     x2, y2 = p2
#     return max(abs(x2 - x1), abs(y2 - y1))

# Check if a given coordinate is within the grid boundaries
def is_within_grid(coord, grid):
    rows, cols = grid.shape
    return 0 <= coord[0] < rows and 0 <= coord[1] < cols

# Check if a given coordinate is free
def is_free(coord, grid):
    return grid[coord[0]][coord[1]] == 0

def astar(start, destination, grid):
    # Initialize the open and closed sets
    open_set = []
    closed_set = dict()

    # Define the possible movements
    movements = [
        (-1, 0),  # Up
        (1, 0),   # Down
        (0, -1),  # Left
        (0, 1),   # Right
        (-1, -1), # Diagonal: Up-Left
        (-1, 1),  # Diagonal: Up-Right
        (1, -1),  # Diagonal: Down-Left
        (1, 1)    # Diagonal: Down-Right
    ]

    # Initialize the start node with a cost of 0
    heapq.heappush(open_set, (0, start))

    # Initialize the cost dictionary with infinite values
    cost = {start: 0}

    # Initialize the parent dictionary to track the path
    parent = {}

    while open_set:
        # Pop the node with the lowest cost from the open set
        _, current = heapq.heappop(open_set)

        # Check if the current node is the destination
        if current == destination:
            break

        # Add the current node to the closed set
        closed_set[current] = 1

        # Explore the neighboring nodes
        for move in movements:
            new_coord = (current[0] + move[0], current[1] + move[1])

            # Skip if the new coordinate is not within the grid boundaries or is not free
            if not is_within_grid(new_coord, grid) or not is_free(new_coord, grid):
                continue

            # Calculate the new cost from the start node to the neighbouring node
            new_cost = cost[current] + 1

            # If the new cost is lower than the existing cost or the neighbouring node is not in the closed set,
            # update the cost and add the neighbouring node to the open set
            if new_cost < cost.get(new_coord, float('inf')):
                cost[new_coord] = new_cost
                priority = new_cost + distance(new_coord, destination)
                heapq.heappush(open_set, (priority, new_coord))
                parent[new_coord] = current
                print(current)

    # Reconstruct the path from the destination to the start
    path = []
    current = destination
    while current in parent:
        path.append((current[1], current[0])) # (x, y)
        current = parent[current]
    path.append((start[1], start[0])) # (x, y)

    # Reverse the path
    path.reverse()

    return path
