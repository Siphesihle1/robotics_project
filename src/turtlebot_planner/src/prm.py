import networkx as nx
import random
import math
import numpy as np
import matplotlib.pyplot as plt
import heapq

class GraphVisualization:
    def __init__(self):
        self.visual = []
        self.G = nx.Graph()

    def add_edges(self, edges):
        self.G.add_edges_from(edges)
        
    def visualize(self):
        nx.draw_networkx(self.G)
        plt.show()

class PRM:
    def __init__(self, num_samples, k, data, roadmap=nx.Graph()):
        self.num_samples = num_samples
        self.k = k
        self.data = data
        self.roadmap = roadmap
        self.path = []

    # Checks if the given (x, y) coordinate is in free space
    def is_free_space(self, x, y):
        return self.data[y][x] == 0

    # Calculates Euclidean distance between two points
    def distance(self, p1, p2):
        x1, y1 = p1
        x2, y2 = p2
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    
    # -- Bresenham's line algorithm --
    def check_high(self, x1, y1, x2, y2):
        dx = x2 - x1
        dy = y2 - y1
        xi = 1
        if dx < 0:
            xi = -1
            dx = -dx
        
        D = (2 * dx) - dy
        x = x1

        for y in range(y1, y2 + 1):
            if not self.free_proximity(x, y): return False

            if D > 0:
                x = x + xi
                D = D + (2 * (dx - dy))
            else:
                D = D + 2*dx

        return True
    
    def check_low(self, x1, y1, x2, y2):
        dx = x2 - x1
        dy = y2 - y1
        yi = 1

        if dy < 0:
            yi = -1
            dy = -dy
        
        D = (2 * dy) - dx
        y = y1

        for x in range(x1, x2 + 1):
            if not self.free_proximity(x, y): return False

            if D > 0:
                y = y + yi
                D = D + (2 * (dy - dx))
            else:
                D = D + 2*dy

        return True
    
    # Checks if there is a line of sight between two points
    def line_of_sight(self, x1, y1, x2, y2):
        if abs(y2 - y1) < abs(x2 - x1):
            if x1 > x2:
                return self.check_low(x2, y2, x1, y1)
            else:
                return self.check_low(x1, y1, x2, y2)
        else:
            if y1 > y2:
                return self.check_high(x2, y2, x1, y1)
            else:
                return self.check_high(x1, y1, x2, y2)
    
    # Check if area around (x, y) is free
    def free_proximity(self, x, y, proximity_radius=3):
        w, h = self.data.shape
        if (y > proximity_radius and y < h - proximity_radius) and (x > proximity_radius and x < w - proximity_radius): # Check if not close to boundaries
            # Extract the proximity region around the given coordinates
            data_slice = self.data[y - proximity_radius : y + proximity_radius + 1, x - proximity_radius : x + proximity_radius + 1]

            # Check if the proximity region contains any obstacles (values other than 0)
            if not np.any(data_slice != 0):
                return True

        return False

    
    def build_roadmap(self):
        height, width = self.data.shape

        # Samples nodes in free space
        for _ in range(self.num_samples):
            x = random.randint(0, width - 101)
            y = random.randint(100, height - 1)
            if self.free_proximity(x, y):
                self.roadmap.add_node((x, y))

        self.connect_nodes(self.roadmap.nodes)

    # Connects each node to its k nearest neighbors (if there is a path in free space)
    def connect_nodes(self, nodes):
        for node in nodes:
            nearest_nodes = heapq.nsmallest(self.k + 1, self.roadmap.nodes, key=lambda n: self.distance(node, n))
            nearest_nodes.remove(node)

            for nearest_node in nearest_nodes:
                if self.line_of_sight(node[0], node[1], nearest_node[0], nearest_node[1]):
                    # print("Free", node, nearest_node)
                    self.roadmap.add_edge(node, nearest_node)

    def visualize(self):
        G = GraphVisualization()
        G.add_edges(self.roadmap.edges)
        G.visualize()
    
    def plan(self, start_node, dest_node):
        # Add start and dest nodes (if in free space)
        self.roadmap.add_nodes_from([start_node, dest_node])

        if self.is_free_space(start_node[0], start_node[1]) and self.is_free_space(dest_node[0], dest_node[1]): 
            self.connect_nodes([start_node, dest_node])

            # Perform A* path planning
            try:
                self.path = nx.astar_path(self.roadmap, start_node, dest_node, self.distance)
                return 1
            except:
                return 0
            
        return 2