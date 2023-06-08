import networkx as nx
import yaml
import random
import math
import matplotlib.image as pgm
# import matplotlib.pyplot as plt
import heapq

# Defining a Class
# class GraphVisualization:
   
#     def __init__(self):
          
#         # visual is a list which stores all 
#         # the set of edges that constitutes a
#         # graph
#         self.visual = []
          
#     # addEdge function inputs the vertices of an
#     # edge and appends it to the visual list
#     def add_edge(self, a, b):
#         temp = [a, b]
#         self.visual.append(temp)
          
#     # In visualize function G is an object of
#     # class Graph given by networkx G.add_edges_from(visual)
#     # creates a graph with a given list
#     # nx.draw_networkx(G) - plots the graph
#     # plt.show() - displays the graph
#     def visualize(self):
#         G = nx.Graph()
#         G.add_edges_from(self.visual)
#         nx.draw_networkx(G)
#         plt.show()

# Loading data from yaml file
def load_map(map_file):
    with open(map_file, 'r') as file:
        map_data = yaml.safe_load(file)
    return map_data

# Loads the PGM image file     
def load_image(map_data):
    image_file = map_data['image']
    return pgm.imread(image_file)
    
# Checks if the given (x, y) coordinate is in free space
def is_free_space(image, x, y):
    occupancy_value = image[y][x]
    return occupancy_value == 255

# Calculates Euclidean distance between two points
def distance(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

# Checks if there is a line of sight between two points
def line_of_sight(map_data, image, x1, y1, x2, y2):
    dx = abs(x2 - x1)
    dy = abs(y2 - y1)
    x = x1
    y = y1
    n = 1 + dx + dy
    inc_x = 1 if x2 > x1 else -1
    inc_y = 1 if y2 > y1 else -1
    dx *= 2
    dy *= 2

    for _ in range(n):
        if not is_free_space(image, x, y):
            return False
        if dx > dy:
            x += inc_x
            dx -= dy
        else:
            y += inc_y
            dy -= dx

    return True

def build_roadmap(map_data, image, num_samples, k):
    roadmap = nx.Graph()

    width = image.shape[1]
    height = image.shape[0]

    # Samples nodes in free space
    for _ in range(num_samples):
        x = random.randint(0, width - 1)
        y = random.randint(0, height - 1)
        if is_free_space(image, x, y):
            roadmap.add_node((x, y))

    # Connects each node to its k nearest neighbors (if there is a path in free space)
    for node in roadmap.nodes:
        nearest_nodes = heapq.nsmallest(k + 1, roadmap.nodes, key=lambda n: distance(node, n))
        nearest_nodes.remove(node)
        for nearest_node in nearest_nodes:
            if line_of_sight(map_data, image, node[0], node[1], nearest_node[0], nearest_node[1]):
                roadmap.add_edge(node, nearest_node)

    return roadmap

if __name__ == '__main__':
    map_data = load_map('turtlebot_map2.yaml')
    image = load_image(map_data)
    num_samples = 2000
    k = 10
    
    roadmap = build_roadmap(map_data, image, num_samples, k)

    # Perform path planning or other operations using the roadmap
    # ...

    print('Nodes:', roadmap.nodes)
    print('Edges:', roadmap.edges)