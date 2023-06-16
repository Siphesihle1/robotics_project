#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import *
from nav_msgs.srv import GetMap
from gazebo_msgs.srv import GetModelState
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import networkx as nx
from prm import PRM
from turtlebot_planner.msg import Path
import sys

# Display the map
def plot_map(data, edges, marked_nodes):
    plt.imshow(data)
    
    plt.title( "Map" )
    plt.xlabel('x-axis')
    plt.ylabel('y-axis')

    for node0, node1 in edges:
        x_vals = [node0[0], node1[0]]
        y_vals = [node0[1], node1[1]]
        plt.plot(x_vals, y_vals, 'yo', linestyle='-')
    
    for node in marked_nodes:
        plt.scatter(node[0], node[1], s=100, c='g')

    plt.show()

# Get map data and pack into dict
def get_map_data():
    map_info = dict()
    rospy.wait_for_service('/static_map')
    try:
        get_map = rospy.ServiceProxy('/static_map', GetMap)
        response = get_map()
        map_info["width"] = response.map.info.width
        map_info["height"] = response.map.info.height
        map_info["data"] = np.array(response.map.data).reshape(map_info["height"], map_info["width"])
        map_info["resolution"] = response.map.info.resolution
        map_info["origin"] = response.map.info.origin
        return map_info
    except rospy.ServiceException as e:
        print("Map service call failed.")
    return None

# Get current robot location
def get_robot_location():
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        response = get_model_state(model_name='mobile_base')
        robot_x = response.pose.position.x
        robot_y = response.pose.position.y
        return (robot_x, robot_y)
    except rospy.ServiceException as e:
        print("Model state service call failed.")
    return None

# From coordinate to pixel space and back
def normalize(location, resolution, origin, type=0):
    if type == 0:    
        return (
            int(round((location[0] - origin.x) / resolution)),
            int(round((location[1] - origin.y) / resolution))
        )
    return (
            origin.x + location[0] * resolution,
            origin.y + location[1] * resolution
        )

# 2k * 2j slice
def gen2dlslice(data, k, j, coord):
    slice = data[coord[1] - k: coord[1] + k, coord[0] - j: coord[0] + j]
    slice[k][j] = 42
    print(slice)

# Flattens a list
def flatten(path):
    flattend_path = []
    for node in path:
        flattend_path.extend([node[0], node[1]])
    return flattend_path

def parse_edge(line):
    part1, part2 = (val.strip() for val in line.split(":"))
    node1 = tuple(int(val.strip()) for val in  part1.split(","))
    node2 = tuple(int(val.strip()) for val in  part2.split(","))
    return [(node1, node2)]
    
def create_graph(lines):
    G = nx.Graph()
    for line in lines:
        G.add_edges_from(parse_edge(line))
    return G

def write_to_file(num_samples, k, edges, file_name):
    with open(file_name, 'w') as f:
        f.write("{} {}\n".format(num_samples, k))
        lines = ["{}, {} : {}, {}\n".format(
            node0[0], node0[1], node1[0], node1[1])
            for node0, node1 in edges]
        f.writelines(lines)
    
def build_roadmap(data, file_name, num_samples=1000, k=30):
    prm = None
    with open(file_name, "r") as f:
        lines = f.readlines() 
        if len(lines) == 0:
            # Create if empty
            prm = PRM(num_samples=num_samples, k=k, data=data)
            prm.build_roadmap()

            # write to file
            print("Writing to file...")
            write_to_file(num_samples, k, prm.roadmap.edges, file_name)
        else:
            # Use existing roadmap
            _num_samples, _k = (int(val.strip()) for val in lines[0].split(" "))
            G = create_graph(lines[1:])
            prm = PRM(num_samples=_num_samples, k=_k, data=data, roadmap=G)
    
    return prm

if __name__ == "__main__":
    rospy.init_node("planner", anonymous=True)

    # Initialize path publisher
    path_publisher = rospy.Publisher('/path_info', Path, queue_size=10)
    
    # Get map data (try)
    map_info = get_map_data()
    if not map_info: exit(0)

    origin = map_info["origin"].position
    resolution = map_info["resolution"]

    # Build roadmap
    print("Builing roadmap...")
    prm = build_roadmap(num_samples=1500, k=50, data=map_info["data"], file_name=sys.argv[1])
    
    # Get current and goal location (try)
    location = get_robot_location()
    if not location: exit(0)

    cmd_input = raw_input("Location: ")
    while cmd_input != "-1":
        goal_coords_str = cmd_input.split(",")
        goal = tuple(float(i.strip()) for i in goal_coords_str)
        goal_location = normalize(goal, resolution, origin)

        print("Localizing...")
        curr_location = normalize(get_robot_location(), resolution, origin) # pixel coordinates

        # Create plan and publish if found
        ret = prm.plan(curr_location, goal_location)
        if ret == 1:
            print("Found path!")
            
            # -- Shows map with path
            # paired_path_edges = [(prm.path[i], prm.path[i+1]) for i in range(0, len(prm.path) - 1)]
            # plot_map(map_info["data"], paired_path_edges, prm.path)

            # Convert to world coordinates
            path = [normalize(waypoint, resolution, origin, 1) for waypoint in prm.path] 
            
            # Publish path
            flattened_path = flatten(path)

            print("Publishing...")
            path_publisher.publish(Path(length=len(flattened_path), path=flattened_path))
        elif ret == 2:
            print("Destination node not in free space.")
        else:
            print("Couldn't find path.")
        
        cmd_input = raw_input("Location: ")