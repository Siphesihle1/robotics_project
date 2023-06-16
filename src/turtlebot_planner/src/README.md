To run first install the required packages:
```
pip install -r requirements.txt
```

Steps:
1. Run the map server node
```
rosrun map_server map_server turtlebot_map.yaml
```

2. Run the path planner using the roadmap file as an argument (or use an empty file to create a new roadmap)
```
rosrun <package> plan_path.py roadmap
```

3. Run navigator node
```
rosrun <package> follow_path_pid.py
```

4. To perform a scan while navigating
```
rosrun <package> scan.py
rostopic echo /witsdetector
```