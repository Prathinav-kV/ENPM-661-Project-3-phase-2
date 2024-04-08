#!/usr/bin/env python
# coding: utf-8

# In[7]:


import numpy as np
from queue import PriorityQueue
import time
from datetime import datetime
import pygame
import sys


def valid_point(x, y, radius, clear):
    d = radius+clear
    # Check if inside the bounding box
    inside_box = d <= x <= 600 - d and d <= y <= 200 - d

    # Check if outside rectangle 1
    outside_rectangle_1 = not (150 - d < x < 175 + d and y > 100 - d)

    # Check if outside rectangle 2
    outside_rectangle_2 = not (250 - d < x < 275 + d and y < 100 + d)

    # Check if outside circle
    distance_to_circle_center = ((x - 420)**2 + (y - 120)**2) ** 0.5
    outside_circle = distance_to_circle_center > (60+d)

    # Combine all conditions
    return inside_box and outside_rectangle_1 and outside_rectangle_2 and outside_circle



def move_star(node, wheel_rpms, radius, clear, wheel_radius, wheel_distance):
    # Extract RPMs for each wheel from the provided tuple
    RPM1 = wheel_rpms[0]  # RPM for the first wheel
    RPM2 = wheel_rpms[1]  # RPM for the second wheel
    
    # Define action set: all possible combinations of RPMs for the robot's wheels
    # This includes moving straight with both wheels at the same RPM, turning in place
    # with wheels moving in opposite directions, and turning while moving forward
    action_set = [
        [RPM1, RPM1],  # Both wheels at RPM1, move straight
        [RPM2, RPM2],  # Both wheels at RPM2, move straight
        [RPM1, RPM2],  # Differential movement, turn
        [RPM2, RPM1],  # Differential movement, turn
        [0, RPM1],     # Turn in place, one wheel stationary
        [RPM1, 0],     # Turn in place, one wheel stationary
        [0, RPM2],     # Turn in place, one wheel stationary
        [RPM2, 0]      # Turn in place, one wheel stationary
    ]
    
    newNodes = []  # Initialize a list to store feasible new nodes resulting from the actions

    # Iterate over each action in the action set
    for rpms in action_set:
        # Calculate the cost and resulting node for the action
        result = cost(node, rpms, radius, wheel_radius, wheel_distance, clear)
        # If the action results in a valid movement (i.e., doesn't collide with obstacles)
        if result is not None:
            newNode, moveCost = result  # Unpack the resulting node and cost
            # Append the new node, its movement cost, and the RPMs used to the list of new nodes
            newNodes.append((newNode, moveCost, rpms))
    
    return newNodes  # Return the list of feasible new nodes



def cost(node, rpms, radius, wheel_radius,wheel_distance,clear):
    # Initialize time and time step for simulation
    t = 0  # Current time
    dt = 0.1  # Time step for each simulation iteration
    
    # Extract current node's position and orientation
    current_node_x = node[0]  # X position
    current_node_y = node[1]  # Y position
    current_node_theta = node[2] * 30  # Orientation, converting from discrete representation to degrees
    current_node_theta = np.radians(current_node_theta)  # Convert orientation from degrees to radians
    
    # Initialize cumulative cost-to-go and new node's state to current state
    C2G = 0.0  # Cumulative cost-to-go, initially 0
    newNode_x = current_node_x  # New node's initial X position
    newNode_y = current_node_y  # New node's initial Y position
    newNode_theta = current_node_theta  # New node's initial orientation
    
    # Simulate movement for 1 second, updating the node's state at each time step
    while t < 1:
        t += dt  # Increment time by time step
        # Calculate changes in position and orientation based on differential drive kinematics
        Delta_x = 0.5 * wheel_radius * (rpms[0] + rpms[1]) * np.cos(newNode_theta) * dt * 2 * np.pi / 60  # Change in X
        Delta_y = 0.5 * wheel_radius * (rpms[0] + rpms[1]) * np.sin(newNode_theta) * dt * 2 * np.pi / 60  # Change in Y
        Delta_theta = (wheel_radius / wheel_distance) * (rpms[0] - rpms[1]) * dt * 2 * np.pi / 60  # Change in theta
        
        # Update new node's state
        newNode_x += Delta_x
        newNode_y += Delta_y
        newNode_theta += Delta_theta
        
        # Update cumulative cost-to-go (distance traveled during this step)
        C2G += np.sqrt(Delta_x**2 + Delta_y**2)
        
        # Check if the new position is within a valid area (not colliding with any obstacles)
        if not valid_point(newNode_x, newNode_y, radius, clear):
            return None  # If new position is invalid, return None indicating failure to move

    # Normalize and discretize new orientation to match the discrete environment representation
    newNode_theta = int(np.rad2deg(newNode_theta)) % 360  # Convert from radians to degrees and normalize
    newNode_theta = newNode_theta // 30  # Discretize orientation into 30-degree increments

    # Return the new node's state and the cost to move to it
    return [int(newNode_x), int(newNode_y), int(newNode_theta)], int(C2G)


def heuristic(node, goal_node):
    # Calculate the Euclidean distance between the current node and the goal node
    return 1.5 * np.sqrt(np.square(goal_node[0] - node[0]) + np.square(goal_node[1] - node[1]))


def inGoal(node, goal_node):
    
    # Define the radius around the goal node considered as reaching the goal
    goal_radius = 5
    
    # Extract the x and y coordinates of the goal node
    x_goal = goal_node[0]
    y_goal = goal_node[1]

    # Extract the x and y coordinates of the current node
    x_node = node[0]
    y_node = node[1]
    
    # Calculate the Euclidean distance between the current node and the goal node
    return np.sqrt(np.square(x_node-x_goal) + np.square(y_node-y_goal)) < goal_radius




def a_star_algorithm(start, goal, wheel_rpms, radius, clear, wheel_radius, wheel_distance):
    # Initialize a 3D cost grid to store the cost of reaching each node, setting initial costs to infinity.
    # The grid dimensions are based on the workspace size and the discretization of orientation (12 directions).
    cost_grid = [[[float('inf')] * 12 for _ in range(200)] for _ in range(600)]
    # Set the starting node's cost to 0 as it is the initial position.
    cost_grid[start[0]][start[1]][start[2]] = 0

    # Initialize a 3D parent grid to keep track of the parent node for each visited node, facilitating path reconstruction.
    # Initially, all nodes have no parent (None).
    parent_grid = [[[None] * 12 for _ in range(200)] for _ in range(600)]
    # The start node does not have a parent, thus it's set to None explicitly.
    parent_grid[start[0]][start[1]][start[2]] = None

    # Initialize a 3D visited grid to mark nodes as visited or not, and a list to keep track of visited nodes in order.
    visited_grid = [[[False] * 12 for _ in range(200)] for _ in range(600)]
    visited_list = []

    # Initialize a priority queue for the open set, where nodes are prioritized by their total cost (path cost + heuristic).
    open_queue = PriorityQueue()
    # Add the start node to the open set with a priority of 0 (start node's cost + heuristic is zero).
    open_queue.put((0, start))  # (priority, node)

    # Loop until there are no more nodes in the open set (open_queue is empty).
    while open_queue:
        # Retrieve and remove the node with the lowest total cost from the open set.
        element = open_queue.get()
        node = element[1]
        # Mark the current node as visited in the visited grid and add it to the visited list.
        visited_grid[node[0]][node[1]][node[2]//30] = True
        visited_list.append(node)
        
        # Check if the current node is within the goal area, using the inGoal function.
        # If so, return the parent grid and list of visited nodes for path reconstruction and visualization.
        if inGoal(node, goal):
            return parent_grid, visited_list

        # Generate all possible valid movements from the current node using the move_star function.
        # This considers the robot's motion model and obstacle avoidance.
        actions = move_star(node, wheel_rpms, radius, clear, wheel_radius, wheel_distance)
        # Retrieve the cost of reaching the current node from the cost grid.
        node_cost = cost_grid[node[0]][node[1]][node[2]]

        # Iterate over each possible movement/action generated.
        for action in actions:
            # Extract the resulting node (position and orientation), its movement cost, and the RPMs used.
            move, move_cost, rpms = action
            # Extract and calculate the indices for the resulting node in the cost, parent, and visited grids.
            x, y, theta = move
            x_index, y_index, theta_index = int(x), int(y), int(theta)
            
            # Check if the resulting node has not been visited.
            if not visited_grid[x_index][y_index][theta_index]:
                    # Calculate the new cost to reach the resulting node (current node cost + movement cost).
                    new_cost = node_cost + move_cost
                    # If this new cost is lower than the previously recorded cost for the resulting node, update it.
                    if new_cost < cost_grid[x_index][y_index][theta_index]:
                        cost_grid[x_index][y_index][theta_index] = new_cost
                        # Calculate the priority of the node for the open set (new cost + heuristic cost to goal).
                        priority = new_cost + heuristic([x, y], goal)
                        # Add the resulting node to the open set with the calculated priority.
                        open_queue.put((priority, (x, y, theta)))
                        # Update the parent of the resulting node to the current node for path reconstruction.
                        parent_grid[x_index][y_index][theta_index] = (node, rpms)

    # If the goal is not reached and the open set is empty, return the current state with a message indicating failure.
    return parent_grid, visited_list, print("Failed to find goal")



def find_path(parent_grid, visited_list, start):
    # Start from the last node in the visited list, which should be the goal node or the node closest to the goal.
    current_node = visited_list[-1]
    # Initialize the path with the current (goal) node.
    path = [current_node]
    # The start node is our initial position.
    start_node = start
    
    # Loop until we trace back to the start node.
    while start_node != current_node:
        # Retrieve the parent node and the RPMs used to get to the current node.
        parent_data = parent_grid[current_node[0]][current_node[1]][current_node[2]]
        temp_node, rpms = parent_data
        # Insert the parent node at the beginning of the path list, effectively reversing the path as we go.
        path.insert(0, (temp_node, rpms))
        # Update the current node to be the parent node, moving backwards through the path.
        current_node = temp_node
    
    # Return the constructed path from start to goal.
    return path

#Animate function
def animate(start, goal, path, visited_list):
    pygame.init()
    pastel_background = (230, 230, 250)
    dark_pastel_background = (200, 200, 220)
    red = (255, 0, 0)
    black = (0, 0, 0)
    blue = (0, 0, 255)
    white = (255, 255, 255)
    canvas_width = 600
    canvas_height = 200
    canvas = pygame.display.set_mode((canvas_width, canvas_height))
    static_surface = pygame.Surface((canvas_width, canvas_height))
    pygame.display.set_caption("Canvas with Shapes")
    static_surface.fill(pastel_background)
    
    rectangles = [pygame.Rect(150, 0, 25, 100), pygame.Rect(250, 100, 25, 100)]
    
    for rect in rectangles:
        pygame.draw.rect(static_surface, dark_pastel_background, rect)
        
    pygame.draw.circle(static_surface, dark_pastel_background, (420, 80), 60)

    clock = pygame.time.Clock()
    visited_index = 0  
    nodes_per_frame = 50
    path_index = 0
    path_nodes_per_frame = 1
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                
        canvas.blit(static_surface, (0, 0))
        for i in visited_list[:visited_index]:
            pygame.draw.circle(canvas, white, (int(i[0]), int(200 - i[1])), 1)
        visited_index += nodes_per_frame
        
        if visited_index >= len(visited_list):
            for node_action in path[:path_index]:
                if isinstance(node_action, tuple) and len(node_action) > 0 and isinstance(node_action[0], tuple) and len(node_action[0]) >= 2:
                    node = node_action[0]
                    x, y = node[0], node[1]  
                    pygame.draw.circle(canvas, black, (int(x), 200 - int(y)), 1)
                # else:
                #     print(f"Unexpected structure: {node_action}")
            path_index += path_nodes_per_frame  
            
        pygame.draw.circle(canvas, red, (start[0], 200 - start[1]), 5)
        pygame.draw.circle(canvas, blue, (goal[0], 200 - goal[1]), 5)
        pygame.display.flip()
        clock.tick(60)
    pygame.quit()
    sys.exit()

def get_path():
    #user input values in cm
    clear = round(int(input("Enter the clearance in the canvas: ")))
    clear = abs(clear)
    radius = 22 #cm
    wheel_radius = 3.3 #cm
    wheel_distance = 28.7 #cm
    
    start_x = abs(int(input("Enter the starting coordinate x: ")))
    start_y = abs(int(input("Enter the starting coordinate y: ")))
    start_th = (int(input("Enter the starting orientation in degrees: "))%360)//30
    
    while not (valid_point(start_x,start_y, radius,clear)):
        print("Please try another starting point not in the object area")
        start_x = abs(int(input("Enter the starting coordinate x: ")))
        start_y = abs(int(input("Enter the starting coordinate y: ")))
    
    
    goal_x = abs(int(input("Enter the goal coordinate x: ")))
    goal_y = abs(int(input("Enter the goal coordinate y: ")))
    
    while not (valid_point(goal_x,goal_y,radius,clear)):
        print("Please try another goal point not in the object area")
        goal_x = abs(int(input("Enter the goal coordinate x: ")))
        goal_y = abs(int(input("Enter the goal coordinate y: ")))
    
    RPM1 =  abs(int(input("Enter the Wheel RPM1 ")))
    RPM2 =  abs(int(input("Enter the Wheel RPM2 ")))
    
    start = (start_x,start_y,start_th)
    goal = (goal_x,goal_y)
    wheel_rpms = (RPM1,RPM2)
    
    start_time = datetime.now()
    print('Exploring nodes...')
    explored = a_star_algorithm(start, goal,wheel_rpms, radius, clear, wheel_radius, wheel_distance)
    parent_grid = explored[0]
    visited_list = explored[1]
    
    print('Generating path...')
    path = find_path(parent_grid, visited_list, start)
    end_time = datetime.now()
    time_taken = end_time - start_time
    print('Path found in: ', time_taken," seconds")

    #converting the path recieved into meters
    new_path = []
    for i in path[:-1]:
        points = i[0]
        rpms = i[1]
        x = points[0]
        y = points[1]
        theta = points[2]

        x = float(x/100) - 0.5
        y = float(y/100) - 1
        theta = theta*30

        values = [x,y,theta]
        new_path.append([values,rpms])
    new_path.append([((goal_x/100)-0.5,(goal_y/100)-1),[0,0]])
    animate(start,goal,path,visited_list)
    return new_path

if __name__=="__main__":
    get_path()


# In[ ]:




