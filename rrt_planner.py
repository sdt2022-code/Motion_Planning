"""
Assignment #2 Template file
"""
import random
import math
import numpy as np

"""
Problem Statement
--------------------
Implement the planning algorithm called Rapidly-Exploring Random Trees (RRT)
for the problem setup given by the RRT_DUBINS_PROMLEM class.

INSTRUCTIONS
--------------------
1. The only file to be submitted is this file rrt_planner.py. Your implementation
   can be tested by running RRT_DUBINS_PROBLEM.PY (check the main function).
2. Read all class and function documentation in RRT_DUBINS_PROBLEM carefully.
   There are plenty of helper function in the class to ease implementation.
3. Your solution must meet all the conditions specificed below.
4. Below are some do's and don'ts for this problem as well.

Conditions
-------------------
There are some conditions to be satisfied for an acceptable solution.
These may or may not be verified by the marking script.

1. The solution loop must not run for more that a certain number of random iterations
   (Specified by a class member called MAX_ITER). This is mainly a safety
   measure to avoid time-out-related issues and will be set generously.

2. The planning function must return a list of nodes that represent a collision-free path
   from start node to the goal node. The path states (path_x, path_y, path_yaw)
   specified by each node must define a Dubins-style path and traverse from node i-1 -> node i.
   (READ the documentation for the node class to understand the terminology)

3. The returned path should have the start node at index 0 and goal node at index -1,
   while the parent node for node i from the list should be node i-1 from the list, ie,
   the path should be a valid list of nodes.
   (READ the documentation of the node to understand the terminology)

4. The node locations must not lie outside the map boundaries specified by
   RRT_DUBINS_PROBLEM.map_area.

DO(s) and DONT(s)
-------------------
1. Do not rename the file rrt_planner.py for submission.
2. Do not change change the PLANNING function signature.
3. Do not import anything other than what is already imported in this file.
4. You can write more function in this file in order to reduce code repitition
   but these function can only be used inside the PLANNING function.
   (since only the planning function will be imported)
"""

def rrt_planner(rrt_dubins, display_map=False):
    """
        Execute RRT planning using Dubins-style paths. Make sure to populate the node_list.

        Inputs
        -------------
        rrt_dubins  - (RRT_DUBINS_PROBLEM) Class conatining the planning
                      problem specification
        display_map - (boolean) flag for animation on or off (OPTIONAL)

        Outputs
        --------------
        (list of nodes) This must be a valid list of connected nodes that form
                        a path from start to goal node

        NOTE: In order for rrt_dubins.draw_graph function to work properly, it is important
        to populate rrt_dubins.nodes_list with all valid RRT nodes.
    """
    # LOOP for max iterations
    i = 0
    indicator = False
    np.random.seed(2)
    #temp =[]
    #temp.append(rrt_dubins.start)
 
    while i < rrt_dubins.max_iter:
        i += 1

        # Generate a random vehicle state (x, y, yaw)

        x_sample = random.uniform(rrt_dubins.x_lim[0] , rrt_dubins.x_lim[-1])
        y_sample = random.uniform(rrt_dubins.y_lim[0] , rrt_dubins.y_lim[-1])
        yaw_sample = random.uniform(0,2*math.pi)
        node_sample = rrt_dubins.Node(x_sample , y_sample, yaw_sample) 
        

        # Find an existing node nearest to the random vehicle state

        dist =[]
        for j in range(len(rrt_dubins.node_list)):
            dist.append((np.sqrt((rrt_dubins.node_list[j].x - node_sample.x)**2+(rrt_dubins.node_list[j].y - node_sample.y)**2)))
        
        nearest_node_index = np.argmin(dist)

        nearest_node = rrt_dubins.node_list[nearest_node_index]


        # Propogate from nearest_node to sampled node
        new_node = rrt_dubins.propogate(nearest_node,node_sample)

    

        # Check if the path between nearest node and random state has obstacle collision
        # Add the node to nodes_list if it is valid
        if rrt_dubins.check_collision(new_node):
            rrt_dubins.node_list.append(new_node)

            if rrt_dubins.calc_dist_to_goal(new_node.x , new_node.y)<2:
                is_goal_node = rrt_dubins.propogate(new_node , rrt_dubins.goal)
                rrt_dubins.node_list.append(is_goal_node)

                indicator = True
            
                                       

                 
        # Draw current view of the map
        # PRESS ESCAPE TO EXIT
        if display_map:
            rrt_dubins.draw_graph()


        # Check if new_node is close to goal (did so in the last loop)
        
        if indicator:    
           path_to_goal_node = []
           node_to_add = rrt_dubins.node_list[-1]
           path_to_goal_node.append(node_to_add)

           while True: 
 
            node_to_add = node_to_add.parent
            path_to_goal_node.append(node_to_add)
            
            if node_to_add == rrt_dubins.start:
                path_to_goal_node.reverse()
                print("Iters:", i, ", number of nodes:", len(rrt_dubins.node_list))
                print("The total cost from start to finish is: {}".format(is_goal_node.cost))
                return(path_to_goal_node)            

          
        if i == rrt_dubins.max_iter:
            print('reached max iterations')

            # Return path, which is a list of nodes leading to the goal...
    return rrt_dubins.node_list
