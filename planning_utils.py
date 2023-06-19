from enum import Enum
from queue import PriorityQueue
import numpy as np
from math import sqrt
from udacidrone.frame_utils import global_to_local


def create_grid(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))

    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(np.clip(north - d_north - safety_distance - north_min, 
                            0, 
                            north_size-1)),
                int(np.clip(north + d_north + safety_distance - north_min, 
                            0, 
                            north_size-1)),
                int(np.clip(east - d_east - safety_distance - east_min, 
                            0, 
                            east_size-1)),
                int(np.clip(east + d_east + safety_distance - east_min, 
                            0, 
                            east_size-1)),
            ]
            grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = 1

    # TODO (ivogeorg): 
    # 1. Add obstacle for sea.
    # 2. Add obstacle for Market Street artifact.
    
    return grid, int(north_min), int(east_min)


def create_grid_flipped(data, drone_altitude, safety_distance):
    """
    Returns a flipped grid. Origin (0, 0) is bottom left.

    The verbose offset names are meant as a knowledge point.
    """
    grid, \
    zero_local_to_grid_origin_north_offset, \
    zero_local_to_grid_origin_east_offset = \
        create_grid(data, drone_altitude, safety_distance)

    return np.flipud(grid), \
        zero_local_to_grid_origin_north_offset, \
        zero_local_to_grid_origin_east_offset


def closest_clear_node(grid, node_position):
    """
    If given node is obstructed, return the closest clear position.
    """
    if not grid[node_position[0], node_position[1]]:
        print("Grid position {} is clear".format(node_position))
        return node_position   
    else:
        print("Grid position {} is obstructed".format(node_position))
        print("Looking for an adjacent clear position...")   
        found = False
        # Iterative deepening in a square of side equal to the smaller grid 
        # dimension
        for r in range(1, min(grid.shape[0], grid.shape[1])):
            # Generate the list of node offsets
            offsets = [(n, e) for e in range(-r, r + 1) 
                                for n in range(-r, r + 1)]
            # Try all adjacent nodes at "radius" r
            for m in offsets:
                # Adjacent position
                new_pos = (node_position[0] + m[0], node_position[1] + m[1])
                # Check if within the grid
                if new_pos[0] >= 0 and new_pos[0] < grid.shape[0] and \
                    new_pos[1] >= 0 and new_pos[1] < grid.shape[1]:
                    # Check if clear
                    if not grid[new_pos[0], new_pos[1]]:
                        print("Found clear grid position ({0}, {1})".format( \
                            new_pos[0], new_pos[1]))
                        return (new_pos)

    return None 


def global_position_to_grid_node(global_position, global_home,
                                grid, elevation,
                                north_offset, east_offset):
    """
    Convert global position to grid node tuple (n, e).

    Independent of grid axis 0 orientation. That is, it assumes
    grid is flipped outside of this function. Clips global
    coordinates to edges of world, so arbitrary positions can be
    given. Uses IDS to find a close adjacent unobstructed node.
    """

    # Calculate relative local position
    local_position = global_to_local(global_position, global_home)

    # Calculate row and column indices of grid cell
    grid_position = (int(np.ceil(local_position[0] - north_offset)), 
                    int(np.ceil(local_position[1] - east_offset)))

    # Clip to grid edges
    if grid_position[0] < 0:
        grid_position[0] = 0
    if grid_position[0] >= grid.shape[0]:
        grid_position[0] = grid.shape[0] - 1
    if grid_position[1] < 0:
        grid_position[1] = 0
    if grid_position[1] >= grid.shape[1]:
        grid_position[1] = grid.shape[1] - 1

    # TODO (ivogeorg):
    # If node is obstructed at elevation, find the closest
    # unobstructed with IDS. This doesn't guarantee that it can be 
    # reached, until 3D grid (e.g. courtyard of hotel).
    grid_position = closest_clear_node(grid, grid_position)

    # TODO (ivogeorg):
    # Need to avoid trying to land on the Market St anomaly. In general,
    # should be able to land on buildings and not only at zero elevation.
    # 3D grid is needed for the general case and either a distance
    # sensor or access to the grid's 3-rd dimension during flight.

    return grid_position


class Action(Enum):
    """
    An action is represented by a 3 element tuple.

    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """

    WEST = (0, -1, 1)
    EAST = (0, 1, 1)
    NORTH = (-1, 0, 1)
    SOUTH = (1, 0, 1)
    SW = (1, -1, sqrt(2))
    SE = (1, 1, sqrt(2))
    NW = (-1, -1, sqrt(2))
    NE = (-1, 1, sqrt(2))

    @property
    def cost(self):
        return self.value[2]

    @property
    def delta(self):
        return (self.value[0], self.value[1])


def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    valid_actions = list(Action)
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node

    # check if the node is off the grid or
    # it's an obstacle

    if x - 1 < 0 or grid[x - 1, y] == 1:
        valid_actions.remove(Action.NORTH)
        if Action.NW in valid_actions:
            valid_actions.remove(Action.NW)
        if Action.NE in valid_actions:
            valid_actions.remove(Action.NE)
    if x + 1 > n or grid[x + 1, y] == 1:
        valid_actions.remove(Action.SOUTH)
        if Action.SW in valid_actions:
            valid_actions.remove(Action.SW)
        if Action.SE in valid_actions:
            valid_actions.remove(Action.SE)
    if y - 1 < 0 or grid[x, y - 1] == 1:
        valid_actions.remove(Action.WEST)
        if Action.NW in valid_actions:
            valid_actions.remove(Action.NW)
        if Action.SW in valid_actions:
            valid_actions.remove(Action.SW)
    if y + 1 > m or grid[x, y + 1] == 1:
        valid_actions.remove(Action.EAST)
        if Action.NE in valid_actions:
            valid_actions.remove(Action.NE)
        if Action.SE in valid_actions:
            valid_actions.remove(Action.SE)
    
    return valid_actions


def a_star(grid, h, start, goal):

    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False
    
    while not queue.empty():
        item = queue.get()
        current_node = item[1]
        if current_node == start:
            current_cost = 0.0
        else:              
            current_cost = branch[current_node][0]
            
        if current_node == goal:        
            # print('Found a path.')
            found = True
            break
        else:
            for action in valid_actions(grid, current_node):
                # get the tuple representation
                da = action.delta
                next_node = (current_node[0] + da[0], current_node[1] + da[1])
                branch_cost = current_cost + action.cost
                queue_cost = branch_cost + h(next_node, goal)
                
                if next_node not in visited:                
                    visited.add(next_node)               
                    branch[next_node] = (branch_cost, current_node, action)
                    queue.put((queue_cost, next_node))
             
    if found:
        print('**********************')
        print('     Found path!      ')
        print('**********************') 
        # retrace steps
        n = goal
        path_cost = branch[n][0]
        path.append(goal)
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************') 

    # TODO (ivogeorg):
    # Return found (bool), path (None if not found), path_cost (-1 if no path)
    return path[::-1], path_cost



def heuristic(position, goal_position):
    return np.linalg.norm(np.array(position) - np.array(goal_position))

