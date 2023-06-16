import numpy as np
from planning_utils import create_grid
from udacidrone.frame_utils import global_to_local

# Read in obstacle map
data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)

# Set drone altitude in meters
drone_altitude = 5

# Create grid representation of environment
grid, north_offset, east_offset = create_grid(data, drone_altitude, 1)

# Flip the grid horizontally
grid = np.flipud(grid)

# Convert global position to local position
global_position = (-122.397450, 37.792480, 0)
print("Global position: ", global_position)

global_home = (-122.397450, 37.792480, 0)
print("Global home: ", global_home)

local_position = global_to_local(global_position, self.global_home)
print("Local position (relative to home): ", local_position)

# Calculate row and column indices of grid cell
grid_position = (int(np.ceil(local_position[0] - north_offset)), int(np.ceil(local_position[1] - east_offset)))

# Print the grid position (from flipped grid)
print("Grid position (from grid flipped up-down to correspond to simulator): ", grid_position)
