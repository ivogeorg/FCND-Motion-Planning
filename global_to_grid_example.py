import numpy as np
from planning_utils import create_grid
from udacidrone.frame_utils import global_to_local

# Read in obstacle map
data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)

# Set drone altitude in meters
drone_altitude = 5

# Create grid representation of environment
grid, north_offset, east_offset = create_grid(data, drone_altitude, 1)

# Convert global position to local position
global_position = (37.792480, -122.397450, 0)
local_position = global_to_local(global_position, self.global_home)

# Calculate row and column indices of grid cell
grid_position = (int(np.ceil(local_position[0] - north_offset)), int(np.ceil(local_position[1] - east_offset)))
