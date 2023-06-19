import numpy as np
from numpy.random import randint

# Toy grid
grid = np.array([
        [1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0],
        [1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0],
        [1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0],
        [1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0],
        [1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0],
        [0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 0, 0],
        [0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 0, 0],
        [0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    ])

print("Grid shape: ", grid.shape)

node_position = (randint(0, grid.shape[0]), randint(0, grid.shape[1]))
print("Random grid position: ", node_position)

if not grid[node_position[0], node_position[1]]:
    print("Position {} is clear".format(node_position))   
else:
    print("Position {} is obstructed".format(node_position))   
    found = False
    # Iterative deepening in a square of side equal to the smaller grid 
    # dimension
    for r in range(1, min(grid.shape[0], grid.shape[1])):
        # Generate the list of node offsets
        offsets = [(n, e) for e in range(-r, r + 1) for n in range(-r, r + 1)]
        # Try all adjacent nodes at "radius" r
        for m in offsets:
            # Adjacent position
            new_pos = (node_position[0] + m[0], node_position[1] + m[1])
			# Check if within the grid
            if new_pos[0] >= 0 and new_pos[0] < grid.shape[0] and \
                new_pos[1] >= 0 and new_pos[1] < grid.shape[1]:
				# Check if clear
                if not grid[new_pos[0], new_pos[1]]:
                    print("Found clear position ({0}, {1})".format( \
                        new_pos[0], new_pos[1]))
                    found = True
                    break
        if found:
            print("Search succeeded")
            break

