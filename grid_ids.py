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

# Offset multipliers relative to a node position (considered (0, 0))
multipliers = [(-1, -1), (1, -1), (1, 1), (-1, 1)]

rand_pos = (randint(0, grid.shape[0]), randint(0, grid.shape[1]))
print("Random grid position: ", rand_pos)

if not grid[rand_pos[0], rand_pos[1]]:
    print("Position {1} is clear".format(rand_pos))   
else:
    found = False
    # Iterative deepening in a square of side equal to the smaller grid 
    # dimension
    for i in range(min(grid.shape[0], grid.shape[1])): # TODO: Start at 1
        # Circle around
        # TODO: This doesn't circle around at all (diag and i > 1)
        for m in multipliers:
            # Check out of bounds
            new_pos = (rand_pos[0] + i * m[0], rand_pos[1] + i * m[1])
            if new_pos[0] >= 0 and new_pos[0] < grid.shape[0] and \
                new_pos[1] >= 0 and new_pos[1] < grid.shape[1]:
                if not grid[new_pos[0], new_pos[1]]:
                    print("Found clear position ({1}, {2})".format( \
                        new_pos[0], new_pos[1]))
                found = True
                break
        if found:
            break
