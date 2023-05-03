## Udacity
## Flying Car Nanodegree
## Project: 3D Motion Planning
## Author: Ivo Georgiev (ivogeorg@gmail.com)
![Quad Image](./misc/enroute.png)

---


# Required Steps for a Passing Submission:
1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].
7. Write it up.
8. Congratulations!  Your Done!

## [Rubric](https://review.udacity.com/#!/rubrics/1534/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it! Below I describe how I addressed each rubric point and where in my code each point is handled.

### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`

The motion planning code, captured in these scripts, is one of three components of the autonoumous quadcopter drone simulation, the other two being (1) the FCND Unity-based simulator, which contains the city world and all the models and flight dynamics for the quadcopter drone, and (2) the `udacidrone` API, which serves as a pass-through for the MAVLink messages between the motion planning code and the simulator. 

The simulated drone lives entirely inside the Unity framework and executes C# scripts to move through the simulator's city environment. 

The `udacidrone` API and the derived class of the `Drone` class define state-based drones, which transition from state to state as a result of commands sent from the derived classes to the simulator by having executed previously defined callback functions for each transition. The simulator, in turn, sends updates of the position and flight dynamics back to the planning code. 

Both the commands from the derived `Drone` and the state updates from the simulator are sent as asynchronous MAVLink-protocol messages over TCP/IP, in this case through a specific port of the `localhost` address. 



<img src="assets/noun-notes.png" height="200" />   

**TODO:** A system diagram showing derived `Drone` with planning code (i.e. `MotionPlanning`), `udacidrone` API, simulator as a white box with simulated drone and city environment, MAVLink connection with two terminals, and prinicpal messages sreaming in each direction.

<img src="assets/noun-question.png" height="200" />   

1. Does the drone have a "camera"? The simulator shows the drone as seen from the side at a distance. Can the drone "take pictures" from its own perspective, and if yes, what is the orientation of the camera.
2. Does the drone have a LIDAR? There is mention of `Lidar` in drone sensors C# scripts.
3. Does the drone have a distance sensor?

And here's a lovely image of my results (ok this image has nothing to do with it, but it's a nice example of how to include images in your writeup!)
![Top Down View](./misc/high_up.png)

Here's | A | Snappy | Table
--- | --- | --- | ---
1 | `highlight` | **bold** | 7.41
2 | a | b | c
3 | *italic* | text | 403
4 | 2 | 3 | abcd

### Implementing Your Path Planning Algorithm

#### 1. Set your global home position
Here students should read the first line of the csv file, extract lat0 and lon0 as floating point values and use the self.set_home_position() method to set global home. Explain briefly how you accomplished this in your code.


And here is a lovely picture of our downtown San Francisco environment from above!
![Map of SF](./misc/map.png)

#### 2. Set your current local position
Here as long as you successfully determine your local position relative to global home you'll be all set. Explain briefly how you accomplished this in your code.


Meanwhile, here's a picture of me flying through the trees!
![Forest Flying](./misc/in_the_trees.png)

#### 3. Set grid start position from local position
This is another step in adding flexibility to the start location. As long as it works you're good to go!

#### 4. Set grid goal position from geodetic coords
This step is to add flexibility to the desired goal location. Should be able to choose any (lat, lon) within the map and have it rendered to a goal location on the grid.

#### 5. Modify A* to include diagonal motion (or replace A* altogether)
Minimal requirement here is to modify the code in planning_utils() to update the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2), but more creative solutions are welcome. Explain the code you used to accomplish this step.
The additional "diagonal" actions are defined as follows in the `Action` class in `planning_utils.py`
```python
    WEST = (0, -1, 1)
    EAST = (0, 1, 1)
    NORTH = (-1, 0, 1)
    SOUTH = (1, 0, 1)
    SW = (1, -1, sqrt(2))
    SE = (1, 1, sqrt(2))
    NW = (-1, -1, sqrt(2))
    NE = (-1, 1, sqrt(2))
```
and are used to filter the valid actions as follows in the `valid_actions` method
```python
    if x - 1 < 0 or grid[x - 1, y] == 1:
        valid_actions.remove(Action.NORTH)
        valid_actions.remove(Action.NW)
        valid_actions.remove(Action.NE)
    if x + 1 > n or grid[x + 1, y] == 1:
        valid_actions.remove(Action.SOUTH)
        valid_actions.remove(Action.SW)
        valid_actions.remove(Action.SE)
    if y - 1 < 0 or grid[x, y - 1] == 1:
        valid_actions.remove(Action.WEST)
        valid_actions.remove(Action.NW)
        valid_actions.remove(Action.SW)
    if y + 1 > m or grid[x, y + 1] == 1:
        valid_actions.remove(Action.EAST)
        valid_actions.remove(Action.NE)
        valid_actions.remove(Action.SE)
```

#### 6. Cull waypoints 
For this step you can use a collinearity test or ray tracing method like Bresenham. The idea is simply to prune your path of unnecessary waypoints. Explain the code you used to accomplish this step.



### Execute the flight
#### 1. Does it work?
It works!

### Double check that you've met specifications for each of the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points.
  
# Extra Challenges: Real World Planning

For an extra challenge, consider implementing some of the techniques described in the "Real World Planning" lesson. You could try implementing a vehicle model to take dynamic constraints into account, or implement a replanning method to invoke if you get off course or encounter unexpected obstacles.


