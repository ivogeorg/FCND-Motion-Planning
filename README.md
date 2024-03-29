# FCND Project 2: Motion Planning

Planning in the complex urban environment of San Francisco using the `udacidrone` API and the Unity-based Udacity simulator. A [guide](project-and-submission-guide.md) describes the project setup and the submission requirements. Extensive inline notes provided in [motion_planning.py](motion_planning.py).

<img src="/assets/FCND-Planning-Upban-World.png" width="840"/>

## Starter Code

The overall architecture of this project is (i) user code contained in a sublclass of the `udacidrone.Drone` class, (ii) the `udacidrone` API serving as an interface between high-level user planning and control and the simulator, mediated via a two-way MAVLink protocol asynchronous message stream, and (iii) the Unity-based simulator that contains the urban world specification and the low-level dynamics and control model of the drone.

### 1. Planning Subclass

The majority of the functionality of the drone is encapsulated in a the subclass `MotionPlanning(Drone)` where the parent `Drone` is defined in the [`udacidrone` package](#2-udacitdronedrone). 

#### 1.1. File `motion_planning.py`

At a high-level, the drone is defined as a state machine, with a `state_callback` method calling a handler method for each state-to-state transition. The states are `MANUAL`, `ARMING`, `TAKEOFF`, `WAYPOINT`, `LANDING`, `DISARMING`, AND `PLANNING`. When the drone is armed (i.e. is in state `ARMING`) it calls `plan_path` and enters the state `PLANNING`. After planning, the drone takes off (state `TAKEOFF`) and positions itself at the start position. The `local_position_callback` transitions the drone to state `WAYPOINT` and calls the `waypoint_transition` method. The drone starts following the planned path (state `WAYPOINT`). A state machine makes control easier by dividing the control task into separate smaller tasks and ensuring that the drone is always in an identifiable state and there is a predefined transition out of the current state.

`plan_path` contains the main part of the difference from `BackyardFlyer`, which had a simple predefined square trajectory to follow. `MotionPlanning` is able to fly arbitrary trajectories inside a complex urban environment from between given start and stop positions. The method takes a text file containing collision coordinates for downtown San Francisco and then creates and lays over a grid over the environment out of which waypoint trajectories will be picked to navigate the drone around it. Given a start and goal positions from the grid, the heuristic A* search is used to find a path between them. After pruning and further finetuning of the path, the path is sent to the simulator, and the drone sets off to complete it.

`motion_planning.py` is the executable that is run to plan and fly the `MotionPlanning` drone in the [simulator](#4-simulator).

#### 1.2. File `motion_utils.py`

The file contains helper methods for creating and navigating a grid-based configuration space:

`create_grid` creates a 2D grid based on the colliders data at a certain drone height and with a certain standoff safety distance.

The class `Action(Enum)` assumes a grid-based environment and defines straight (North, South, East, and West) transitions between grid points and, later on, diagonal (SE, SW, NE, NW) ones. Transitions between grid points carry a cost, 1 for straight and sqrt(2) for diagonal.

`valid_actions` filters the available actions for a drone at a certain point (aka "node"). Adding the diagonal actions to the enumerated class `Action` also requires additional filtering code in this method.

`a_star` performs a heuristic search in the grid space based on the Euclidean-distance heuristic.

### 2. `udacidrone.Drone`

Defined in [udacidrone/udacidrone/drone.py](https://github.com/udacity/udacidrone/blob/4d0addf86055f622b313a977913ab5372fb11ac1/udacidrone/drone.py) the `Drone` class contains:
1. The dynamic state of the vehicle (`self.update_property`, etc.)
2. Telemetry handling (`log_telemetry`, etc.)
3. Messaging (`on_message_receive`, `self.update_property`)
4. Callback management (`register_callback`, etc.)
5. Command wrappers (`arm`, `cmd_position`, etc.)

This is essentially a class that serves as an interface to the vehicle dynamics model in the [simulator](#4-simulator). A subclass inherits everything that defines the vehicle and its control from its parent so that it can focus on the additional functionality (in this case, planning a path through a complex 2.5D environment.

#### 2.1. Local `udacidrone` Tree

Due to the necessity to debug the [hanging issue](#45-hanging-in-the-messaging-protocol) described further down below, and furthermore because of the dissappearance of the preinstalled Udacity workspaces, I have imported locally the `udacidrone` tree into the repository and it is run from there.

### 3. `udacidrone` Communications

`udacidrone.connection` contains various channels through which the drone can be connected to its high-level control (e.g. "go_left" rather than a Euler angles). `MotionPlanning` is using a [MAVLink](https://mavlink.io/en/) protocol connection, defined in `mavlink_connection.py` and `mavling_utils.py` files under `udacidrone/udacidrone/connection`. MAVLink is an asynchronous-message protocol, with messages defined in the `Message` class and its subclasses `message_types.py` and the enumerated class `MsgID` in `udacidrone/udacidrone/messaging/message_ids.py`.

The information flowing between the simulator and the `Drone` class (or subclass) consists mainly of state updates, commands, and telemetry.

### 4. Simulator

The [FCND-Simulator](https://github.com/udacity/FCND-Simulator) is based on [Unity](https://unity.com), a 3D gaming, design, and modeling engine. Most of the configurations and code that are specific to the simulator are contained in the [`Assets`](https://github.com/udacity/FCND-Simulator/tree/b662149ae52fee37372ce0e014d624cfe52229c8/Assets) directory.

#### 4.1. World

The assets used to build the world are contained in the `AS Assets`, `CityMap`, `Materials`, `Resources`, `Textures`, `Wrld` and other directories.

#### 4.2. Drone

There are a several directories which contain visual assets for the drone model, namely `CoraTEST`, `Cora_KittyHawk`, and `Drone_Taxi`, but all of the low-level dynamics, sensing, positioning, navigation, messaging, and control are contained in the form of C# code files under the `Scripts` directory. For example, the main drone class is likely `Drones.QuadDrone` in `FCND-Simulator/Assets/Scripts/Drones/QuadDrone.cs` with `DroneInterface`, `DroneControllers`, `DroneVehicles`, and `DroneSensors` all included (`using`) and containing the details. As all derived classes in Unity, `QuadDrone` inherits from `MonoBehavior`, the base class of Unity. `QuadDrone` also implements the `IDrone` interface for drone control, defined in `FCND-Simulator/Assets/Scripts/DroneInterface/Drone.cs`.

#### 4.3. Messaging & Communications

Naturally, the simulator communicates over MAVLink with the high-level user code in `udacidrone.Drone` and its subclasses. A C# implementation of MAVLink is contained in `FCND-Simulator/Assets/Scripts/MAVLink`.

#### 4.4. The "Elevated" Market Street Artifact

There is a simulator artifact in the form of an "elevated" section of Market Street, between Market and Front/Fremont and Market and Spear. It is 3 meters tall and has "entrances" at the street level (but they don't appear consistently across simulator resets) as if it is a galleria. I know for sure there isn't anything like that there. This might be an unfinished attempt to model the underground Embarcadero BART and the MUNI Market & Main stations. Here are screenshots showing the two ends of the artifact:

| Street Level | Bird's Eye View |
| --- | --- |
| <img src="/assets/FCND-Planning-Reason-for-overshoot-at-start.png" width="450"/>  | <img src="/assets/FCND-Planning-Mrkt-St-Anomaly-1.png" width="450"/> |
| <img src="/assets/FCND-Planning-Elevated-Mkt-St.png" width="450"/>  | <img src="/assets/FCND-Planning-Mrkt-St-Anomaly-2.png" width="450"/> |

Here is a rare view of the galleria with gates:

<img src="/assets/FCND-Planning-Rare-view-of-Mrkt-St-anomaly.png" width="450"/>

Incidentally, the same can be seen at the bottom of a building on the North edge of the world, looking South from "outside" the world.

This artifact acts like an obstacle and causes the drone centered in the middle of the colliders map data `(lon0, lat0, 0)` to overshoot at start as if passing through an obstacle (not sure how this is handled in Unity). Zero altitude is definitely "inside" the "galleria" but the drone cannot return to it. The drone, if above the "roof" of the "galleria", cannot properly land, disarm, and turn off guidance, since it cannot meet the criterion of (approximately) zero elevation. Here is a sequence of screenshots showing the overshoot:

| Below the roof | Above the roof |
| --- | --- |
| <img src="/assets/Overshoot-1.png" width="450" /> | <img src="/assets/Overshoot-4.png" width="450" /> | 
| <img src="/assets/Overshoot-2.png" width="450" /> | <img src="/assets/Overshoot-5.png" width="450" /> | 
| <img src="/assets/Overshoot-3.png" width="450" /> | <img src="/assets/Overshoot-6.png" width="450" /> | 

The map data does not show any obstacle at the same location and anyway the default flight elevation of 5 meters can avoid it altogether. This only affects taking off from and landing anywhere in the footprint of the "galleria". Unfortunately, the simulator always initializes the drone at the center of the city, which is inside the galleria anomaly. So, the first flight will be affected, but as long as the drone does not try to land on the roof of the galleria, the rest will be fine. A bird's eye inspection of the rest of the city shows no similar anomalies.

It's interesting to point out that since the 2D is for a certain altitude, a clear (white or magenta) region is not necessarily at zero altitude, so the drone might not be able to land there w/o modification of the [`velocity_callback`](/motion_planning.py#L61) code.

#### 4.5. Hanging in the Messaging Protocol

When the A* takes a long time to find a path (usually of unpruned length of 200+ steps) some internal timeout or protocol invariant is violated and the program hangs in the `pymavlink` code. The following is a representative output upon killing the application:

```shell
(fcnd) orbital@yocto-sandbox:~/git-repos/FCND-Motion-Planning$ /home/orbital/miniconda3/envs/fcnd/bin/python /home/orbital/git-repos/FCND-Motion-Planning/motion_planning.py
Logs/TLog.txt
Logs/NavLog.txt
starting connection
arming transition
Global home :  [-122.3974533   37.7924804    0.       ]
Global position:  [-122.3971478   37.7905706    0.207    ]
Local position:  [-2.12161575e+02  2.71234398e+01 -2.04146892e-01]
North offset = -316, East offset = -445
Starting from current position
Grid position (105, 474) is obstructed
Looking for an adjacent clear position...
Found clear grid position (104, 473)
Grid position (158, 483) is obstructed
Looking for an adjacent clear position...
Found clear grid position (166, 496)
Local Start and Goal:  (104, 473) (166, 496)
Searching for a path ...
**********************
     Found path!      
**********************
Path length:  285
Sending waypoints to simulator ...
takeoff transition
^CTraceback (most recent call last):
  File "/home/orbital/git-repos/FCND-Motion-Planning/motion_planning.py", line 415, in <module>
    drone.start()
  File "/home/orbital/git-repos/FCND-Motion-Planning/motion_planning.py", line 369, in start
    self.connection.start()
  File "/home/orbital/git-repos/FCND-Motion-Planning/udacidrone/connection/mavlink_connection.py", line 235, in start
    self.dispatch_loop()
  File "/home/orbital/git-repos/FCND-Motion-Planning/udacidrone/connection/mavlink_connection.py", line 113, in dispatch_loop
    msg = self.wait_for_message()
  File "/home/orbital/git-repos/FCND-Motion-Planning/udacidrone/connection/mavlink_connection.py", line 199, in wait_for_message
    msg = self._master.recv_match(blocking=True, timeout=1)
  File "/home/orbital/miniconda3/envs/fcnd/lib/python3.6/site-packages/pymavlink/mavutil.py", line 355, in recv_match
    self.select(timeout/2)
  File "/home/orbital/miniconda3/envs/fcnd/lib/python3.6/site-packages/pymavlink/mavutil.py", line 212, in select
    (rin, win, xin) = select.select([self.fd], [], [], timeout)
KeyboardInterrupt
(fcnd) orbital@yocto-sandbox:~/git-repos/FCND-Motion-Planning$ 
```
Once the application hangs due to this problem, the simulator has to be reset before it would stop hanging, leading me to suspect that the problem is simulator-related (after all, the drone model is contained in the simulator and it communicated using `pymavlink` with the `udacidrone.Drone`.

## Code Sections

### 1. Diagonal Actions

The diagonal actions are implemented in the `planning_uitls.py` file, specifically:
1. In the `Action` class:
   ```
      SW = (1, -1, sqrt(2))
      SE = (1, 1, sqrt(2))
      NW = (-1, -1, sqrt(2))
      NE = (-1, 1, sqrt(2))
   ```
2. In the `valid_actions` method, whenever a principal direction is removed, the diagonal directions having it as a component are also removed, conditionally to prevent attempts at removing non-existent list elements:
   ```
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
    ```
3. In the simulator, the zig-zag trajectory is now a straight line:
   
   <img src="/assets/Project2-diagonal-motion.png" width="450"/>  

### 2. Global Start and Goal

The function `global_position_to_grid_node(global_position, global_home, grid, elevation, north_offset, east_offset)` is the workhorse of providing the start and goal positions using global (lon, lat, alt) coordinates. It is implemented in `planning_utils.py` along with the helper function `closest_clear_node(grid, node_position)` which performs a sort of "outward spiral" iterative deepening search for the closest unobstructed node to the one corresponding to the given global position.

The start is either the center of the grid (the `lat0, lon0` coordinates given at the top of the `colliders.csv` file), as done in the following code in the encapsulating method `init_navigation(self)` in `motion_planning.py`:
```python
latlon = ""
with open("colliders.csv", "r") as f:
    latlon = f.readline()
lat0, lon0 = latlon.split(", ")
lat0 = float(lat0.split()[1])
lon0 = float(lon0.split()[1])
```
or the (approximate) end of the previous misssion, which is maintained by the (unreset) simulator under `global_position`.

### 3. Randomized Goal

The goal is randomized in the following code in the `plan_path` method in `motion_planning.py`:
```python
 # NOTE 26 (ivogeorg):
  # Long paths take a very long time to find by A*, and cause timeout-
  # related problems with the messaging protocol, including a hang after
  # takeoff_transition. The easiest way to circumvent this problem is to
  # limit the distance between the start and the goal. This will result
  # in shorter flights with very little delay between them. Defining
  # latitude and longitude steps (a tenth of the corresponding span)
  # allows to generate goals within a small square of the start
  # position. Clip to world edges.
  LAT_MIN = 37.789649
  LAT_MAX = 37.797903
  LON_MIN = -122.402424
  LON_MAX = -122.392115
  lat_step = fabs(LAT_MAX - LAT_MIN) / 10.0
  lon_step = fabs(LON_MAX - LON_MIN) / 10.0
  goal_global_lat = np.random.uniform(self.global_position[1] - lat_step,
                                      self.global_position[1] + lat_step)
  goal_global_lat = minmax(LAT_MIN, goal_global_lat, LAT_MAX)
  goal_global_lon = np.random.uniform(self.global_position[0] - lon_step,
                                      self.global_position[0] + lon_step)
  goal_global_lon = minmax(LON_MIN, goal_global_lon, LON_MAX)
  grid_goal = global_position_to_grid_node(
                  (goal_global_lon, goal_global_lat, 0.0), 
                  self.global_home, 
                  self.grid, self.TARGET_ALTITUDE, 
                  self.north_offset, self.east_offset)
```
To limit the length of the path (and so avoid some of the occurrences of the [hanging issue](#45-hanging-in-the-messaging-protocol), the distance of the goal from the start is constrained within a rectangle of half-size 1/10-th of the corresponding global coordinate. The function `minmax` from `planning_utils.py` is used to clip the random coordinates to the edges of the world. The maximum and minimum global coordinates were determined by manual drone flight around the world.

### 4. Connection Timeout

The mavlink connection timeout has been increased from 60 to 1800 to allow A* to find a path.

### 5. Path Pruning

The path is being pruned in the `prune_path(path)` function in `planning_utils.py`, currently using the collinearity test for straight sections of the path. The screenshot below shows path with pruned straight sections.

<img src="/assets/FCND-Planning-Pruned-Path-Collinearity.png" width="450"/>  

In a future release, a Bresenham-like algorithm will be used to prune the zig-zag sections of the path.

## Flying the Drone

### 1. Overview of Setup

The drone can fly multiple consecutive missions in a row without resetting the simulator. It randomizes its next goal, finds a path to it, and follows the waypoints to reach it. That goal becomes the new start position. The goal is generated in global coorindates and is converted first to local coordinates and then to a grid node. If the grid node has an obstacle at the target altitude, the closest unobstructed node is found.

<img src="/assets/FCND-Planning-Flying-Around.png" width="850"/>  

### 2. Instructions

1. Open the simulator on the correct reset state (see below). If it opens in the wrong one (in which there is no city), hit Shift-r. Pressing Shift-r multiple times just alternates the two reset states.
   | Wrong | Correct |
   | --- | --- |
   | <img src="/assets/FCND-Planning-Wrong-Start.png" width="300"/> | <img src="/assets/FCND-Planning-Correct-Start.png" width="300"/> | 
3. Arm the drone and fly up (SPACE). Because of the Market St. obstacle described [above](#44-the-elevated-market-street-artifact), the drone will overshoot, but eventually will settle at some altitude corresponding to how much SPACE was pressed.
4. Fly a small distance along Davis St. to make sure that the drone is not on top of the Market St. obstacle. For example, fly forward with W (see below).
   | Near | Far |
   | --- | --- |
   | <img src="/assets/FCND-Planning-Start-Near.png" width="300"/> | <img src="/assets/FCND-Planning-Start-Far.png" width="300"/> | 
5. Land the drone (C).
6. Run the code (`python motion_planning.py`). The drone will set its current position as `grid_start`, find a random goal, set the closest unobstructed grid node to `grid_goal`, and attempt to find a path. If it finds a path, it will fly the trajectory and land. If it doesn't it will disarm and switch to manual. In either case, repeat this step to see several flights in a row, flying around the city.
7. *Note that we don't randomize `grid_start` because the drone will attempt to fly to it regardless of obstacles in between its current position and `grid_start`.*


## *Private Notes*

### *1. Notes on goal given in [lon, lat]*

1. If a node is not given explicitly, and this is the case for a global (lon, lat) position, then the closest node has to be found. This is `target_node_ini`.
2. [global_to_sim_example.py](global_to_sim_example.py) contains the code to convert from global (lon, lat) to a grid node which is in conformance with the simulator frame. Consider encapsulating in a function `global_to_grid_node()` in [planning_utils.py](planning_utils.py). It can handle:
   1. Out-of-bound input global coordinates, clipping at the edges of the world.
   2. The Market Street anomaly.
   3. The water of the bay in the top-right (NE) corner of the simulator world. 
4. The grid coming from `create_grid` has to be inverted with `np.flipud()` (flip/reverse axis 0) to get in conformance with the simulator frame. Consider retruning an already flipped grid from `create_grid` so `global_to_grid_node()` assumes a flipped grid.
5. Consider renaming the offsets to more descriptive names. They are negative, so they point to the grid origin (which is top-left or, after `np.flipud()`, bottom_left) from the perspective of global home and zero local position (which is the center of both the simulation world and the colliders data). When they are flipped in [motion_planning.py](motion_planning.py#L171), they point to global home and zero local position from the perspective of the grid origin. So, consider names `zero_local_to_grid_origin_north_offset` and `zero_local_to_grid_origin_east_offset`. These are verbose but will be encapsulated in `global_to_grid_node()`.
6. If `target_node_ini` has an obstacle, the closest unobstructed node has to be found. This is `target_node_clear`. A modified `a_star_clear` can be used to find it, starting from `target_node_ini` and stopping when the closest unobstructed node is found. 
7. If `target_node_ini` is clear, it is returned, otherwise `target_node_clear` is.
8. `a_star` can be used normally with `grid_start` and `grid_goal` for each flight.

#### *1.1. Questions on the grid*

1. ~What are the coordinate numbers in the [colliders file](no-latlon-colliders.csv)? Since there are both negative and positive float values for the first two columns, these are neither global (which should be a positive latitude and a negative longitude) nor local coordinates (which should have north and east both positive).~ The coordinate numbers are Cartesian distances in meters relative to the center of the grid given by the global position on the first line.
   1. Note that `global_to_local` takes two arguments, `global_position` and `global_home` so essentially the local coordinates are returned relative to the **home position**, which is what the significance of `global_home` and `set_home_position` is. This might provide a clue to the mapping of local positions to grid and to the role of the offsets (see below).
   2. Global coordinates of the 4 corners (approximate (lon, lat)): SW (-122.4024, 37.7897), NW (-122.4024, 37.7979), NE (-122.3921, 37.7979), SE (-122.3921, 37.7897).
   3. The center of the grid is: C (-122.3975, 37.7925).
   4. Running them with zero altitude through `global_to_local` with C as `global_home` gives:
      1. SW [-313.42789948, -429.43659388,   -0.0]
      2. NW [ 596.35963319, -435.25197039,   -0.0]
      3. NE [ 602.20641541,  471.54922346,   -0.0]
      4. SE [-307.58154996,  477.46484185,   -0.0]
3. ~Where exactly is the start position, relative to the grid, when set as follows `grid_start = (-north_offset, -east_offset)`?~ Global home, if lon0 and lat0 from the first line of the colliders data are used to set it, by the logic of `global_to_local`, local position (0, 0, 0). 
4. ~Why are the offsets integers?~ All the node positions are tuples of integer distances in meters. The grid has a resolution of 1 meter in each axis (north, east, alt/up). This is achieved by rounding, and this certainly introduces error. The safe distance of 3 meters takes care of that.
5. ~Why are they negated? Does this have to do with flipping the grid so that North is positive up and East is positive to the right, so that the origin of the grid is bottom left?~ They are negated to change the perspective from the grid origin to local position (0, 0, 0), because are returned by `create_grid` as calculated from the opposite perspective. The grid has to be flipped by `np.flipud()` to (i) visualize correctly (as a normal map of San Francisco) and (ii) correspond to the simulator frame. 
6. ~Why does the grid have to be flipped?~ To correspond to the simulator frame. 
7. ~Any way to display all the clear nodes on the grid?~ Yes, though there will be many and very dense. See this [notebook](/notebooks/A-Star-City.ipynb).
8. ~What exactly, relative to the grid, is the goal in `grid_goal = (-north_offset + 10, -east_offset + 10)`?~ 10 meters from grid origin in each axis direction (north, east).
9. How should the tuple indices of `grid_clear_nodes` be used to conform to this goal formation?
10. ~Is there a correspondence between the indices of the grid nodes and their local positions? For example, are they evenly spaced?~ A corresponding grid node can be found from a local position by using `ceil` and casting to `int` to get the grid node indices. Nodes are one meter apart in each axis direction. Because of this rounding, one cannot recover a single local position tuple from a node index tuple. At best, it will be a circle (or sphere) around the node, showing the local positions that round to this node.
11. What is the relationship between the path grid nodes and the waypoints? It looks like there is a 1-to-1 correspondence with a constant N-E offset for the grid nodes.
12. What is the proper way to randomize the goals for the goal expression?
13. ~How to visualize the start, goal, and path on the grid? (See section on graph. Should be similar.)~ This [notebook](/notebooks/A-Star-City.ipynb) has code for that. 


#### *1.2. Questions on the simulator*

1. ~What are the parameters in the simulator? Are they settable?~ PID coefficients, max values, etcetera drone dynamics. Nothing about the view.
2. ~Can the simulator view perspective relative to the drone be changed? Does it have to be relative to the drone?~ The controls work fine and the whole world can be viewed from above. The perspective can be changed with **Pan Camera** and can be zoomed. The camera can also be tilted, yawed, and zoomed. See next question.
3. Is there a drone camera view separate from the simulator observer view?
4. ~What does the simulator show? How does that correspond to the grid?~ The simulator has two reset states which alternate on pressing Shift-R. One shows the city, the other note. The portion of the city almost exactly corresponds to the colliders. There might be some discrepancy, which is usually handled by a 3-meter safety distance.


