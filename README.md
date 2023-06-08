# FCND Project 2: Motion Planning

Planning in the complex urban environment of San Francisco using the `udacidrone` API and the Unity-based Udacity simulator. A [guide](project-and-submission-guide.md) describes the project setup and the submission requirements. The code-writing **TODOs**, both required and optional, are all inlined in [motion_planning.py](motion_planning.py).

## Starter Code

The overall architecture of this project is (i) user code contained in a sublclass of the `udacidrone.Drone` class, (ii) the `udacidrone` API serving as an interface between high-level user planning and control and the simulator, mediated via a two-way MAVLink protocol asynchronous message stream, and (iii) the Unity-based simulator that contains the urban world specification and the low-level dynamics and control model of the drone.

### 1. Planning Subclass

The majority of the functionality of the drone is encapsulated in a the subclass `MotionPlanning(Drone)` where the parent `Drone` is defined in the [`udacidrone` package](#2-udacitdronedrone). 

#### 1.1. File `motion_planning.py`

The drone is defined roughly as a state machine, with a `state_callback` method calling a handler method for each state-to-state transition. The states are `MANUAL`, `ARMING`, `TAKEOFF`, `WAYPOINT`, `LANDING`, `DISARMING`, AND `PLANNING`. When the drone is armed (i.e. is in state `ARMING`) it calls `plan_path` and enters the state `PLANNING`. After planning, the drone takes off (state `TAKEOFF`) and positions itself at the start position. The `local_position_callback` transitions the drone to state `WAYPOINT` and calls the `waypoint_transition` method. The drone starts following the planned path (state `WAYPOINT`).

`plan_path` contains the main part of the difference from `BackyardFlyer`, which had a simple predefined square trajectory to follow. `MotionPlanning` is able to fly arbitrary trajectories inside a complex urban environment from between given start and stop positions. The method takes a text file containing collision coordinates for downtown San Francisco and then creates and lays over a grid over the environment out of which waypoint trajectories will be picked to navigate the drone around it. Given a start and goal positions from the grid, the heuristic A* search is used to find a path between them. After pruning and further finetuning of the path, the path is sent to the simulator, and the drone sets off to complete it.

`motion_planning.py` is the executable that is run to plan and fly the `MotionPlanning` drone in the [simulator](#4-simulator).

#### 1.2. File `motion_utils.py`

The file contains helper methods for creating and navigating a grid-based set of trajectories:

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
2. In the `valid_actions` method:
   ```
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
3. In the simulator, the zig-zag trajectory is now a straight line:
   **TODO**

### 2. 
