import argparse
import time
import msgpack                  # (ivogeorg) To send waypoints to simulator
from enum import Enum, auto

import numpy as np
from numpy.random import randint

from planning_utils import a_star, heuristic, create_grid, create_grid_flipped, global_position_to_grid_node
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection):  # TODO (ivogeorg): Global lat-lon start-target coord args! (v.2)
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    # TODO (ivogeorg): 1. Parametrize the deadband radius for waypoints.
    # TODO (ivogeorg): 2. For different waypoints (e.g. graph nodes) pick a
    #                     different appropriate deadband radius. For tight
    #                     corners, it should be smaller; for straight streches
    #                     or smooth long turns, it should be larger.
    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(
                    self.target_position[0:2] - 
                    self.local_position[0:2]) < 1.0: 
                    # TODO (ivogeorg): Deadband radius
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0: 
                        # TODO (ig): Why?
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1: # TODO (ig): Should land at target, not only home
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(
                self.target_position[0], self.target_position[1], 
                self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False
        print("Local position: ", self.local_position)

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        # DONE (ivogeorg): read lat0, lon0 from colliders into floating point
        # values
        # NOTE 13 (ivogeorg): 
        # First line of colliders file is "lat0 37.792480, lon0 -122.397450". 
        # This is the global position of the center of the grid.
        latlon = ""
        with open("colliders.csv", "r") as f:
            latlon = f.readline()
        lat0, lon0 = latlon.split(", ")
        lat0 = float(lat0.split()[1])
        lon0 = float(lon0.split()[1])
        
        # DONE: set home position to (lon0, lat0, 0)
        # NOTE 14 (ivogeorg):
        # Home position is used by `udacidrone.frame_utils.global_to_local`
        # to convert global (lon, lat, up) coordinates to local (nor, east, 
        # down) coordinates relative to the home location.
        # NOTE 15 (ivogeorg):
        # The simulator initializes the drone at this location by default, and
        # works with meters relative to it. By the logic of `global_to_local`,
        # this is local position zero (0., 0., 0.). The colliders data is all
        # in meters relative to this zero location.
        self.set_home_position(lon0, lat0, 0.0)

        # DONE: retrieve current global position
        # NOTE 16 (ivogeorg): 
        # global_position is a property of of udacidrone.Drone
        # This is in lon, lat, alt (or up).
 
        # DONE: convert to current local position using global_to_local()
        # NOTE 17 (ivogeorg): 
        # local_position is also a property of udacidrone.Drone, so it is
        # not settable and is maintained along with global_position.
        # This is in north, east, down.
        
        print('Global home : ', self.global_home)
        print('Global position: ', self.global_position)
        print('Local position: ', self.local_position)

        # Read in obstacle map
        data = np.loadtxt(
                'colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        
        # Define a grid for a particular altitude and safety margin around 
        # obstacles

        # DEBUG (ivogeorg): 
        # Drone path crosses buildings!!!
        # grid, north_offset, east_offset = \
        #     create_grid_flipped(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        grid, north_offset, east_offset = \
            create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        # NOTE 18 (ivogeorg):
        # create_grid_flipped flips axis 0 of grid (or, up-down) to conform to
        # the frame of the simulator.
        # NOTE 19 (ivogeorg):
        # The grid is a 971 by 971 points at a resolution of 1 meter and only
        # whole numbers. The origin of the grid (0, 0), after flipping, is the
        # bottom-left (SW) corner. The offsets returned by create_grid_flipped
        # define its location (in meters) relative to local position (0,0,0).
        # NOTE 20 (ivogeorg):
        # The simulator works with waypoints in meters relative to global home
        # and local position (0., 0., 0.) at the center of the world (and 
        # grid). For example, a waypoint of (20, 10, 0, 0) is 20 m north and 
        # 10 m east of this position, and thus at a grid position 
        # (north_offset + 20, east_offset + 10).
        print("North offset = {0}, East offset = {1}".format(north_offset, 
                                                            east_offset))
        
        # Define starting point on the grid (this is just grid center)
        # grid_start = (-north_offset, -east_offset)
        # NOTE 21 (ivogeorg): 
        # Due to the rounding to integers in create_grid_flipped, there aren't
        # unique global or local position tuples for the nodes. Instead, there 
        # are circles (in 2D) and spheres (in 3D) around each node the 
        # coordinates of which round to the node.
        
        # DONE: convert start position to current position rather than map 
        # center
        # NOTE 22 (ivogeorg): 
        # This makes sense if there are several flights one after the other. 
        # The simulator, if not reset, keeps the current location of the 
        # drone, so the drone will start where it ended the previous flight. 
        # Verified with starter code and "zig-zag" trajectory.
        # NOTE 23 (ivogeorg):
        # In the likely event that the drone didn't land or is otherwise not
        # situated exactly on top of a grid node, this has to pass through
        # global_position_to_grid_node. This function will be the workhorse of
        # the scenario where start and goal are given in global coordinates
        # (lon, lat, alt).
        print('Starting from current position')
        grid_start = global_position_to_grid_node(
                        self.global_position, self.global_home, 
                        grid, TARGET_ALTITUDE, 
                        north_offset, east_offset)

        # DONE: Set goal as some arbitrary position on the grid
        # NOTE 24 (ivogeorg): 
        # When a_star as originally written in mapping_utils is
        # given a grid, it can only find paths between two grid nodes. Start and
        # goal positions off the grid cannot be on the path, unless a_star is
        # modified. a_star can also work with a graph, as long as start and goal
        # are graph nodes. Of course, the drone can freely fly off the grid!
        # NOTE 25 (ivogeorg):
        # The goal position is randomized within the following
        # 4 corners (approximate (lon, lat)): 
        # SW (-122.402424, 37.789649), NW (-122.402424, 37.797903), 
        # NE (-122.392115, 37.797903), SE (-122.392115, 37.789649)
        goal_global_lat = np.random.uniform(37.789649, 37.797903)
        goal_global_lon = np.random.uniform(-122.392115, -122.402424)
        grid_goal = global_position_to_grid_node(
                        (goal_global_lon, goal_global_lat, 0.0), 
                        self.global_home, 
                        grid, TARGET_ALTITUDE, 
                        north_offset, east_offset)

        # TODO (ivogeorg):
        #       Global (lon, lat) start and target should be read in from cmd line 
        #       arguments or defaulted, in the constructor. They should be converted
        #       to local and the nearest unobstructed grid points should be
        #       identified. Grid points are local (north, east, altitude). (v.2)

        # grid_goal = (-north_offset + 20, -east_offset - 11)
        
        # DONE: adapt to set goal as latitude / longitude position and convert
 
        # Run A* to find a path from start to goal
        # DONE (ivogeorg): 
        # Add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)
        print('Local Start and Goal: ', grid_start, grid_goal)
        path, _ = a_star(grid, heuristic, grid_start, grid_goal)
        print("Path length: ", len(path))
        # TODO (ivogeorg): If path is None, should gracefully shut down.
        # `plan_path` should therefore return a boolean to indicate 
        # success of failure. This should be treated accordingly in the
        # transitions code.
        
        # TODO: prune path to minimize number of waypoints
        # TODO (ivogeorg): Function prune_path() in planning_utils.py.
        # Collinearity worked well. How does Brezenham prune?
        # With diagonal actions Brezenham's utility diminishes.

        # TODO: (if you're feeling ambitious): Try a different approach altogether!
        # TODO (ivogeorg): 
        #       Probabilistic roadmap with receding-horizon local replanning
        #       and smoothing (v.2)

        # Convert path to waypoints
        waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in path]
        # NOTE (ivogeorg): For this to make sense it means that the simulator works in
        # meters relative to global home and local position zero (0, 0, 0)!

        # Set self.waypoints
        self.waypoints = waypoints
        # Send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")  
        
        # TODO (ivogeorg):
        # Examine the logs:
        # 1. What messages are sent, when, and how frequently?
        # 2. Study which callbacks are called when.
        # 3. What is the best place for each state transition?

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()

# TODO (ivogeorg): 
#       Command-line argument for planner version (1-grid, 2-graph, 3-receding 
#       horizon w/ replanning) (v.2)
# TODO (ivogeorg): 
#       Version 3 built on 3 layers (v.3):
#       1 - approximate global, 
#       2 - receding horizon local replanning (incl. vertical search for 
#           couryard descent), 
#       3 - dynamic obstacle avoidance (incl. wind)
# TODO: (ivogeorg):
#       The now optional [Python controller](https://github.com/udacity/FCND-Controls) 
#       project from the next FCND course (Controls) may contain a richer 
#       sensorium which can help with the implementation of the more advanced 
#       motion planning. (v.2)
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()
    # TODO (ivogeorg):
    #       Global (lat, lon) start and target should be read in from cmd line 
    #       arguments or defaulted, in the constructor. They should be converted
    #       to local and the nearest unobstructed grid points should be
    #       identified. Grid points are local (north, east, altitude). (v.2)

    # conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    # NOTE 53 (ivogeorg):
    # Allow A* to find long paths
    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=1800)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
