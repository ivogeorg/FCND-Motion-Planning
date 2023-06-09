import argparse
import time
import msgpack                  # (ivogeorg) To send waypoints to simulator
from enum import Enum, auto

import numpy as np

from planning_utils import a_star, heuristic, create_grid
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

    def __init__(self, connection):  # TODO (ivogeorg): Global lat-lon start-target coord args!
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
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0: # TODO (ivogeorg): Why?
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
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
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

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

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        # TODO: read lat0, lon0 from colliders into floating point values
        # NOTE (ivogeorg): 
        # Open file, and readline() into a list or strings.
        # List comprehension with float("lat0 37.792480".split()[1]).
        
        # TODO: set home position to (lon0, lat0, 0)
        # NOTE (ivogeorg): 
        # This is self.global_home, so check what's expected (list or tuple).
        # Check how home is used, esp. in relation to start and target.

        # TODO: retrieve current global position
        # NOTE (ivogeorg): 
        # This is self.global_position, so there should be a call to get it.
        # This is in lat, lon.
 
        # TODO: convert to current local position using global_to_local()
        # NOTE (ivogeorg): 
        # This is self.local_position and is a triple.
        
        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        
        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        
        # Define starting point on the grid (this is just grid center)
        # NOTE (ivogeorg): Actually, "origin" instead of "center", since
        # create_grid returns [grid, int(north_min), int(east_min)]
        grid_start = (-north_offset, -east_offset)
        
        # TODO: convert start position to current position rather than map center
        # NOTE (ivogeorg): 
        # Again, "map origin" instead of "map center". This makes sense if there are
        # several flights one after the other. The simulator keeps the "current
        # location" of the drone, so the drone can start where it ended the previous
        # flight. TODO (ivogeorg): Verify.
        # It's very likely that the map origin contains an obstacle at the target altitude

        # TODO (ivogeorg): Funciton closest_grid_node_local() in planning_utils.py to make
        # sure the drone stays within the grid. Using local coordinates (N, E), also
        # minding the altitude. Args: north, east, altitude.
        
        # Set goal as some arbitrary position on the grid
        # TODO (ivogeorg): 
        #       Randomize but position away from obstacles at any altitude.
        #       This may be done in a random sampling loop. Note that this may
        #       include courtyards which may require climbing to the top of
        #       the surrounding building and then descending into the courtyard.
        #       Have to experiment with such goal positions. Also experiement
        #       with spiral descent into courtyards.
        # TODO (ivogeorg):
        #       Global (lat, lon) start and target should be read in from cmd line 
        #       arguments or defaulted, in the constructor. They should be converted
        #       to local and the nearest unobstructed grid points should be
        #       identified. Grid points are local (north, east, altitude).
        grid_goal = (-north_offset + 10, -east_offset + 10)  # TODO (ivogeorg): Use closest_grid_node()
        
        # TODO: adapt to set goal as latitude / longitude position and convert
        # TODO (ivogeorg): 
        #       There should probably be a check that lat/lon are within the
        #       map. These may be added as command-line arguments.
        #       Again, this makes no sense.
        # TODO (ivogeorg): Define function closest_grid_node_global() taking lat, lon
        # and reuse closest_grid_node_local()

        # Run A* to find a path from start to goal
        # DONE (ivogeorg): 
        # Add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)
        print('Local Start and Goal: ', grid_start, grid_goal)
        path, _ = a_star(grid, heuristic, grid_start, grid_goal)
        
        # TODO: prune path to minimize number of waypoints
        # TODO (ivogeorg): Collinearity worked well. How does Brezenham prone?

        # TODO: (if you're feeling ambitious): Try a different approach altogether!
        # TODO (ivogeorg): 
        #       Probabilistic roadmap with receding-horizon local replanning
        #       and smoothing (v.2)

        # Convert path to waypoints
        waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in path]
        # Set self.waypoints
        self.waypoints = waypoints
        # Send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

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

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn)  # TODO (ivogeorg): Global lat-lon start-target coord args! (v.2)
    time.sleep(1)

    drone.start()
