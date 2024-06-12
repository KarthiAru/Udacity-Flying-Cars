import argparse
import time
import msgpack
from enum import Enum, auto
import numpy as np
import re
from matplotlib import pyplot as plt
import networkx as nx
from planning_utils import create_grid, a_star, heuristic, prune_path
from planning_utils import create_graph, a_star_graph, closest_point
import math

import sys
sys.path.append("/Users/ka/Documents/Learning/Flying-Cars-and-Autonomous-Flight-Engineer/udacidrone")

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local, local_to_global


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(
        self, connection, global_goal_position=None, mode=0, allow_diagonal=0, prune=0
    ):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL
        self.global_goal_position = global_goal_position
        self.mode = mode
        self.allow_diagonal = allow_diagonal
        self.prune = prune

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if (
                np.linalg.norm(self.target_position[0:2] - self.local_position[0:2])
                < 1.0
            ):
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
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
        # if self.global_position[0] == 0.0 and self.global_position[1] == 0.0:
        #     print("no global position data, wait")
        #     return
        self.arm()
        self.take_control()
        # self.set_home_position(self.global_position[0], self.global_position[1],
        #                        self.global_position[2])  # set the current location to be the home position

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print("target position", self.target_position)
        self.cmd_position(
            self.target_position[0],
            self.target_position[1],
            self.target_position[2],
            self.target_position[3],
        )

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
        TARGET_ALTITUDE = 4
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        # TODO: read lat0, lon0 from colliders into floating point values
        print("\nLoading map data from file ... ")
        # Read in obstacle map
        data = np.loadtxt("colliders.csv", delimiter=",", dtype="float64", skiprows=2)
        header = open("colliders.csv").readline()
        print("Done ✅\n")
        s = re.findall(r"[-+]?\d*\.\d+|\d+", header)
        lat0 = float(s[1])
        lon0 = float(s[3])
        # print(f'Home lat : {lat0}, lon : {lon0}')

        # TODO: set home position to (lon0, lat0, 0)
        self.set_home_position(lon0, lat0, 0)

        # TODO: retrieve current global position
        print(
            "current global home: north = %.2f, east = %0.2f, down = %0.2f"
            % (
                self.global_position[0],
                self.global_position[1],
                self.global_position[2],
            )
        )

        # TODO: convert to current local position using global_to_local()
        local_north, local_east, local_down = global_to_local(
            self.global_position, self.global_home
        )

        # Define a grid for a particular altitude and safety margin around obstacles
        
        start_time = time.time()
        if self.mode == 1:
            print("\nCreating Graph ... (this may take ~60s)")
            grid, edges, north_offset, east_offset = create_graph(
                data, TARGET_ALTITUDE, SAFETY_DISTANCE
            )
            graph = nx.Graph()
            for e in edges:
                p1 = e[0]
                p2 = e[1]
                dist = np.linalg.norm(np.array(p2) - np.array(p1))
                graph.add_edge(p1, p2, weight=dist)
        else:
            print("Creating Grid ... ")
            grid, north_offset, east_offset = create_grid(
                data, TARGET_ALTITUDE, SAFETY_DISTANCE
            )
        end_time = time.time()
        print(f"Elapsed time: {end_time - start_time:.1f} seconds")
        print("Done ✅\n")
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))

        # Define starting point on the grid (this is just grid center)
        # grid_start = (-north_offset, -east_offset)

        # TODO: convert start position to current position rather than map center
        grid_start = (
            int(np.ceil(local_north - north_offset)),
            int(np.ceil(local_east - east_offset)),
        )

        # Set goal as some arbitrary position on the grid
        # grid_goal = (-north_offset + 10, -east_offset + 10)
        # TODO: adapt to set goal as latitude / longitude position and convert
        goal_north, goal_east, goal_alt = global_to_local(
            self.global_goal_position, self.global_home
        )
        grid_goal = (
            int(np.ceil(goal_north - north_offset)),
            int(np.ceil(goal_east - east_offset)),
        )

        if self.mode == 1:
            grid_start = closest_point(graph, grid_start)
            grid_goal = closest_point(graph, grid_goal)

        # Run A* to find a path from start to goal
        # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)
        print("Local Start and Goal: ", grid_start, grid_goal)

        start_time = time.time()
        if self.mode == 1:
            print("\nSearching for path ... ")
            path, cost = a_star_graph(graph, heuristic, grid_start, grid_goal)
        else:
            print("\nSearching for path ... (this may take ~60s)")
            path, cost = a_star(
                grid, heuristic, grid_start, grid_goal, self.allow_diagonal
            )
        end_time = time.time()

        print(f"Elapsed time: {end_time - start_time:.1f} seconds")
        print(f"Length of path: {len(path)}")
        print("Done ✅\n")

        # TODO: prune path to minimize number of waypoints
        # TODO (if you're feeling ambitious): Try a different approach altogether!
        if self.prune:
            print("Pruning path ... ")
            path = prune_path(path)
            print(f"Length of pruned path: {len(path)}")
            print("Done ✅\n")

        # # Plot Map
        # plt.imshow(grid, origin="lower", cmap="Greys")
        # # Plot the path with lines connecting the points
        # if path is not None and len(path) > 1:
        #     pp = np.array(path)  # Use the full path
        #     plt.plot(pp[:, 1], pp[:, 0], color="g")  # Draw lines
        #     pp_mid = np.array(path[1:-1])
        #     plt.scatter(pp_mid[:, 1], pp_mid[:, 0], color="red", s=6)  # Plot way points

        # # Mark the start location with a green "x"
        # plt.plot(
        #     path[0][1],
        #     path[0][0],
        #     color="green",
        #     markersize=6,
        #     marker="x",
        #     label="Start",
        # )
        # # Mark the end location with a blue "d"
        # plt.plot(
        #     path[-1][1],
        #     path[-1][0],
        #     color="blue",
        #     markersize=6,
        #     marker="d",
        #     label="Goal",
        # )

        # plt.legend(loc="best")

        # plt.xlabel("EAST")
        # plt.ylabel("NORTH")
        # text_str = f"Time: {(end_time - start_time):.1f}s, Points: {len(path)}, Cost: {cost:.1f}"
        # plt.text(
        #     0.05,
        #     0.95,
        #     text_str,
        #     transform=plt.gca().transAxes,
        #     verticalalignment="top",
        #     fontsize=9,
        #     bbox=dict(facecolor="white", alpha=0.75),
        # )

        # plt.savefig("images/example_plot.png")

        # Convert path to waypoints
        # waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in path]
        waypoints = [
            [
                math.ceil(p[0]) + north_offset,
                math.ceil(p[1]) + east_offset,
                TARGET_ALTITUDE,
                0,
            ]
            for p in path
        ]
        # Set self.waypoints
        self.waypoints = waypoints
        print(f"Length of waypoints: {len(waypoints)}")
        # TODO: send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        try:
            self.connection.start()
            self.stop_log()
        except ConnectionResetError as err:
            return err, self.waypoints

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=5760, help="Port number")
    parser.add_argument(
        "--host", type=str, default="127.0.0.1", help="host address, i.e. '127.0.0.1'"
    )
    parser.add_argument(
        "--mode",
        type=int,
        default=0,
        help="Choose 0 for Grid, 1 for Graph based search",
    )
    parser.add_argument(
        "--allow_diagonal", type=int, default=1, help="Allow drone to move diagonally"
    )
    parser.add_argument("--prune", type=int, default=1, help="Prune points in the path")
    parser.add_argument(
        "--target",
        type=float,
        nargs=3,
        default=[-122.39743738, 37.79263316, -0.147],
        help="Target location (lon, lat, alt)",
    )
    args = parser.parse_args()

    conn = MavlinkConnection("tcp:{0}:{1}".format(args.host, args.port), timeout=60)
    drone = MotionPlanning(
        conn,
        global_goal_position=np.array(args.target),
        mode=args.mode,
        allow_diagonal=args.allow_diagonal,
        prune=args.prune,
    )
    time.sleep(1)

    drone.start()
