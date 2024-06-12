import argparse
import time
from enum import Enum
import numpy as np
import visdom

import sys
sys.path.append("/Users/ka/Documents/Learning/Flying-Cars-and-Autonomous-Flight-Engineer/udacidrone")

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID

class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5


class BackyardFlyer(Drone):

    def __init__(self, connection):
        super().__init__(connection)
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # TODO: Register all your callbacks here
        self.register_callback(MsgID.STATE, self.state_callback)
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        #self.register_callback(MsgID.LOCAL_POSITION, self.update_ne_plot)
        #self.register_callback(MsgID.LOCAL_POSITION, self.update_d_plot)

        # Open a file to log data
        self.log_file = open("Logs/flight_data_log.txt", "w")
        self.log_file.write("timestamp,armed,guided,x,y,z,vx,vy,vz\n")

        # # default opens up to http://localhost:8097
        # self.v = visdom.Visdom()
        # assert self.v.check_connection()

        # # Plot NE
        # ne = np.array([self.local_position[0], self.local_position[1]]).reshape(1, -1)
        # self.ne_plot = self.v.scatter(ne, opts=dict(
        #     title="Local position (north, east)", 
        #     xlabel='North', 
        #     ylabel='East'
        # ))

        # # Plot D
        # d = np.array([self.local_position[2]])
        # self.t = 0
        # self.d_plot = self.v.line(d, X=np.array([self.t]), opts=dict(
        #     title="Altitude (meters)", 
        #     xlabel='Timestep', 
        #     ylabel='Down'
        # ))

    def print_attributes(self):
        attributes = vars(self)
        for attr, value in attributes.items():
            print(f"{attr} = {value}")
    
    # def update_ne_plot(self):
    #     ne = np.array([self.local_position[0], self.local_position[1]]).reshape(1, -1)
    #     self.v.scatter(ne, win=self.ne_plot, update='append')

    # def update_d_plot(self):
    #     d = np.array([self.local_position[2]])
    #     # update timestep
    #     self.t += 1
    #     self.v.line(d, X=np.array([self.t]), win=self.d_plot, update='append')

    def local_position_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.LOCAL_POSITION` is received and self.local_position contains new data
        """
        if self.flight_state == States.TAKEOFF:
        	if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
        		self.all_waypoints = self.calculate_box()
        		self.waypoint_transition()

        elif self.flight_state == States.WAYPOINT:
        	if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 0.1:
        		if len(self.all_waypoints) > 0:
        			self.waypoint_transition()
        		else:
        			if np.linalg.norm(self.local_velocity[0:2]) < 0.1:
        				self.landing_transition()
        x, y, z = self.local_position
        self.log_file.write(f"{time.time()},,,{x},{y},{z},,,\n")

    def velocity_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.LOCAL_VELOCITY` is received and self.local_velocity contains new data
        """
        # MAX_VELOCITY = 1
        # if self.flight_state == States.WAYPOINT:
        #     # Extract current velocity components
        #     vx, vy, vz = self.local_velocity

        #     # Cap the velocities between -MAX_VELOCITY and +MAX_VELOCITY
        #     vx_capped = max(min(vx, MAX_VELOCITY), -MAX_VELOCITY)
        #     vy_capped = max(min(vy, MAX_VELOCITY), -MAX_VELOCITY)
        #     vz_capped = max(min(vz, MAX_VELOCITY), -MAX_VELOCITY)

        #     # Update the drone's velocity using the capped values
        #     current_heading = self._yaw
        #     self.cmd_velocity(vx_capped, vy_capped, vz_capped, current_heading)

        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
            	if abs(self.local_position[2]) < 0.01:
            		self.disarming_transition()
        vx, vy, vz = self.local_velocity
        self.log_file.write(f"{time.time()},,,,,,{vx},{vy},{vz}\n")

    def state_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.STATE` is received and self.armed and self.guided contain new data
        """
        if not self.in_mission:
            return
        if self.flight_state == States.MANUAL:
            self.arming_transition()
        elif self.flight_state == States.ARMING:
        	if self.armed:
        		self.takeoff_transition()
        elif self.flight_state == States.DISARMING:
        	if ~self.armed and ~self.guided:
        		self.manual_transition()
                  
        # Log the armed and guided state
        armed = int(self.armed)  # Convert boolean to int for easier logging
        guided = int(self.guided)  # Convert boolean to int for easier logging
        self.log_file.write(f"{time.time()},{armed},{guided},,,,,,\n")

    def calculate_box(self):
        """TODO: Fill out this method
        
        1. Return waypoints to fly a box
        """
        print("setting waypoints")

        waypoints = [[10.0, 0.0, 3.0], [10.0, 10.0, 3.0], [0.0, 10.0, 3.0], [0.0, 0.0, 3.0]]
        
        return waypoints

    def arming_transition(self):
        """TODO: Fill out this method
        
        1. Take control of the drone
        2. Pass an arming command
        3. Set the home location to current position
        4. Transition to the ARMING state
        """
        print("arming transition")
        self.take_control()
        self.arm()

        # set the current location to be the home position
        self.set_home_position(self.global_position[0],
                               self.global_position[1],
                               self.global_position[2])

        self.flight_state = States.ARMING

    def takeoff_transition(self):
        """TODO: Fill out this method
        
        1. Set target_position altitude to 3.0m
        2. Command a takeoff to 3.0m
        3. Transition to the TAKEOFF state
        """
        print("takeoff transition")
        target_altitude = 3.0
        self.target_position[2] = target_altitude
        self.takeoff(target_altitude)
        self.flight_state = States.TAKEOFF
        print(f"Takeoff Coordinates: Latitude = {self.global_position[0]}, Longitude = {self.global_position[1]}, Altitude = {self.global_position[2]}")

    def waypoint_transition(self):
        """TODO: Fill out this method
    
        1. Command the next waypoint position
        2. Transition to WAYPOINT state
        """
        print("waypoint transition")
        self.target_position = self.all_waypoints.pop(0)
        print("target_position", self.target_position)
        self.cmd_position(*self.target_position, 0.0)
        self.flight_state = States.WAYPOINT

    def landing_transition(self):
        """TODO: Fill out this method
        
        1. Command the drone to land
        2. Transition to the LANDING state
        """
        print("landing transition")
        self.land()
        self.flight_state = States.LANDING
        print(f"Landing Coordinates: Latitude = {self.global_position[0]}, Longitude = {self.global_position[1]}, Altitude = {self.global_position[2]}")

    def disarming_transition(self):
        """TODO: Fill out this method
        
        1. Command the drone to disarm
        2. Transition to the DISARMING state
        """
        print("disarm transition")
        self.disarm()
        self.flight_state = States.DISARMING

    def manual_transition(self):
        """This method is provided
        
        1. Release control of the drone
        2. Stop the connection (and telemetry log)
        3. End the mission
        4. Transition to the MANUAL state
        """
        print("manual transition")

        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

    def start(self):
        """This method is provided
        
        1. Open a log file
        2. Start the drone connection
        3. Close the log file
        """
        print("Creating log file")
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        self.connection.start()
        print("Closing log file")
        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, PX4=False)
    #conn = WebSocketConnection('ws://{0}:{1}'.format(args.host, args.port))
    drone = BackyardFlyer(conn)
    time.sleep(2)
    drone.start()
