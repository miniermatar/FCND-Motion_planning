import argparse
import time
import msgpack
from enum import Enum, auto
import re
import numpy as np

from planning_utils import generate_path
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

    def __init__(self, connection):
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

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            #deadband is adjusted based on drone velocity
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < (0.1* np.linalg.norm(self.local_velocity[0:2])+0.2):
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()


    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if np.linalg.norm(self.local_velocity[0:2]) < 0.001:
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
        time.sleep(2) # 2 seconds delay to start landing and reading velocity

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
        
        #loading data for home position
        f_line= open("colliders.csv", "r")
        start_loc = re.findall("[-\d]+\.\d+",f_line.readline())
        lat=float(start_loc[0])
        lon=float(start_loc[1])
        f_line.close()
        
        self.set_home_position(lon,lat,0)  

        north,east,down = global_to_local(self.global_position, self.global_home)
        print('global home {0}, \nglobal position {1}, \nlocal position {2}'.format(self.global_home, self.global_position, self.local_position))
    
        start = north,east,down 
        #Target altiture is 5 m above current position. In case drone is on top of a building
        TARGET_ALTITUDE = -down+ 5
        safety_distance = 5

        self.target_position[2] = TARGET_ALTITUDE
        
        print("Generating waypoints with headings...")
        waypoints=generate_path(start,safety_distance,self.global_home)
        if waypoints==[0,0,0,0]:
            print ("\nNo path found from start posistion. Move drone to another location\n")
            self.disarming_transition()
        else:
            self.waypoints = waypoints

            self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        while self.in_mission:
            pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port),threaded=False, timeout=1000)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
