import dronekit
from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative
import numpy as np
from math import *
import threading
import random
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
from shapely.geometry import LineString
from helper import *
import sys
import socket
import operator
import time
import logging

class SwarmBot:

    def __init__(self, s,sysid):
        """
        vehicle: dronekit object
        velocity: vector
        s: string sysid
        wplist: list of waypoints
        States- 1: payload present and not dropping
                0: payload present dropping
                -1: payload not present
        """
        self.vehicle = dronekit.connect(s)
        # self.neighbors = []  list of neighbor ids, use the shared memory to access the neighbor data
        self.__velocity = self.vehicle.velocity
        self.__pos = [self.vehicle.location.global_frame.lat, self.vehicle.location.global_frame.lon]
        self.id = sysid
        print("Connected to vehicle id : " + str(self.id))
        self.__wplist = []
        self.__payload = 1
        self.__gbestloc = [0, 0]
        self.__bestlocation = [0, 0]
        self.__gbest = 0
        self.__pbest = 0
        self.__droplocation = [0, 0, 0]
        self.__personal_humans = []
        self.__global_humans = []
        self.__futurepayload = []
        self.mopso = 0
        self.__newloc = []
        self.__droppedlocation = []
        self.v_max = 5
        self.v_min = 3
        self.collision = 0
        logging.basicConfig(filename="SwarmBot.log", format="%(module)s %(lineno)d %(message)s", filemode='w')
        self.logger = logging.getLogger()
        self.logger.setLevel(logging.INFO)

        @self.vehicle.on_message('*')
        def my_method(self, name, msg):
            msg = str(msg)
            if "Arm:" in msg:
                print(msg)
                self.logger.info("# ================================================================ #")
                self.logger.info("#                     Error occured in arming: " + str(msg))
                self.logger.info("# ================================================================ #")
            else:
                pass

    def get_pos(self):
        """
        returns: current position of SwarmBot object
        """
        self.__pos = [self.vehicle.location.global_frame.lat, self.vehicle.location.global_frame.lon]
        return self.__pos

    def get_alt(self):
        """
        returns: current relative altitude of SwarmBot object
        """
        return self.vehicle.location.global_relative_frame.alt

    def get_vel(self):
        """
        returns: current velocity vector of SwarmBot Object
        """
        return self.vehicle.velocity

    def update_Leftout_PD_location(self, newloc):
        self.__newloc = newloc

    def get_gbestloc(self):
        return self.__gbestloc

    def get_PersonalHumans(self):
        return self.__personal_humans

    def update_PersonalHumans(self, humans,distance_between_two_humans):
        temp_list=[]
        print(self.__personal_humans,"before")
        print(distance_between_two_humans)
        if len(self.__personal_humans)>0:

            for j in self.__personal_humans:
                if get_distance(j[0],j[1],humans[0],humans[1])<distance_between_two_humans:
                    temp_list.append(humans)
            if temp_list==[]:
                self.__personal_humans.append(humans)

        else:
            self.__personal_humans.append(humans)

    def get_GlobalHumans(self):
        return self.__global_humans

    def update_GlobalHumans(self, humans):
        self.__global_humans.append(humans)

    def get_droplocation(self):
        return self.__droplocation

    def get_newloc(self):
        return self.__newloc

    def get_state(self):
        return self.__payload

    def get_bestlocation(self):
        return self.__bestlocation

    def get_gbest(self):
        return self.__gbest

    def get_pbest(self):
        return self.__pbest

    def get_futurepayload(self):
        return self.__futurepayload

    def get_sys_id(self):
        return self.id

    def waypoints(self):
        return self.__wplist

    def heading(self):
        """
        returns: Swarm Objects heading
        """
        return self.vehicle.heading

    def altitude(self):
        """
        returns: Swarm Object altitude
        """
        return self.vehicle.location.global_relative_frame.alt

    def update_pos(self, pos, v):
        """
        v: goto velocity
        pos: final position
        """
        # self.position=pos
        pos_x = pos[0]
        pos_y = pos[1]
        pos_z = pos[2]
        self.vehicle.simple_goto(LocationGlobalRelative(pos_x, pos_y, pos_z), v, v)
        self.__pos = [self.vehicle.location.global_frame.lat, self.vehicle.location.global_frame.lon]

    def update_bearing(self,heading_of_wp):
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
            0, #confirmation
            heading_of_wp,    # param 1, yaw in degrees
            0,          # param 2, yaw speed deg/s
            1,          # param 3, direction -1 ccw, 1 cw
            0,          # param 4, relative offset 1, absolute angle 0
            0, 0, 0)    # param 5 ~ 7 not used
        # send command to vehicle
        self.vehicle.send_mavlink(msg)

    def update_vel(self, v):
        """
        v: new velocity vector
        """
        self.__velocity = v
        velocity_x = v[0]
        velocity_y = v[1]
        velocity_z = v[2]
        msg = self.vehicle.message_factory.set_position_target_global_int_encode(
            0,  # time_boot_ms (not used)
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # frame
            0b0000111111000111,  # type_mask (only speeds enabled)
            0,  # lat_int - X Position in WGS84 frame in 1e7 * meters
            0,  # lon_int - Y Position in WGS84 frame in 1e7 * meters
            0,  # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
            # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
            velocity_x,  # X velocity in NED frame in m/s
            velocity_y,  # Y velocity in NED frame in m/s
            velocity_z,  # Z velocity in NED frame in m/s
            0, 0, 0,  # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

        self.vehicle.send_mavlink(msg)

    def update_waypoints(self, p):
        """
        p: list of locations to be added
        """
        self.__wplist = []
        for element in p:
            self.__wplist.append(element)
        # return self.__wplist

    def arm_and_takeoff(self, aTargetAltitude, var):
        """
        while self.vehicle.mode.name !="STABILIZE" :
            print("Changing the vehicle mode to STABALIZE")
            self.vehicle.mode = dronekit.VehicleMode("STABILIZE")
            time.sleep(1)
        """
        while not self.vehicle.is_armable and self.vehicle.mode.name not in ["RTL","LAND"] :
            print(" Waiting for vehicle to initialise...", self.id)
            time.sleep(1)

        print("Arming motors", self.id)

        # Copter should arm in GUIDED mode
        while self.vehicle.mode.name != "GUIDED":
            print("Changing the vehicle mode to GUIDED")
            self.vehicle.mode = dronekit.VehicleMode("GUIDED")
            time.sleep(1)

        self.vehicle.armed = True
        self.vehicle.flush()

        while not self.vehicle.armed and self.vehicle.mode.name not in ["RTL","LAND"]:
            print(" Waiting for arming...", self.id)
            self.vehicle.armed = True
            self.vehicle.flush()
            time.sleep(1)

        ## ====================================================================== ##
        ##                           ARM AND TAKEOFF TESTING                      ##
        ## ====================================================================== ##
        if var == 1:
            self.vehicle.simple_takeoff(aTargetAltitude)
        elif var == 2:
            while not self.vehicle.location.global_relative_frame.alt > 5 and self.vehicle.mode.name not in ["RTL","LAND"]:
                self.vehicle.simple_takeoff(aTargetAltitude)
        elif var == 3:
            print("doing this")
            takeoff_msg = self.vehicle.message_factory.command_long_send(
                            0, 0, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                            0, 0, 0, 0, 0, 0, 0, aTargetAltitude)

        while True and self.vehicle.mode.name not in ["RTL","LAND"]:
            print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)
            if self.vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.90:
                print("Reached target altitude", self.id)
                break
            time.sleep(1)


    def land(self):
        self.vehicle.mode = dronekit.VehicleMode("LAND")

    def mode(self, mode_name):
        self.vehicle.mode = dronekit.VehicleMode(mode_name.upper())
    def get_mode(self):
        return self.vehicle.mode.name.upper()

    def drop(self):
        """
        update state of payload
        """
        self__payload = -1

    def generate_mopso_velocity(self, timer, finalwaypoints, num, TMAX, wstart, wend, GlobalUavData):

        vel = [0, 0, 0]

        R1 = (random.randrange(0, 100, 10) / 100)
        R2 = (random.randrange(0, 100, 10) / 100)
        R3 = (random.randrange(0, 100, 10) / 100)
        w = (wstart - (wstart - wend) * ((timer / TMAX) ** 2))

        if self.__gbestloc[0] and not self.__bestlocation[0]:
            print("gthing")
            vel[0] = w * (self.__velocity[0]) + R3 * 1000 * (
                        finalwaypoints[self.id - 1][0] - self.get_pos()[0]) + R2 * 1000 * (
                                 self.__gbestloc[0] - self.get_pos()[0])
            vel[2] = 0
            vel[1] = w * (self.__velocity[1]) + R3 * 1000 * (
                        finalwaypoints[self.id - 1][1] - self.get_pos()[1]) + R2 * 1000 * (
                                 self.__gbestloc[1] - self.get_pos()[1])

        elif not self.__gbestloc[0] and self.__bestlocation[0]:
            print("bthing")
            # print(R3*1000*(finalwaypoints[self.id-1][0]-self.get_pos()[0]),R2*1000*(self.__gbest[0]-self.get_pos()[0]),R1*1000*(self.__bestlocation[1]-self.get_pos()[1]) )
            vel[0] = w * (self.__velocity[0]) + R3 * 1000 * (
                        finalwaypoints[self.id - 1][0] - self.get_pos()[0]) + R1 * 1000 * (
                                 self.__bestlocation[0] - self.get_pos()[0])
            vel[2] = 0
            vel[1] = w * (self.__velocity[1]) + R3 * 1000 * (
                        finalwaypoints[self.id - 1][1] - self.get_pos()[1]) + R1 * 1000 * (
                                 self.__bestlocation[1] - self.get_pos()[1])

        elif not self.__bestlocation[0] and not self.__gbestloc[0]:
            print("nthing")
            vel[0] = w * (self.__velocity[0]) + R3 * 1000 * (finalwaypoints[self.id - 1][0] - self.get_pos()[0])
            vel[2] = 0
            vel[1] = w * (self.__velocity[1]) + R3 * 1000 * (finalwaypoints[self.id - 1][1] - self.get_pos()[1])
        else:
            print("everything")
            vel[0] = w * (self.__velocity[0]) + R3 * 1000 * (
                        finalwaypoints[self.id - 1][0] - self.get_pos()[0]) + R2 * 1000 * (
                                 self.__gbestloc[0] - self.get_pos()[0]) + R1 * 1000 * (
                                 self.__bestlocation[0] - self.get_pos()[0])
            vel[2] = 0
            vel[1] = w * (self.__velocity[1]) + R3 * 1000 * (
                        finalwaypoints[self.id - 1][1] - self.get_pos()[1]) + R2 * 1000 * (
                                 self.__gbestloc[1] - self.get_pos()[1]) + R1 * 1000 * (
                                 self.__bestlocation[1] - self.get_pos()[1])

        vel = self.inter_uav_collision(vel, GlobalUavData)

        v_mopso = inside_circle(vel, self.v_max, self.v_min)

        return v_mopso
    def velocity_post_drop(self, GlobalUavData):
        """
        """
        vel = [0, 0, 0]
        print("payload_dropping")
        curr_vel = self.__velocity
        vel[0] = 100000 * (self.__droplocation[0] - (self.get_pos()[0]))
        vel[1] = 100000 * ((self.__droplocation[1]) - (self.get_pos()[1]))
        vel[2] = 0
        vel = self.inter_uav_collision(vel, GlobalUavData)
        vc = inside_circle(vel, self.v_max, self.v_min)

        return vc
    
    def inter_uav_collision(self, vel, GlobalUavData):
        """
        """
        for friend in GlobalUavData:

            if self.id != int(friend):

                friend_location = GlobalUavData[friend]['GEOLOCATION']
                current_location = self.get_pos()

                dist = get_distance(friend_location[0], friend_location[1], current_location[0], current_location[1])

                if dist < 30:
                    print(dist, self.id, friend)

                if dist < 5:
                    print("inter uav collision aviodance triggered")
                    vel[0] += 100000 * (-friend_location[0] + current_location[0])
                    vel[1] += 100000 * (-friend_location[1] + current_location[1])
                    vel[2] = 0

                if dist < 0.5:
                    self.collision = 1
            

        return vel

    def checkifpayload(self):

        if (get_distance(self.__pos[0], self.__pos[1], self.__droplocation[0], self.__droplocation[1])) <= 9:
            print("payload dropped")
            self.__payload = -1
            self.__dropped_location = self.__droplocation
            # self.__droplocation = [0, 0 ,0]

    # confirm with aman
    def updategbest(self, GlobalUavData, nearestuav):
        """
        """

        g_best = [(self.__gbest, self.__gbestloc)]
        p_best = [(self.__pbest, self.__bestlocation)]
        newloc = [self.__bestlocation]
        for friend in nearestuav:
            g_best.append((GlobalUavData[friend]["G"], GlobalUavData[friend]["GBESTLOC"]))
            p_best.append((GlobalUavData[friend]["P"], GlobalUavData[friend]["BESTLOC"]))

        max_gfriend = max(g_best)
        max_pfriend = max(p_best)
        if max_gfriend < max_pfriend:
            g_best = max_pfriend[0]
            newloc = max_pfriend[1]
        else:
            g_best = max_gfriend[0]
            newloc = max_gfriend[1]

        self.__gbest = g_best
        self.__gbestloc = newloc

    def change_gpbest(self, finalwaypoints, p1, p4, SAL):
        """
        """
        heading_gbest = bearing(self.get_pos()[0], self.get_pos()[1], self.__gbestloc[0], self.__gbestloc[1])
        heading_bestloc = bearing(self.get_pos()[0], self.get_pos()[1], self.__bestlocation[0], self.__bestlocation[1])
        heading = radians(float(bearing(p1[0], p1[1], p4[0], p4[1])))
        heading_gbest = radians(float(heading_gbest))
        heading_bestloc = radians(float(heading_bestloc))

        if degrees(heading_gbest) > degrees(heading + pi / 2 + pi / 12) or degrees(heading_gbest) < degrees(
                heading - pi / 2 - pi / 12):
            # nearest = self.minimumdistance()

            self.__gbest = 0
            self.__gbestloc = [finalwaypoints[self.id - 1][0], finalwaypoints[self.id - 1][1]]

        if degrees(heading_bestloc) > degrees(heading + pi / 2 + pi / 12) or degrees(heading_bestloc) < degrees(
                heading - pi / 2 - pi / 12):
            self.__pbest = 0
            self.__bestlocation = [finalwaypoints[self.id - 1][0], finalwaypoints[self.id - 1][1]]
        if get_distance(self.__pos[0],self.__pos[1],finalwaypoints[self.id - 1][0], finalwaypoints[self.id - 1][1])<0.15*SAL:
            self.__pbest=1
            self.__gbest=1
            self.__bestlocation = [0,0]
            self.__gbestloc = [0,0]

  

    def minimumdistance(self, GlobalUavData, humans):
        friend_distances = []
        #print(humans)
        current_position = GlobalUavData[str(humans[3])]["GEOLOCATION"]
        for friends in GlobalUavData:
            friend_location = GlobalUavData[friends]["GEOLOCATION"]
            friend_distances.append((get_distance(current_position[0], current_position[1], friend_location[0],
                                              friend_location[1]), friends))
        pso_friends = []
        friend_distances.sort()
        for element in friend_distances[:5]:
            pso_friends.append(element[1])

        return pso_friends  # list of uav ids



    def Modified_Filter(self, human_list,distance_between_two_humans):
        temp_list=[]
        temp_list2=[]
        for j in human_list:#filter3 to stop already dtected humans
            if len(self.get_GlobalHumans())>0:
                #print("Printing global ",self.get_GlobalHumans())
                for element in self.get_GlobalHumans():
                    if get_distance(j[0],j[1],element[0],element[1])<distance_between_two_humans:
                        temp_list.append(j)
                #print(temp_list,"temp_list")
                if j not in temp_list:
                    #print(j)
                    #print("Printing global 1 ",self.get_GlobalHumans())
                    temp_list2.append(j)
                    self.update_GlobalHumans(j)
                    #print("Printing global 2 ",self.get_GlobalHumans())
            else:
                self.update_GlobalHumans(j)
                temp_list2.append(j)


                
        return temp_list2

    def update_pbest(self,detected_humans):
        #print(detected_humans,"lllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllll")
        detected_humans.append([self.__bestlocation[0],self.__bestlocation[1],self.__pbest])
        detected_humans.sort(key=operator.itemgetter(2),reverse=True)
        self.__bestlocation=[detected_humans[0][0],detected_humans[0][1]]
        self.__pbest=detected_humans[0][2]



    def payload_drop(self, GlobalUavData, loc, fol):
        friend_distances = {i: sys.maxsize for i in range(1, 1 + len(GlobalUavData))}
        newloc = []
        for friend in GlobalUavData:
            friend_location = GlobalUavData[friend]["GEOLOCATION"]
            dist = get_distance(loc[0], loc[1], friend_location[0], friend_location[1])
            print(2.5*fol)

            if GlobalUavData[friend]["PAYLOAD"] == 1 and dist <= 2.5 * fol:
                print("check1")
                friend_distances[int(friend)] = dist
            if GlobalUavData[friend]["PAYLOAD"] == 0 and dist <= 2.5 * fol and get_distance(GlobalUavData[friend]["DROPLOCATION"][0], GlobalUavData[friend]["DROPLOCATION"][1],friend_location[0], friend_location[1]) > get_distance(loc[0], loc[1], friend_location[0],friend_location[1]):
                print("check2")
                friend_distances[int(friend)] = dist

        min_dist = min(friend_distances.values())
        if min_dist != sys.maxsize:
            y = list(friend_distances.items())
            print(y)
            critical_key = min(y, key=lambda y: y[1])[0]
            print(critical_key,"critical_key")
            
            if int(critical_key) == int(self.id):
                # drop payload using the current uav
                if self.__payload == 1:
                    self.__droplocation = [loc[0], loc[1]]
                    self.__payload = 0
                else:
                    newloc = self.__droplocation.copy()
                    print("smarter payload to be send")
                    self.update_Leftout_PD_location(newloc)
                    self.__droplocation = [loc[0], loc[1]]
            GlobalUavData[str(critical_key)]["PAYLOAD"]=0
                    # payload_drop(GlobalUavData,self.__droplocation,fol) ### __droplocation should be sent
        else:
            print("yes i saved")
            self.__futurepayload.append(loc)