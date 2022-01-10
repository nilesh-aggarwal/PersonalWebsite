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
from SwarmBot import SwarmBot
import sys
import socket
import json
import threading
import time
import dronekit as dk
import sys
import logging

# ====================================================================================================================================== #
#                                  COMPLETE EXHAUSTIVE SEARCH CODEBASE - FLIGHT CODE 19/01/20                                            #
# ====================================================================================================================================== #

class mopso_swarm:
    def __init__(self,SYSID,takeoff_var):

        self.self_connection_string = '127.0.0.1:14551'
        self.local_sendip="127.0.0.1"
        self.local_sendport = 10000 
        self.local_bindip = "127.0.0.1"
        self.local_bindport = 10001
        self.trans_send_port=10005
        self.humanrecv_port = 10002
        self.shared_dict={"SYSID":0,"GEOLOCATION":[],"HUMANS":[],"G":0.0,"P":0.0,"BESTLOC":(0,0),"GBESTLOC":(0,0),"PD":(0,0),"PAYLOAD":0,"DROPLOCATION":(0,0)}
        self.sleep_time= .1 # 10 hz refresh rate use if doing multithreading
        self.GlobalUavData = {}
        self.human_lt = []
        self.sysid=SYSID
        logging.basicConfig(filename="Integrated_swarm_with_exhaustive_search.log", format="%(module)s %(lineno)d %(message)s", filemode='w')
        self.logger = logging.getLogger()
        self.logger.setLevel(logging.INFO)
        self.takeoff_var = takeoff_var

        try:
            self.swarm_send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.swarm_send_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

            self.swarm_recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.swarm_recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.swarm_recv_sock.bind((self.local_bindip, self.local_bindport))  # bind address
        except Exception as err:
            print("ERROR IN BINDING PORTS ", err)
        pass

    def send_to_server(self):
        while True:
            try:
                app_json = json.dumps(self.shared_dict, sort_keys=True).encode("UTF-8")
                self.swarm_send_sock.sendto(app_json, (self.local_sendip, self.local_sendport))
                #print("Data sent to swarm_code")
                #print(self.shared_dict)
                time.sleep(self.sleep_time)

            except Exception as err:
                self.swarm_send_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                print("local_socket err in main thread", err)
                time.sleep(self.sleep_time)

    def recv_from_server(self):
        self.swarm_recv_sock.settimeout(0.1)
        while True:
            try:

                bin_str = self.swarm_recv_sock.recv(10240)
                dt = json.loads(bin_str.decode('utf-8'))
                time.sleep(self.sleep_time)
                #self.shared_dict=dt
                self.GlobalUavData=dt
                #print(".........DATA_RECEIVED........",dt)

            except Exception as err:
                print("exception occurred in recv_data " + str(err))
                time.sleep(self.sleep_time)
            time.sleep(self.sleep_time)

    def start_human_detection(self,sock, message):

        try:

            message = message.encode("UTF-8")
            sock.sendto(message, ("127.0.0.1", self.trans_send_port))
            print("Data sent to Transition")


        except Exception as err:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

            print("Error in send to swarmcontroller", err)

    def recv_from_human_detection(self, sock):
        sock.settimeout(1)
        while True:
            try:

                bin_str = sock.recv(128)
                dt = json.loads(bin_str.decode('utf-8'))

                self.shared_dict["HUMANS"] = dt["HUMANS"]

            except Exception as err:
                #print("No Data Received from HD " + str(err))
                pass

    def main(self):
        log_time=time.time()
        print ("STARTING MISSION")
        ## ================================================================ ##
        ##                  SENDING AND RECEIVNG THREADS                    ##
        ## ================================================================ ##
        send_thread = threading.Thread(target=self.send_to_server,args=[])
        send_thread.daemon=True
        send_thread.start()

        ## ================================================================ ##
        ##                  CREATING OBJECT OF SWARMBOT                     ##
        ## ================================================================ ##
        uav = SwarmBot(self.self_connection_string,self.sysid)

        ## ================================================================ ##
        ##                     LOADING MISSION DATA                         ##
        ## ================================================================ ##
        N, N1, distance_bw_UAVs, dij, waypoint_list, wp_rad, takeoff_alti, SAL, SAB, Heading, FOV_X, FOV_Y,search_alti,search_speed,transition_speed = Load_files()

        ## ================================================================ ##
        ##                       Tuning Parameters                          ##
        ## ================================================================ ##
        flock_gain = 10000               ## Waypoint attraction parameter
        formation_gain = 10000           ## Formation control parameter
        v_max_z = 1.0                    ## Velocity limit in z direction

        interheight_variation=5
        d_L = np.zeros((N, 1))
        Termination_modes = ["RTL","LAND"]
        Waypoint_List_of_UAV = list()
        hd_flag=False

        alti_jumper=False
        inc_alti=15
        check_link_loss=False
        link_loss_time=0
        motor_failsafe_vel = 1
        GCS_Failsafe_time = 5
        link_time = 0
        min_failsafe_alti=16
        ## ================================================================ ##
        ##                SOCKET OBJECT FOR HUMAN DETECTION                 ##
        ## ================================================================ ##
        trans_send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        trans_send_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        human_recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        human_recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        human_recv_sock.bind((self.local_bindip, self.humanrecv_port))
        humanrecv_thrd = threading.Thread(target=self.recv_from_human_detection, args=[human_recv_sock])
        humanrecv_thrd.daemon=True
        humanrecv_thrd.start()

        ## ================================================================ ##
        ##              UPDATING THE WAYPOINT LIST OF EACH UAV              ##
        ## ================================================================ ##
        for wp in waypoint_list:
            if wp[2] == 0:
                Waypoint_List_of_UAV.append(wp)
                if uav.id%2==0:
                    wp[4]+=interheight_variation
                else:
                    wp[4]+=0

            elif wp[2] == 1:
                print("Search Area Defined")
                #search_speed=wp[3]
                search_speed=5
                #wp[4] += uav.id * interheight_variation
                Search_wp_start = wp
                p1, p2, p3, p4 = rectangle(SAL, SAB, Search_wp_start[0], Search_wp_start[1], Heading)
                Initial_waypoints = initialwaypoints(N, p1,p2,p3,p4,Heading+90)
                Final_waypoints = finalwaypoins(N, p1,p2,p3,p4,Heading+90) 

                Waypoint_List_of_UAV.append([Initial_waypoints[uav.id - 1][0], Initial_waypoints[uav.id - 1][1], 1, transition_speed, wp[4]])
                Search_wp_end = [Final_waypoints[uav.id - 1][0], Final_waypoints[uav.id - 1][1], 1, search_speed, wp[4]]
                Waypoint_List_of_UAV.append(Search_wp_end)

        self.logger.info("Waypoint_List_of_UAV: " + str(Waypoint_List_of_UAV))

        ## ================================================================ ##
        ##                     ARM AND TAKEOFF THE UAV                      ##
        ## ================================================================ ##
        uav.arm_and_takeoff(takeoff_alti, self.takeoff_var)

        wp_update_var = 0
        posL = Waypoint_List_of_UAV[wp_update_var]

        ## ================================================================ ##
        ##     UPDATE THE BEARING OF UAV IN DIRECTION OF FIRST WAYPOINT     ##
        ## ================================================================ ##
        uav.update_bearing(bearing(uav.get_pos()[0], uav.get_pos()[1], Waypoint_List_of_UAV[wp_update_var][0], Waypoint_List_of_UAV[wp_update_var][1]))

        ## =================================================================================================================== ##
        ##                                  LOOPING TILL ALL UAVS REACH THE TAKEOFF ALTITUDE                                   ##
        ## =================================================================================================================== ##        
        end=False
        break_time_limit = 180
        break_time = 0
        while not end and break_time<break_time_limit:
            t1 = time.time()
            bin_str = self.swarm_recv_sock.recv(10240)
            dt = json.loads(bin_str.decode('utf-8'))
            self.GlobalUavData = dt
            print(end,break_time)
            print(self.GlobalUavData.items())
            for key,datalist in self.GlobalUavData.items():

                lat,lon,heading,alti = datalist["GEOLOCATION"]
                sysid = datalist["SYSID"]
                print(key,alti)
                if float(alti) < (takeoff_alti) *.95 :
                    self.logger.info("ALTI OF UAV" +str(key) +"NOT REACHED "+ str((takeoff_alti) * .95)+ "but instead reached "+ str(alti))
                    break
                else:
                    self.logger.info("ALTI OF UAV"+ str(key)+" REACHED "+str((takeoff_alti) * .95))

            else:
                end=True
                break
            print("BREAK_time",break_time)

            break_time += time.time() - t1
        print("outside_breaktime***********************************************************************")
        ## =================================================================================================================== ##
        ##                                               STARTING MAIN LOOP                                                    ##
        ## =================================================================================================================== ##
        UAV_ID = uav.id
        print(len(Waypoint_List_of_UAV))
        end = False
        
        while (posL != Waypoint_List_of_UAV[-1] and uav.vehicle.mode.name not in Termination_modes):
            bin_str = self.swarm_recv_sock.recv(10240)
            dt = json.loads(bin_str.decode('utf-8'))

            self.GlobalUavData = dt
            if self.GlobalUavData == {}:
                continue

            v_formation = [0, 0]
            pos1 = uav.get_pos()
            d_L[UAV_ID - 1] = get_distance(pos1[0], pos1[1], posL[0], posL[1])/1000
            pos2 = [0,0]
            """
            if len(self.GlobalUavData) == 0 or len(self.GlobalUavData) == 1:
                uav.mode("RTL")
                break
            """
            for key,datalist in self.GlobalUavData.items():
                k = datalist["SYSID"] - 1
                print(k)

                lat,lon,bear,alt = datalist["GEOLOCATION"]
                pos2[0] = lat
                pos2[1] = lon

                self.logger.info("actual velocity"+ str(uav.get_vel()))

                if UAV_ID - 1 == k:
                    if [posL[0], posL[1]] != list(Initial_waypoints[UAV_ID-1]) and [posL[0], posL[1]] != list(Final_waypoints[UAV_ID-1]):
                        if d_L[k] - dij[k][N] < wp_rad:
                            wp_update_var += 1
                            print("# ================================================================ #")
                            print("#                 WAYPOINT CHANGED DUE TO ITSELF                   #")
                            print("# ================================================================ #")
                            self.logger.info("# ================================================================ #")
                            self.logger.info("#                 WAYPOINT CHANGED DUE TO ITSELF                   #")
                            self.logger.info("# ================================================================ #")
                            posL = Waypoint_List_of_UAV[wp_update_var]
                            uav.update_bearing(bearing(Waypoint_List_of_UAV[wp_update_var-1][0], Waypoint_List_of_UAV[wp_update_var-1][1], Waypoint_List_of_UAV[wp_update_var][0], Waypoint_List_of_UAV[wp_update_var][1]))
                            time.sleep(1)
                            print("Next_Waypoint", posL)
                            self.logger.info("Next_Waypoint"+str(posL))
                    continue

                d = get_distance(pos1[0], pos1[1], pos2[0], pos2[1])/1000                
                d_L[k] = get_distance(posL[0], posL[1], pos2[0], pos2[1])/1000

                w = (d - dij[UAV_ID - 1][k]) * formation_gain / d

                e = [w * (pos2[0] - pos1[0]), w * (pos2[1] - pos1[1])]
                v_formation = [v_formation[0] + e[0], v_formation[1] + e[1]]

                if v_formation[0] < 0.1 and v_formation[0] > -0.1:
                    v_formation[0] = v_formation[0] * 5

                if v_formation[1] < 0.1 and v_formation[1] > -0.1:
                    v_formation[1] = v_formation[1] * 5

                if d_L[k] - dij[k][N] < wp_rad and [posL[0], posL[1]] != list(Initial_waypoints[UAV_ID-1]) and [posL[0], posL[1]] != list(Final_waypoints[UAV_ID-1]):
                    wp_update_var += 1
                    print("# ================================================================ #")
                    print("#                 WAYPOINT CHANGED DUE TO FRIEND                   #")
                    print("# ================================================================ #")
                    self.logger.info("# ================================================================ #")
                    self.logger.info("#                 WAYPOINT CHANGED DUE TO FRIEND                   #")
                    self.logger.info("# ================================================================ #")
                    posL = Waypoint_List_of_UAV[wp_update_var]
                    uav.update_bearing(bearing(Waypoint_List_of_UAV[wp_update_var-1][0], Waypoint_List_of_UAV[wp_update_var-1][1], Waypoint_List_of_UAV[wp_update_var][0], Waypoint_List_of_UAV[wp_update_var][1]))
                    time.sleep(1)
                    print("Next_Waypoint", posL)
                    self.logger.info("Next_Waypoint"+str(posL))
                    break

            ## ================================================================ ##
            ##    CONDITION TO CHECK IF ALL UAVS REACHED THE INITIAL WAYPOINT   ##
            ## ================================================================ ##
            if [posL[0], posL[1]] == list(Initial_waypoints[UAV_ID-1]):
                for key,datalist in self.GlobalUavData.items():
                    lat,lon,bear,alt = datalist["GEOLOCATION"]
                    temp = get_distance(Initial_waypoints[int(key)-1][0], Initial_waypoints[int(key)-1][1], lat, lon)
                    if temp>10:
                        print("D_L of UAV" ,str(key) , "NOT REACHED ")
                        break
                    else:
                        print("D_L of UAV" ,str(key) , "REACHED ")
                        pass
                else:
                    wp_update_var += 1
                    print("# ================================================================ #")
                    print("#             ALL UAVS REACHED THEIR INITIAL WAYPOINTS             #")
                    print("# ================================================================ #")
                    self.logger.info("# ================================================================ #")
                    self.logger.info("#             ALL UAVS REACHED THEIR INITIAL WAYPOINTS             #")
                    self.logger.info("# ================================================================ #")
                    posL = Waypoint_List_of_UAV[wp_update_var]
                    uav.update_bearing(bearing(Waypoint_List_of_UAV[wp_update_var-1][0], Waypoint_List_of_UAV[wp_update_var-1][1], Waypoint_List_of_UAV[wp_update_var][0], Waypoint_List_of_UAV[wp_update_var][1]))
                    time.sleep(1)
                    print("Next_Waypoint", posL)
                    self.logger.info("Next_Waypoint" + str(posL))

            ## ================================================================ ##
            ##    CONDITION TO CHECK IF ALL UAVS REACHED THE FINAL WAYPOINT     ##
            ## ================================================================ ##
            if [posL[0], posL[1]] == list(Final_waypoints[UAV_ID-1]):
                for key,datalist in self.GlobalUavData.items():
                    lat,lon,bear,alt = datalist["GEOLOCATION"]
                    temp = get_distance(Final_waypoints[int(key)-1][0], Final_waypoints[int(key)-1][1], lat, lon)
                    if temp>10:
                        print("D_L of UAV" ,str(key) , "NOT REACHED ")
                        break
                    else:
                        print("D_L of UAV" ,str(key) , "REACHED ")
                        pass
                else:
                    try:
                        payload_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                        local_ip = "127.0.0.1"
                        payload_port = 20000
                        dt = {}
                        dt["MESSAGE"] = "DROP"
                        dt["SYS_ID"] = 0
                        dt["PAYLOAD"] = []
                        dt["PACKET_NO"] = 0
                        dt["TIMESTAMP"] = 0

                        # payload_sock.sendto(message.encode("utf-8"), (local_ip, payload_port))
                        app_json = json.dumps(dt).encode("UTF-8")
                        payload_sock.sendto(app_json, (local_ip, payload_port))
                        self.logger.info("# ================================================================ #")
                        self.logger.info("#              PAYLOAD_DROPPED                                     #")
                        self.logger.info("# ================================================================ #")

                    except:
                        self.logger.info("#              ERROR IN PAYLOAD DROP                              #")
                        pass
                    wp_update_var += 1
                    print("# ================================================================ #")
                    print("#              ALL UAVS REACHED THEIR FINAL WAYPOINTS              #")
                    print("# ================================================================ #")
                    self.logger.info("# ================================================================ #")
                    self.logger.info("#              ALL UAVS REACHED THEIR FINAL WAYPOINTS              #")
                    self.logger.info("# ================================================================ #")
                    posL = Waypoint_List_of_UAV[wp_update_var]
                    uav.update_bearing(bearing(Waypoint_List_of_UAV[wp_update_var-1][0], Waypoint_List_of_UAV[wp_update_var-1][1], Waypoint_List_of_UAV[wp_update_var][0], Waypoint_List_of_UAV[wp_update_var][1]))
                    time.sleep(1)
                    print("Next_Waypoint", posL)
                    self.logger.info("Next_Waypoint" + str(posL))

            v_flock = [((posL[0] - pos1[0]) * (d_L[UAV_ID - 1] - dij[UAV_ID - 1][N]) / d_L[UAV_ID - 1]) * flock_gain * (
                       1 - dij[UAV_ID - 1][N] / d_L[UAV_ID - 1]),
                       ((posL[1] - pos1[1]) * (d_L[UAV_ID - 1] - dij[UAV_ID - 1][N]) / d_L[UAV_ID - 1]) * flock_gain * (
                       1 - dij[UAV_ID - 1][N] / d_L[UAV_ID - 1])]

            if v_flock[0] < 0.1 and v_flock[0] > -0.1:
                v_flock[0] = v_flock[0] * 5

            if v_flock[1] < 0.1 and v_flock[1] > -0.1:
                v_flock[1] = v_flock[1] * 5

            v_formation = inside_circle(v_formation, posL[3], 0)
            v_flock = inside_circle(v_flock, posL[3], 0)

            v_formation_flocking = [v_flock[0] + v_formation[0], v_flock[1] + v_formation[1]]

            v_formation_flocking = inside_circle(v_formation_flocking, posL[3], 0)

            if d_L[UAV_ID - 1] > wp_rad:
                v_formation_flocking = [v_formation_flocking[0] * (1 - dij[UAV_ID - 1][N] / d_L[UAV_ID - 1]), v_formation_flocking[1] * (1 - dij[UAV_ID - 1][N] / d_L[UAV_ID - 1])]
            
            ## ================================================================ ##
            ##    CONDITION TO SWITCH BETWEEN FLOCKING AND FORMAION FLOCKING    ##
            ## ================================================================ ##
            if [posL[0], posL[1]] == list(Initial_waypoints[UAV_ID-1]):
                
                if hd_flag:
                    try:
                        self.start_human_detection(trans_send_sock, "STOP")
                        hd_flag = False
                    except Exception as err:
                        print(err)

                v_flock = [(posL[0] - pos1[0]) * flock_gain * abs(1 - wp_rad / d_L[UAV_ID - 1]), (posL[1] - pos1[1]) * flock_gain * abs(1 - wp_rad / d_L[UAV_ID - 1])]
                v_final = inside_circle(v_flock, posL[3], 1)
                velocity_z = -1 * (posL[4] - uav.get_alt()) * 0.9
                if abs(velocity_z) > v_max_z:
                    velocity_z = Limit_vel(velocity_z, v_max_z)
                v_final = [v_final[0], v_final[1], velocity_z]


            elif [posL[0], posL[1]] == list(Final_waypoints[UAV_ID-1]):
                if not hd_flag:
                    try:
                        self.start_human_detection(trans_send_sock, "START")
                        hd_flag = True
                    except Exception as err:
                        print(err)

                v_flock = [(posL[0] - pos1[0]) * flock_gain * abs(1 - wp_rad / d_L[UAV_ID - 1]),(posL[1] - pos1[1]) * flock_gain * abs(1 - wp_rad / d_L[UAV_ID - 1])]
                v_final = inside_circle(v_flock, posL[3], 1)
                velocity_z = -1 * (posL[4] - uav.get_alt()) * 0.9
                if abs(velocity_z) > v_max_z:
                    velocity_z = Limit_vel(velocity_z, v_max_z)
                v_final = [v_final[0], v_final[1], velocity_z]
                
            else:
                if hd_flag:
                    try:
                        self.start_human_detection(trans_send_sock, "STOP")
                        hd_flag = False
                    except Exception as err:
                        print(err)

                v_final = inside_circle(v_formation_flocking, posL[3], 0)
                velocity_z = -1 * (posL[4] - uav.get_alt()) * 0.9       ## Use this to control altitude at each waypoint.
                #velocity_z = 0
                if abs(velocity_z) > v_max_z:
                    velocity_z = Limit_vel(velocity_z, v_max_z)
                v_final = [v_final[0], v_final[1], velocity_z]
            
            ## ================================================================ ##
            ##                         FAILSAFES                                ##
            ## ================================================================ ##

            if uav.get_alt() < min_failsafe_alti:
                uav.land()
                print("uav alti less than"+str(min_failsafe_alti)+" Landing the UAV")
                self.logger.info("uav alti less than 20! Landing the UAV")
            print("ALtitude: ",uav.get_alt(),"Current UAV velocity : ",uav.get_vel())
                
            if uav.get_alt() < posL[4]*0.75 and uav.get_vel()[2] > motor_failsafe_vel: # velocity vector to be added
                print(uav.get_alt(),uav.get_vel()[2],uav.get_vel())
                uav.land()
                print("crash failsafe TRIGGERED")
                self.logger.info("UAV CRASH FAILSAFE TRIGGERED")
                self.logger.info("altitude"+str(uav.get_alt()))     #end
            
            if len(self.GlobalUavData) ==1 and check_link_loss == False: 
                check_link_loss = True
                link_time = time.time()
            
            elif len(self.GlobalUavData)==1 and check_link_loss == True:
                link_loss_time = time.time() - link_time
                #print(link_loss_time,"link loss time")
                
                if link_loss_time > GCS_Failsafe_time and alti_jumper==False:
                    print("breaking LOOP")
                    alti_jumper=True
                    for i in Waypoint_List_of_UAV:
                        i[4]=i[4]+ inc_alti# add to stop altijump
                        self.logger.info("link loss from other UAVS")

            else:
                link_loss_time = 0
                check_link_loss = False             



            ## ================================================================ ##
            ##                 UPDATE FINAL VELOCITY TO THE UAV                 ##
            ## ================================================================ ##
            if time.time()-log_time >1:
                self.logger.info("Final_Velocity: " + str(v_final))
                log_time=time.time()
            print("Final v:", v_final)
            #uav.update_bearing(bearing(Waypoint_List_of_UAV[wp_update_var-1][0], Waypoint_List_of_UAV[wp_update_var-1][1], Waypoint_List_of_UAV[wp_update_var][0], Waypoint_List_of_UAV[wp_update_var][1]))
            uav.update_vel(v_final)

        ## ================================================================ ##
        ##                       MAIN LOOP OVER HERE                        ##
        ## ================================================================ ##
        if hd_flag:
            try:
                self.start_human_detection(trans_send_sock, "STOP")
                hd_flag = False
            except Exception as err:
                print(err)

        self.logger.info("MISSION COMPLETED!")
        print("MISSION COMPLETED!")
        time.sleep(2)   # To make all the UAVs land at the same time
        if uav.get_mode() not in Termination_modes:
            uav.land()
        uav.vehicle.close()

if __name__ == "__main__":
    sys_id=int(sys.argv[1])
    takeoff_var = int(sys.argv[2])
    obj=mopso_swarm(sys_id, takeoff_var)
    obj.main()
    sys.exit(1)
