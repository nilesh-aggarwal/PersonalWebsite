import socket
from math import *
import time
import numpy as np
from math import *
import threading
from math import radians, cos, sin, asin, sqrt,atan2
import random
from time import time
import json
import pickle
from geopy import distance

def get_distance(lat1,lon1,lat2,lon2): 
    # lon1=radians(lon1) 
    # lon2=radians(lon2)
    # lat1=radians(lat1) 
    # lat2=radians(lat2) 
    # dlon=lon2-lon1  
    # dlat=lat2-lat1 
    # a=sin(dlat/2)**2+cos(lat1)*cos(lat2)*sin(dlon/2)**2
    # c=2*asin(sqrt(a))
    # R=6371 
    # return(c*R*1000)
    return distance.distance((lat1,lon1),(lat2,lon2)).km*1000


def pointRadialDistance(lat1,lon1,angle,d):
    """
    Return final coordinates (lat2,lon2) [in degrees] given initial coordinates
    (lat1,lon1) [in degrees] and a bearing [in degrees] and distance [in km]
    """
    lat1 = radians(lat1)
    lon1 = radians(lon1)
    R=6371
    d=d/1000
    ad=d/R
    lat2 = asin(sin(lat1)*cos(ad) +cos(lat1)*sin(ad)*cos(angle))
    lon2 = lon1 + atan2(sin(angle)*sin(ad)*cos(lat1),cos(ad)-sin(lat1)*sin(lat2))
    lat = degrees(lat2)
    lon = degrees(lon2)
    b=[lat,lon]
    return b

def rectangle(m,n,lat,lon,head):
    heading=float(head)
    heading=radians(heading)
    e=pointRadialDistance(lat,lon,heading,n)
    rheading=heading+pi/2
    lheading=heading-pi/2
    p1=pointRadialDistance(lat,lon,lheading,m/2)
    p2=pointRadialDistance(lat,lon,rheading,m/2)
    p3=pointRadialDistance(e[0],e[1],rheading,m/2)
    p4=pointRadialDistance(e[0],e[1],lheading,m/2)
    return p1,p2,p3,p4

def rectangle2(pos1,heading,fol,fow):
    u=pointRadialDistance(pos1[0],pos1[1],heading-pi,fow/2)
    n=[]
    n=rectangle(fol,fow,u[0],u[1],heading)
    return n

def bearing(lat1,lon1,lat2,lon2):
    lon1=radians(lon1)
    lon2=radians(lon2)
    lat1=radians(lat1)
    lat2=radians(lat2)
    dlon=abs(lon2-lon1)
    bearing=atan2(sin(dlon)*cos(lat2),cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(dlon))
    return (degrees(bearing))

def initialwaypoints(n,p1,p2,p3,p4,x):
    x=float(x)
    x=radians(x)
    dis=float(get_distance(p1[0],p1[1],p2[0],p2[1])/n)
    formation_waypoints=[]
    for i in range(n):
        if i==0:
            u=pointRadialDistance(p1[0],p1[1],x,(dis/2))
            formation_waypoints.append(u)
        else:
            lat_lon=pointRadialDistance(formation_waypoints[i-1][0],formation_waypoints[i-1][1],x,(dis))
            formation_waypoints.append(lat_lon)
    return formation_waypoints

def finalwaypoins(n,p1,p2,p3,p4,x):
    x=float(x)
    x=radians(x)
    b=[]
    dis=float(get_distance(p1[0],p1[1],p2[0],p2[1])/n)
    for i in range(n):
        if i==0:
            u=pointRadialDistance(p4[0],p4[1],x,(dis/2))
            b.append(u)
        else:
            nmmm=pointRadialDistance(b[i-1][0],b[i-1][1],x,(dis))
            b.append(nmmm)
    return b

def inside_circle(vel, v_max, v_min):
        #print(vel)
        x=vel[0]
        y=vel[1]
        vc=[]
        if (x*x)+(y*y)-(v_max**2)<=0 and (x*x)+(y*y)-(v_min**2)>=0 :
            vc.append(x)
            vc.append(y)
            vc.append(0)
        elif (x*x)+(y*y)-(v_max**2)>=0:
            if x!=0:
                vc.append(v_max*cos(atan2(y,x)))
                vc.append(v_max*sin(atan2(y,x)))
                vc.append(0)
            elif y!=0:
                vc.append(v_max*sin(atan2(x,y)))
                vc.append(v_max*cos(atan2(x,y)))
                vc.append(0)
            else:
                vc.append(x)
                vc.append(y)
                vc.append(0)

        elif (x*x)+(y*y)-(v_min**2)<=0:
            if x!=0:

                vc.append(v_min*cos(atan2(y,x)))
                vc.append(v_min*sin(atan2(y,x)))
                vc.append(0)
            elif y!=0:
                vc.append(v_min*sin(atan2(x,y)))
                vc.append(v_min*cos(atan2(x,y)))
                vc.append(0)
            else:
                vc.append(x)
                vc.append(y)
                vc.append(0)
        return vc

def create_uav_ports():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return sock

def create_and_bind_uav_ports(address):

    sock=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    try:
        sock.bind(address)
        sock.settimeout(0.1)
    except:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(address)
        sock.settimeout(0.1)
    return sock

def send_data(sock, address, port, data):
    data_json = json.dumps(data).encode("UTF-8")
    sock.sendto(data_json, (address, port))

def recv_data(sock):
    try:
        data = sock.recv(1200)
        data = json.loads(data.decode('utf-8'))
        return data
    except:
        print("Exception occured in recv_data in helper functions file ")
        empty_data = {"SYSID":0,"GEOLOCATION":[],"HUMANS":[],"G":0.0,"P":0.0,"BESTLOC":(0,0),"GBESTLOC":(0,0),"PD":(0,0),"PAYLOAD":0,"DROPLOCATION":(0,0)}
        return empty_data

def weight_matrix(d_matrix, N, N1, distance_bw_UAVs):
    dij = np.zeros((N + 1, N + 1))
    for i in range(N):
        for j in range(N):
            dij[i][j] = d_matrix[i][j]
            print(dij[i][j], "\n")

    for i in range(N):
        dij[i][N] = d_matrix[N1][i]

    dij = dij * distance_bw_UAVs

    return dij

def Load_files():
    parameter_file = open('number_of_UAVs', 'rb')
    parameters = pickle.load(parameter_file)
    try:

        N = int(parameters[0])
        N1 = int(parameters[1])- 1
        distance_bw_UAVs = int(parameters[2]) / 1000
        wp_rad = int(parameters[3]) / 1000
        takeoff_alti=int(parameters[4])
        search_alti=int(parameters[5])
        transition_speed = int(parameters[6])
        search_speed = int(parameters[7])
        SAL = int(parameters[8])
        SAB = int(parameters[9])
        Heading = int(parameters[10])
        FOV_X = int(parameters[11])
        FOV_Y = int(parameters[12])

        d_matrix_file = open('weight_matrix', 'rb')
        matrix = pickle.load(d_matrix_file)
        dij = weight_matrix(matrix, N, N1, distance_bw_UAVs)

        wp_file = open("wp_list", 'rb')
        waypoint_list = pickle.load(wp_file)                   ##########################CHANGES TO BE DONE HERE
        waypoint_list.append([1,1,0,0,0])
        
    except Exception as err:
        print(err)
        takeoff_alti=35
        search_alti=25
        search_speed=3
        transition_speed=5
        wp_rad = 0.025
        takeoff_alti = 10
        SAL = 1000
        SAB = 1000
        Heading = 0
        FOV_X = 40
        FOV_Y = 70

    return N, N1, distance_bw_UAVs, dij, waypoint_list, wp_rad, takeoff_alti, SAL, SAB, Heading, FOV_X, FOV_Y,search_alti,search_speed,transition_speed

def Limit_vel(v, v_max):
    return (v / abs(v)) * min(abs(v), abs(v_max))

def T_max_calc_func(SAL, v):
    return SAL/v + 100

def fn(onelist,disitance_between_two_humans):
    #twice the error in human detection

    final_list = [] 
    a=[]
    b=[]
    for num in onelist: 
        for i in onelist:
            if i!=num:
                if get_distance(num[0],num[1],i[0],i[1]) <disitance_between_two_humans:
                    a.append([num,i]) 
    for i in a:
        j=[i[1],i[0]]
        if j in a:
            a.remove(j)
    for i in a:
        b.append(i[0])
        b.append(i[1])
    for i in a:
        if i[0][2]>=i[1][2]:
            final_list.append(i[0])
        else:
            final_list.append(i[1])
    for i in onelist:
        if i not in b:
            final_list.append(i)
    return final_list 

def filterhumans(loooopy,disitance_between_two_humans):
    while True:
        b=fn(loooopy,disitance_between_two_humans)
        if len(b)==len(loooopy):
            break
        loooopy=b
    temp_list=[]
    for i in b:
        if i not in temp_list:
            temp_list.append(i)

    return temp_list


