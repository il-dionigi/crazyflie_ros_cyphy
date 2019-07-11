#!/usr/bin/env python
import rospy
import tf
import numpy as np


import atexit
from crazyflie_driver.msg import Position
from crazyflie_driver.msg import ConsoleMessage
from crazyflie_driver.msg import Hover
from crazyflie_driver.msg import GenericLogData
from std_msgs.msg import Empty
from crazyflie_driver.srv import UpdateParams
from threading import Thread
from geometry_msgs.msg import PointStamped, TransformStamped, PoseStamped #PoseStamped added to support vrpn_client


LOCODECK_TS_FREQ = 499.2*(10**6) * 128
delta_bs = [0,0,0,0, 0,0,0,0]
beaconPos = [0,0,0]
beaconPos2 = [0,0,0]
cameraPos = [0,0,0]
beaconDists = [0,0,0,0, 0,0,0,0]
beaconMins =  [0,0,0,0, 0,0,0,0]
beaconMaxs =  [0,0,0,0, 0,0,0,0]
beaconMeans = [0,0,0,0, 0,0,0,0]
ts = [0,0,0, 0,0,0,0]
count1 = 0
count2 = 0
samples = 20
#x, y, z, yaw
currPos = [0,0,0,0] # current setpoint
STOP = False
delta_d_list = []
delta_p_list = []
delta_b_list = []
bc_diffx = []
bc_diffy = []
bc_diffz = []

def exit_handler():
    rospy.loginfo("///Exiting. sending stop command///")
    positionMove([0,0,0,0],0.1)
    stop_pub = rospy.Publisher("cmd_stop", Empty, queue_size=1)
    stop_msg = Empty()
    STOP = True
    stop_pub.publish(stop_msg)


#They have the same coords now, so this isn't needed
def beaconToCameraCoords(pos_b):
    shift = [0.0, 0.0, 0.0] # position according to beacons when camera pos is 0,0,0
    pos_c = [0,0,0]
    # shift
    pos_c[0] = pos_b[0] - shift[0]
    pos_c[1] = pos_b[1] - shift[1]
    pos_c[2] = pos_b[2] - shift[2]
    # rotate
    pos_c[0] = pos_c[0]
    pos_c[1] = pos_c[1]
    return pos_c
    
def callback_beacon_ranging1(data):
    global beaconDists, count1, beaconMins, beaconMaxs, beaconMeans
    
    for i in range(4):
        beaconDists[i] = data.values[i]
    """count1 = count1 + 1
    if (count1 == samples):
        count1 = 0
        for i in range(4):
            rospy.loginfo("{} Mean: {} Max: {} Min: {}".format(i, beaconMeans[i]/samples, beaconMaxs[i], beaconMins[i]))
            beaconMaxs[i] = data.values[i]
            beaconMins[i] = data.values[i]
            beaconMeans[i] = data.values[i]  
    else:
        for i in range(4):
            if beaconDists[i] > beaconMaxs[i]:
                beaconMaxs[i] = beaconDists[i]
            if beaconDists[i] < beaconMins[i]:
                beaconMins[i] = beaconDists[i]
            beaconMeans[i] += beaconDists[i]"""

def callback_beacon_ranging2(data):
    global beaconDists, count2
    for i in range(4,8):
        beaconDists[i] = data.values[i-4]

def callback_pos_beacons(data):
    global beaconPos
    beaconPos[0] = data.values[0]
    beaconPos[1] = data.values[1]
    beaconPos[2] = data.values[2]

def callback_pos_beacons2(data):
    global beaconPos2
    beaconPos2[0] = data.values[0]
    beaconPos2[1] = data.values[1]
    beaconPos2[2] = data.values[2]


def callback_pos_camera(data):
    global cameraPos
    cameraPos[0] = data.point.x
    cameraPos[1] = data.point.y
    cameraPos[2] = data.point.z

def callback_twr_time(data):
    global ts, delta_d_list, delta_p_list, delta_b_list
    for i in range(len(data.values)):
        ts[i] = data.values[i]

    delta_p_list.append(1000*((ts[5])/LOCODECK_TS_FREQ + ts[6]*(2**32/LOCODECK_TS_FREQ)) )
    delta_d_list.append(1000*(ts[4]-ts[3])/LOCODECK_TS_FREQ)


def callback_twr_beacon(data):
    global delta_bs
    for i in range(len(data.values)):
        delta_bs[i] = data.values[i]
    delta_b_list.append(1000*delta_bs[0]/LOCODECK_TS_FREQ)

def publisherThread():
    global currPos
    sequence = 0
    while not rospy.is_shutdown():
        if STOP:
            return
    
        msgPos.x = currPos[0]
        msgPos.y = currPos[1]
        msgPos.z = currPos[2]
        msgPos.yaw = currPos[3]
        msgPos.header.seq = sequence
        msgPos.header.stamp = rospy.Time.now()
        pubPos.publish(msgPos)

        #rospy.loginfo("sending...(M)")
        #rospy.loginfo("x:"+ str(msgPos.x) + " y:" + str(msgPos.y) + " z:" + str(msgPos.z))
        sequence += 1
        rate.sleep() 

def positionMove(pos=[0,0,0,0], t=1, N=20):
    global currPos
    currPos = pos
    for i in range(N):
        rospy.sleep(1.0*t/N)
        if (N > 1):
            get_diff()
        #print_beacon_camera_diff()

def get_diff():
    global cameraPos, beaconPos2, bc_diffx, bc_diffy, bc_diffz
    dx = cameraPos[0] - beaconPos2[0]
    dy = cameraPos[1] - beaconPos2[1]
    dz = cameraPos[2] - beaconPos2[2]
    bc_diffx.append(np.abs(dx))
    bc_diffy.append(np.abs(dy))
    bc_diffz.append(np.abs(dz))

def get_stats():
    global delta_b_list, delta_d_list, delta_p_list, bc_diffx, bc_diffy, bc_diffy
    rospy.loginfo("STATS: delta_b, delta_d, delta_p in ms, dx, dy, dz in m")
    #rospy.loginfo("db mean:{}, stddev:{}, max-min:{}".format(np.mean(delta_b_list), np.std(delta_b_list), np.max(delta_b_list)-np.min(delta_b_list)))
    #rospy.loginfo("dd mean:{}, stddev:{}, max-min:{}".format(np.mean(delta_d_list), np.std(delta_d_list), np.max(delta_d_list)-np.min(delta_d_list)))
    rospy.loginfo("dp mean:{}, stddev:{}, max-min:{}".format(np.mean(delta_p_list), np.std(delta_p_list), np.max(delta_p_list)-np.min(delta_p_list)))
    #rospy.loginfo(str(delta_b_list))
    rospy.loginfo("dx mean:{}, stddev:{}, max-min:{}".format(np.mean(bc_diffx), np.std(bc_diffx), np.max(bc_diffx)-np.min(bc_diffx)))
    rospy.loginfo("dy mean:{}, stddev:{}, max-min:{}".format(np.mean(bc_diffy), np.std(bc_diffy), np.max(bc_diffy)-np.min(bc_diffy)))
    rospy.loginfo("dz mean:{}, stddev:{}, max-min:{}".format(np.mean(bc_diffz), np.std(bc_diffz), np.max(bc_diffz)-np.min(bc_diffz)))
    f = open("delta_p_"+str(np.mean(delta_p_list))+".txt", "a")
    f.write("\nNEW RUN: \ndp=" + str(np.mean(delta_p_list) ) + "\n")
    f.write("\nmeanx:{}\n".format(np.mean(bc_diffx) ))
    f.write("\nstddevx:{}\n".format(np.std(bc_diffx) ))
    f.write("\nmeany:{}\n".format(np.mean(bc_diffy) ))
    f.write("\nstddevy:{}\n".format(np.std(bc_diffy) ))
    f.write("\nmeanz:{}\n".format(np.mean(bc_diffz) ))
    f.write("\nstddevz:{}\n".format(np.std(bc_diffz) ))
    for num in bc_diffx:
        f.write("\ndx:"+str(num))
    for num in bc_diffy:
        f.write("\ndy:"+str(num))
    for num in bc_diffz:
        f.write("\ndz:"+str(num))

def print_beacon_camera_diff():
    global cameraPos, beaconPos, beaconPos2, beaconDists
    dx = cameraPos[0] - beaconPos2[0]
    dy = cameraPos[1] - beaconPos2[1]
    dz = cameraPos[2] - beaconPos2[2]
    rospy.loginfo("camera... x:"+ str(cameraPos[0]) + " y:" + str(cameraPos[1]) + " z:" + str(cameraPos[2]))
    #rospy.loginfo("beaconSE... x:"+ str(beaconPos[0]) + " y:" + str(beaconPos[1]) + " z:" + str(beaconPos[2]))
    rospy.loginfo("beaconKF... x:"+ str(beaconPos2[0]) + " y:" + str(beaconPos2[1]) + " z:" + str(beaconPos2[2]))
    rospy.loginfo("diff... dx:"+ str(dx) + " dy:" + str(dy) + " dz:" + str(dz))
    """rospy.loginfo("beaconDists...")
    rospy.loginfo("0: " + str(beaconDists[0]) + " 1: " + str(beaconDists[1]))
    rospy.loginfo("2: " + str(beaconDists[2]) + " 3: " + str(beaconDists[3]))
    rospy.loginfo("4: " + str(beaconDists[4]) + " 5: " + str(beaconDists[5]))
    rospy.loginfo("6: " + str(beaconDists[6]) + " 7: " + str(beaconDists[7]))"""

def print_ts():
    global ts
    rospy.loginfo("times: ")
    #for i in range(0,5):
    #    rospy.loginfo("t{}: {}".format(i+1, ts[i]))
    rospy.loginfo("calculated deltas (s):\n delta_beacon:{}, delta_drone:{}, delta_p:{}".format((ts[2]-ts[1])/LOCODECK_TS_FREQ, (ts[4]-ts[3])/LOCODECK_TS_FREQ, ts[5]/LOCODECK_TS_FREQ))

def print_deltas():
    global delta_bs, ts
    rospy.loginfo("delta_bs: ")
    for i in range(6):
        rospy.loginfo("delta_b{}: {}".format(i, delta_bs[i]/LOCODECK_TS_FREQ))
    rospy.loginfo("delta_drone:{}".format((ts[4]-ts[3])/LOCODECK_TS_FREQ))
    #rospy.loginfo("delta_pl32:{}".format((ts[5])/LOCODECK_TS_FREQ))
    #rospy.loginfo("delta_ph8:{}".format((ts[6])/LOCODECK_TS_FREQ))
    rospy.loginfo("delta_p:{}".format((ts[5])/LOCODECK_TS_FREQ + ts[6]*(2**32/LOCODECK_TS_FREQ) ))

def setpoints_to_time(s1, s2):
    return 1.2*((s1[0] - s2[0])**2 + (s1[1]-s2[1])**2)**(0.5)

if __name__ == '__main__':
    atexit.register(exit_handler)
    rospy.init_node('beaconTest', anonymous=True)
    worldFrame = rospy.get_param("~worldFrame", "/world")
    rate = rospy.Rate(10) #  hz
    rospy.Subscriber("external_position", PointStamped, callback_pos_camera)
    #rospy.Subscriber("SE", GenericLogData, callback_pos_beacons)
    rospy.Subscriber("KF", GenericLogData, callback_pos_beacons2)
    #rospy.Subscriber("Ranging1", GenericLogData, callback_beacon_ranging1)
    #rospy.Subscriber("Ranging2", GenericLogData, callback_beacon_ranging2)
    rospy.Subscriber("TWRtime", GenericLogData, callback_twr_time)
    rospy.Subscriber("TWRbeacons", GenericLogData, callback_twr_beacon)
    #for position mode
    msgPos = Position()
    msgPos.header.seq = 0
    msgPos.header.stamp = rospy.Time.now()
    msgPos.header.frame_id = worldFrame
    msgPos.x = 0.0
    msgPos.y = 0.0
    msgPos.z = 0.0
    msgPos.yaw = 0.0
    pubPos = rospy.Publisher("cmd_setpoint", Position, queue_size=1)

    #publish to console
    msgConsole = ConsoleMessage()
    msgConsole.header.seq = 0
    msgConsole.header.stamp = rospy.Time.now()
    msgConsole.header.frame_id = worldFrame
    msgConsole.data = [ord('y'), ord('r'), ord('L'), 0]
    pubConsole = rospy.Publisher("cmd_console_msg", ConsoleMessage, queue_size=1)

    rospy.wait_for_service('update_params')
    rospy.loginfo("found update_params service")
    update_params = rospy.ServiceProxy('update_params', UpdateParams)

    rospy.set_param("kalman/resetEstimation", 1)
    update_params(["kalman/resetEstimation"])
    rospy.sleep(0.1)
    rospy.set_param("kalman/resetEstimation", 0)
    update_params(["kalman/resetEstimation"])
    rospy.sleep(0.1)
    rospy.set_param("flightmode/posSet", 1)
    update_params(["flightmode/posSet"])
    rospy.sleep(3)
    pubConsole.publish(msgConsole)
    msgPublisher = Thread(target=publisherThread)
    msgPublisher.start()

    timeAlloted = 1.5
    print_beacon_camera_diff()

    rospy.sleep(2)
    
    for i in range(1000):
        rospy.sleep(10)
        print_deltas()
        print_beacon_camera_diff()
        #rospy.sleep(2)
        #print_ts() 

  

    
    positionMove([beaconPos2[0],beaconPos2[1],0.4,0],.1, N=1) #takeoff
    rospy.loginfo("SHOULD BE IN AIR?!...")
    setpoint = [0,0,0.5,0]
    last_sp = setpoint
    #positionMove(setpoint, timeAlloted+3, N=1) #zero x,y
    #square_setpoints = [ [0,0.5,0.5,0], [0.5,0.5,0.5,0],  [0.5,0,0.5,0], [0,0,0.5,0] ]
    large_square = [ [1,0.5,0.5,0], [-1,0.5,0.5,0],  [-1,-0.5,0.5,0], [1,-0.5,0.5,0], [0,0,0.5,0] ]
    #triangle_setpoints = [ [0,0.5,0.5,0], [-0.3,0,0.5,0],  [0.3,0,0.5,0], [0,0.5,0.5,0] ]
    #traj_setpoints = [ [0, 0, 0.5, 0], [-1, 0, 0.5, 0], [0, -0.8, 0.5, 0], [-0.5, -0.5, 0.5, 0], [1, -0.2, 0.5, 0], [0.5, 0.3, 0.5, 0], [1, -0.5, 0.5, 0], [0,0,0.5,0] ]
    sp_list = large_square
    for i in range(len(sp_list)):
        setpoint = traj_setpoints[i]
        timeAlloted = setpoints_to_time(setpoint, last_sp)
        positionMove(setpoint, timeAlloted, N=50)
        last_sp = setpoint
        rospy.loginfo("SETPOINT..." + str(setpoint))
        print_beacon_camera_diff()

    #land
    positionMove([0,0,0.3,0],0.5, N=1)
    positionMove([0,0,0.2,0],0.4,N=1)
    positionMove([0,0,0,0],0.3, N=1)
    #print_beacon_camera_diff()
    rospy.loginfo("Done.")

    get_stats()
    

    exit_handler()

    

