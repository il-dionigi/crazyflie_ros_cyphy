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

Order = 'F'
delta_p_param = 0
LOCODECK_TS_FREQ = 499.2*(10**6) * 128
delta_bs = [0,0,0,0, 0,0,0,0]
beaconPos = [0,0,0]
beaconPos2 = [0,0,0]
cameraPos = [0,0,0]
beaconDists = [0,0,0,0, 0,0,0,0]
beaconMins =  [0,0,0,0, 0,0,0,0]
beaconMaxs =  [0,0,0,0, 0,0,0,0]
beaconMeans = [0,0,0,0, 0,0,0,0]
delta_pl32 = 0
delta_ph8 = 0
singleRanging = 0
allRangings = 0
betweenRounds = 0
betweenRangings = 0
singleRanging_h8 = 0
allRangings_h8 = 0
betweenRounds_h8 = 0
betweenRangings_h8 = 0
ts = [0,0,0, 0,0,0,0]
count1 = 0
count2 = 0
samples = 100
#x, y, z, yaw
currPos = [0,0,0,0] # current setpoint
shift = [0,0]
STOP = False
delta_d_list = []
delta_p_list = []
delta_b_list = []
bc_diffx = []
bc_diffy = []
bc_diffz = []
error = False
tofs = [1,2,3, 4]
encStates = []
#x,y,z,K
encState = [0,0,0,0]
Ktotal = 0.000001
K1count = 0
obstacles = [ [1,1,0.6], [0,2,0.2], [2,-0.5,0.3], [-1, -6, 3] ]

def err_handler():
    global error, STOP
    if (not error):
        rospy.loginfo("///|Exiting. sending stop command|///")    
    stop_pub = rospy.Publisher("cmd_stop", Empty, queue_size=1)
    stop_msg = Empty()
    STOP = True
    stop_pub.publish(stop_msg)
    error = True
    
    
def exit_handler():
    global STOP
    rospy.loginfo("///Exiting. sending stop command///")
    positionMove([0,0,0,0],0.1)
    stop_pub = rospy.Publisher("cmd_stop", Empty, queue_size=1)
    stop_msg = Empty()
    STOP = True
    stop_pub.publish(stop_msg)

def check_within_bounds():
    global cameraPos, shift
    x, y, z = cameraPos
    if ( -2.4 < x < 2.4 and -1.7 < y < 1.7 and z < 1.3):
        return True
    else:
        if (y < -1.7):
            shift[1] = 0.5
        if (y > 1.7):
            shift[1] = -0.5
        if (x < -2):
            shift[0] = 0.5
        if (x > 2):
            shift[0] = -0.5
        #err_handler()
        return False
    
def callback_beacon_ranging1(data):
    global beaconDists, count1, beaconMins, beaconMaxs, beaconMeans, samples
    
    for i in range(4):
        beaconDists[i] = data.values[i]
    """
    count1 = count1 + 1
    if (count1 == samples):
        count1 = 0
        for i in range(4):
            rospy.loginfo("B{} Mean: {} Max: {} Min: {}".format(i, beaconMeans[i]/samples, beaconMaxs[i], beaconMins[i]))
            beaconMaxs[i] = data.values[i]
            beaconMins[i] = data.values[i]
            beaconMeans[i] = data.values[i]  
    else:
        for i in range(4):
            if beaconDists[i] > beaconMaxs[i]:
                beaconMaxs[i] = beaconDists[i]
            if beaconDists[i] < beaconMins[i]:
                beaconMins[i] = beaconDists[i]
            beaconMeans[i] += beaconDists[i]
    """

def callback_beacon_ranging2(data):
    global beaconDists, count2
    for i in range(4,8):
        beaconDists[i] = data.values[i-4]
    """
    count2 = count2 + 1
    if (count2 == samples):
        count2 = 0
        for i in range(4,8):
            rospy.loginfo("B{} Mean: {} Max: {} Min: {}".format(i, beaconMeans[i]/samples, beaconMaxs[i], beaconMins[i]))
            beaconMaxs[i] = data.values[i-4]
            beaconMins[i] = data.values[i-4]
            beaconMeans[i] = data.values[i-4]  
    else:
        for i in range(4,8):
            if beaconDists[i] > beaconMaxs[i]:
                beaconMaxs[i] = beaconDists[i]
            if beaconDists[i] < beaconMins[i]:
                beaconMins[i] = beaconDists[i]
            beaconMeans[i] += beaconDists[i]
    """

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
    check_within_bounds()

def callback_twr_time(data):
    global ts, delta_d_list, delta_p_list, delta_b_list, delta_pl32, delta_ph8
    for i in range(len(data.values)):
        ts[i] = data.values[i]

    #delta_p_list.append(1000*((ts[5])/LOCODECK_TS_FREQ + ts[6]*(2**32/LOCODECK_TS_FREQ)) )
    delta_d_list.append(1000*(ts[4]-ts[3])/LOCODECK_TS_FREQ)


def callback_twr_beacon(data):
    global delta_bs
    for i in range(len(data.values)):
        delta_bs[i] = data.values[i]
    delta_b_list.append(1000*delta_bs[0]/LOCODECK_TS_FREQ)

def callback_twr_other(data):
    global singleRanging, allRangings, betweenRounds, betweenRangings, singleRanging_h8, allRangings_h8, betweenRounds_h8, betweenRangings_h8
    singleRanging = data.values[0]
    allRangings = data.values[1]
    betweenRounds = data.values[2]
    betweenRangings = data.values[3]

    singleRanging_h8 = data.values[4]
    allRangings_h8 = data.values[5]
    betweenRounds_h8 = data.values[6]
    #betweenRangings_h8 = data.values[7]

def callback_twr_eve(data):
    global tofs
    for i in range(4):
        tofs[i] = data.values[i]
    #est, est2 actual_add, actual_mult

def callback_twr_enc(data):
    global encStates, encState, Ktotal, K1count
    if (encState[0] == data.values[0]):
        return
    for i in range(4):
        encState[i] = data.values[i]
    encStates.append(np.copy(encState))
    Ktotal += 1
    K1count += encState[3]


def publisherThread():
    global currPos, shift
    sequence = 0
    while not rospy.is_shutdown():
        if STOP:
            msgPos.x = 0
	    msgPos.y = 0
	    msgPos.z = 0 
            msgPos.header.seq = sequence
            msgPos.header.stamp = rospy.Time.now()
            for j in range(10):
                pubPos.publish(msgPos)
                sequence += 1
                rate.sleep()
            return
        else:
	    msgPos.x = currPos[0]+shift[0]
	    msgPos.y = currPos[1]+shift[1]
	    msgPos.z = currPos[2]
        msgPos.yaw = currPos[3]
        msgPos.header.seq = sequence
        msgPos.header.stamp = rospy.Time.now()
        pubPos.publish(msgPos)

        #rospy.loginfo("sending...(M)")
        #rospy.loginfo("x:"+ str(msgPos.x) + " y:" + str(msgPos.y) + " z:" + str(msgPos.z))
        sequence += 1
        rate.sleep() 

def positionMove(pos=[0,0,0,0], t=1, N=1):
    global currPos, STOP
    currPos = pos      
    if (STOP):
        return
    for i in range(N):
        rospy.sleep(1.0*t/N)
        if (N > 2):
            get_diff()
        #print_beacon_camera_diff()

def get_diff():
    global cameraPos, beaconPos2, bc_diffx, bc_diffy, bc_diffz, delta_p_list, ts
    dx = cameraPos[0] - beaconPos2[0]
    dy = cameraPos[1] - beaconPos2[1]
    dz = cameraPos[2] - beaconPos2[2]
    if (np.abs(dx) > 2.5 or np.abs(dy) > 2.5):
        err_handler()
    bc_diffx.append(np.abs(dx))
    bc_diffy.append(np.abs(dy))
    bc_diffz.append(np.abs(dz))
    delta_p_list.append(1000*((ts[5])/LOCODECK_TS_FREQ + ts[6]*(2**32/LOCODECK_TS_FREQ)) )

def get_stats():
    global error, delta_b_list, delta_d_list, delta_p_list, bc_diffx, bc_diffy, bc_diffy
    if (error or len(delta_p_list) < 2 or np.std(delta_p_list) == 0):
        rospy.loginfo("XXXX ERROR XXXX DID NOT SAVE XXXX")
        return
    else:
        rospy.loginfo("XXXX SAVED XXXX")
    rospy.loginfo("STATS: delta_b, delta_d, delta_p in ms, dx, dy, dz in m")
    #rospy.loginfo("db mean:{}, stddev:{}, max-min:{}".format(np.mean(delta_b_list), np.std(delta_b_list), np.max(delta_b_list)-np.min(delta_b_list)))
    #rospy.loginfo("dd mean:{}, stddev:{}, max-min:{}".format(np.mean(delta_d_list), np.std(delta_d_list), np.max(delta_d_list)-np.min(delta_d_list)))
    rospy.loginfo("dp mean:{}, stddev:{}, max-min:{}".format(np.mean(delta_p_list), np.std(delta_p_list), np.max(delta_p_list)-np.min(delta_p_list)))
    #rospy.loginfo(str(delta_b_list))
    rospy.loginfo("dx mean:{}, stddev:{}, max-min:{}".format(np.mean(bc_diffx), np.std(bc_diffx), np.max(bc_diffx)-np.min(bc_diffx)))
    rospy.loginfo("dy mean:{}, stddev:{}, max-min:{}".format(np.mean(bc_diffy), np.std(bc_diffy), np.max(bc_diffy)-np.min(bc_diffy)))
    rospy.loginfo("dz mean:{}, stddev:{}, max-min:{}".format(np.mean(bc_diffz), np.std(bc_diffz), np.max(bc_diffz)-np.min(bc_diffz)))
    
    f = open("order:"+Order+str(np.mean(delta_p_list))+".txt", "a")
    f.write("\nNEW RUN: \ndp=" + str(np.mean(delta_p_list) ) + "\n")
    f.write("\nstddevdp:{}\n".format(np.std(delta_p_list) ))
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

def save_enc_trace():
    global encStates, K1count, Ktotal
    if (len(encStates) < 2 or K1count < 2):
        rospy.loginfo("XXXX ERROR XXXX DID NOT SAVE XXXX")
        return
    else:
        rospy.loginfo("XXXX SAVED XXXX")
    mse = 0
    f = open("EncK1percent:{}.txt".format(1.0*K1count/Ktotal), "a")
    f.write("NEW RUN\n")
    for estate in encStates:
        actual = [ estate[0], estate[1], estate[2] ]
        if (estate[3]):
            actual = [-actual[0], -actual[1], 3-actual[2] ]
        for i in range(3):
            f.write(str(estate[i]) + ",")
        f.write(str(estate[3]) + "\n")

    p1, p2 = eve_split_trajs()
    path = p1
    if (check_collision(p1) ):
        path = p2
    elif (check_collision(p2) ):
        path = p1
    else:
        print("NO CRASHES, GUESS")
    
    mse = 0 
    for i in range(len(path)):
        actual = encStates[i]
        if actual[3]:
            actual = mirror(np.copy(actual))
        mse += dist(path[i], actual)
    mse = mse / len(path)
    print("ERROR:{}".format(mse))
    if (mse == 0):
        print("Attack succesful!")
    else:
        print("Attack failed")
    
def check_collision(path):
    global obstacles
    for point in path:
        px, py, pz = point
        for obs in obstacles:
            ox, oy, orad = obs
            if ( abs(px - ox) < orad and abs(py - oy) < orad ):
                return True
    return False 

def mirror(point):
    return [ -point[0], -point[1], 3-point[2] ]

def eve_split_trajs():
    #get the 2 possible trajectories
    global encStates
    path1 = [ encStates[0][:3] ]
    path2 = [ mirror(encStates[0]) ]
    for i in range(1, len(encStates)):
        p1last = path1[-1]
        p2last = path2[-1]
        if (dist(p1last, encStates[i]) < dist(p2last, encStates[i]) ):
            path1.append(encStates[i][:3] )
            path2.append( mirror(encStates[i]) )
        else:
            path2.append(encStates[i][:3] )
            path1.append( mirror(encStates[i]) )
    return path1, path2
    

def print_beacon_camera_diff(ranging_data=False):
    global cameraPos, beaconPos, beaconPos2, beaconDists
    dx = cameraPos[0] - beaconPos2[0]
    dy = cameraPos[1] - beaconPos2[1]
    dz = cameraPos[2] - beaconPos2[2]
    rospy.loginfo("camera... x:"+ str(cameraPos[0]) + " y:" + str(cameraPos[1]) + " z:" + str(cameraPos[2]))
    #rospy.loginfo("beaconSE... x:"+ str(beaconPos[0]) + " y:" + str(beaconPos[1]) + " z:" + str(beaconPos[2]))
    rospy.loginfo("beaconKF... x:"+ str(beaconPos2[0]) + " y:" + str(beaconPos2[1]) + " z:" + str(beaconPos2[2]))
    rospy.loginfo("diff... dx:"+ str(dx) + " dy:" + str(dy) + " dz:" + str(dz))
    if (not ranging_data):
        return
    rospy.loginfo("beaconDists...")
    for i in range(8):
    	rospy.loginfo(str(i) + ": " + str(beaconDists[i]) + "(mean:" + str(beaconMeans[i]) + ")")
    
def print_ts():
    global ts
    rospy.loginfo("times: ")
    #for i in range(0,5):
    #    rospy.loginfo("t{}: {}".format(i+1, ts[i]))
    rospy.loginfo("calculated deltas (s):\n delta_beacon:{}, delta_drone:{}, delta_p:{}".format((ts[2]-ts[1])/LOCODECK_TS_FREQ, (ts[4]-ts[3])/LOCODECK_TS_FREQ, ts[5]/LOCODECK_TS_FREQ))

def print_deltas(only_dp=False):
    global delta_bs, ts
    if not only_dp:
	rospy.loginfo("delta_bs: ")
	for i in range(6):
            rospy.loginfo("delta_b{}: {}".format(i, delta_bs[i]/LOCODECK_TS_FREQ))
	rospy.loginfo("delta_drone:{}".format((ts[4]-ts[3])/LOCODECK_TS_FREQ))
    #rospy.loginfo("delta_pl32:{}".format((ts[5])/LOCODECK_TS_FREQ))
    #rospy.loginfo("delta_ph8:{}".format((ts[6])/LOCODECK_TS_FREQ))
    rospy.loginfo("delta_p:{}".format(ticksToTime(ts[5], ts[6])) )

def print_twr_other():
    global singleRanging, allRangings, betweenRounds, betweenRangings, singleRanging_h8, allRangings_h8, betweenRounds_h8, betweenRangings_h8
    rospy.loginfo("singleRanging:{}".format(ticksToTime(singleRanging, singleRanging_h8)))
    rospy.loginfo("allRangings:{}".format(ticksToTime(allRangings, allRangings_h8)))
    rospy.loginfo("betweenRounds:{}".format(ticksToTime(betweenRounds, betweenRounds_h8)))
    rospy.loginfo("betweenRangings:{}".format(ticksToTime(betweenRangings, betweenRangings_h8)))

def print_twr_eve():
    global tofs
    dists = [x*3*10**8 for x in tofs]
    rospy.loginfo("ESTIMATE_ADD TOF:{}, DIST:{}".format(tofs[0],dists[0]))
    rospy.loginfo("ESTIMATE_MULT TOF:{}, DIST:{}".format(tofs[1],dists[1]))
    rospy.loginfo("ACTUAL_ADD TOF:{}, DIST:{}".format(tofs[2],dists[2]))
    rospy.loginfo("ACTUAL_MULT TOF:{}, DIST:{}".format(tofs[3],dists[3]))

def print_enc():
    global encState, beaconPos2, K1count, Ktotal
    rospy.loginfo("Beacon v Enc(K={}), K1%:{}".format(encState[3], 1.0*K1count/Ktotal))
    s = "xyz"
    for i in range(3):
        rospy.loginfo("{}  b:{},e:{}".format(s[i], beaconPos2[i], encState[i]))

def dist(s1, s2):
    return ((s1[0] - s2[0])**2 + (s1[1]-s2[1])**2 + (s1[2] - s2[2])**2)**(0.5)

def setpoints_to_time(s1, s2):
    return 1.2*dist(s1, s2)

def ticksToTime(ticks_low, ticks_high=0):
    return (ticks_low/LOCODECK_TS_FREQ) + ticks_high*(2**32/LOCODECK_TS_FREQ) 

if __name__ == '__main__':
    atexit.register(exit_handler)
    rospy.init_node('beaconTest', anonymous=True)
    worldFrame = rospy.get_param("~worldFrame", "/world")
    rate = rospy.Rate(10) #  hz
    rospy.Subscriber("external_position", PointStamped, callback_pos_camera)
    rospy.Subscriber("SE", GenericLogData, callback_pos_beacons)
    rospy.Subscriber("KF", GenericLogData, callback_pos_beacons2)
    rospy.Subscriber("Ranging1", GenericLogData, callback_beacon_ranging1)
    rospy.Subscriber("Ranging2", GenericLogData, callback_beacon_ranging2)
    rospy.Subscriber("TWRtime", GenericLogData, callback_twr_time)
    rospy.Subscriber("TWRbeacons", GenericLogData, callback_twr_beacon)
    rospy.Subscriber("TWRother", GenericLogData, callback_twr_other)
    rospy.Subscriber("TWReve", GenericLogData, callback_twr_eve)
    rospy.Subscriber("TWRenc", GenericLogData, callback_twr_enc)
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
    msgConsole.data = [ord('%'), ord('T'), ord('S'), 0]
    pubConsole = rospy.Publisher("cmd_console_msg", ConsoleMessage, queue_size=1)
    #msgConsole.data = [ord('%'), ord('T'), ord('S'), delta_p_param, 0, 0, 0] # CHANGE TDMA SLOT
    #msgConsole.data = [ord('%'), ord('O'), ord('F'), 0, 0, 0, 0] # CHANGE TO FIXED ORDER
    #msgConsole.data = [ord('%'), ord('O'), ord('R'), 0, 0, 0, 0] # CHANGE TO RANDOM ORDER
    #msgConsole.data = [ord('%'), ord('O'), ord(Order), 0, 0, 0, 0]
    msgConsole.data = [ord('%'), ord('S'), ord('F'), 0, 0, 0, 0]
    rospy.wait_for_service('update_params')
    rospy.loginfo("found update_params service")
    update_params = rospy.ServiceProxy('update_params', UpdateParams)
    #pubConsole.publish(msgConsole)
    rospy.set_param("kalman/resetEstimation", 1)
    update_params(["kalman/resetEstimation"])
    rospy.sleep(0.1)
    rospy.set_param("kalman/resetEstimation", 0)
    update_params(["kalman/resetEstimation"])
    rospy.sleep(0.1)
    pubConsole.publish(msgConsole)
    rospy.set_param("flightmode/posSet", 1)
    update_params(["flightmode/posSet"])
    rospy.sleep(3)


    pubConsole.publish(msgConsole)
    msgPublisher = Thread(target=publisherThread)
    msgPublisher.start()

    timeAlloted = 1.5
    print_beacon_camera_diff()

    rospy.sleep(2)


    #test code
    """
    while (True):
        rospy.sleep(1)
        #print_twr_other()
        #print_twr_eve()
        #print_ts()
        #print_deltas(True)
        #print_beacon_camera_diff(ranging_data=False)
        print_enc()
        #num = i
        #msgConsole.data = [ord('%'), ord('T'), ord('S'), num, 0, 0, 0]
        #pubConsole.publish(msgConsole)
        #rospy.sleep(2)
        #print_ts()

    if cameraPos[0] + cameraPos[1] == 0:
        err_handler()
    """

    """
    positionMove([beaconPos2[0],beaconPos2[1],0.1,0],.1, N=1) #takeoff
    rospy.loginfo("SHOULD BE IN AIR?!...")
    setpoint = [0,0,0.5,0]
    last_sp = setpoint
    positionMove(setpoint, 0.5, N=2) #zero x,y
    rospy.sleep(1)
    
    #square_setpoints = [ [0,0.5,0.5,0], [0.5,0.5,0.5,0],  [0.5,0,0.5,0], [0,0,0.5,0] ]
    #large_square = [ [1.5,0.5,0.5,0], [-1.5,0.5,0.5,0],  [-1.5,-0.5,0.5,0], [1.5,-0.5,0.5,0], [0,0,0.5,0] ]
    #large_square_difz = [ [1.5,0.5,0.4,0], [-1.5,0.5,0.7,0],  [-1.5,-0.5,0.5,0], [1.5,-0.5,0.3,0], [0,0,0.4,0] ]
    #triangle_setpoints = [ [0,0.5,0.5,0], [-0.3,0,0.5,0],  [0.3,0,0.5,0], [0,0.5,0.5,0] ]
    traj_setpoints = [ [0, 0, 0.5, 0], [-0.8, 0, 0.5, 0], [-.8, -0.6, 0.5, 0], [-0.7, -0.7, 0.5, 0], [0, -0.2, 0.5, 0], [0, 0, 0.5, 0] ]
    sp_list = traj_setpoints
    for i in range(len(sp_list)):
        setpoint = sp_list[i]
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
    """
    rospy.loginfo("Done.")
    for j in range(100):
        rospy.sleep(0.1)
        print_enc()
    #get_stats()
    save_enc_trace()
    

    exit_handler()

    

