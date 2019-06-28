#!/usr/bin/env python
import rospy
import tf


import atexit
from crazyflie_driver.msg import Position
from crazyflie_driver.msg import Hover
from crazyflie_driver.msg import GenericLogData
from std_msgs.msg import Empty
from crazyflie_driver.srv import UpdateParams
from threading import Thread
from geometry_msgs.msg import PointStamped, TransformStamped, PoseStamped #PoseStamped added to support vrpn_client



beaconPos = [0,0,0]
beaconPos2 = [0,0,0]
cameraPos = [0,0,0]
#x, y, z, yaw
currPos = [0,0,0,0] # current setpoint
STOP = False

def exit_handler():
    rospy.loginfo("///Exiting. sending stop command///")

    stop_pub = rospy.Publisher("cmd_stop", Empty, queue_size=1)
    stop_msg = Empty()
    STOP = True
    stop_pub.publish(stop_msg)

def beaconToCameraCoords(pos_b):
    # hopefully this is not needed anymore
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
    

def callback_pos_beacons(data):
    global beaconPos
    beaconPos[0] = data.values[0]
    beaconPos[1] = data.values[1]
    beaconPos[2] = data.values[2]

def callback_pos_beacons2(data):
    global beaconPos
    beaconPos2[0] = data.values[0]
    beaconPos2[1] = data.values[1]
    beaconPos2[2] = data.values[2]


def callback_pos_camera(data):
    global cameraPos
    cameraPos[0] = data.point.x
    cameraPos[1] = data.point.y
    cameraPos[2] = data.point.z

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

def positionMove(pos=[0,0,0,0], t=1):
    global currPos
    currPos = pos
    for i in range(10):
        rospy.sleep(t/10.0)
        print_beacon_camera_diff()

def print_beacon_camera_diff():
    global cameraPos, beaconPos, beaconPos2
    dx = cameraPos[0] - beaconPos[0]
    dy = cameraPos[1] - beaconPos[1]
    dz = cameraPos[2] - beaconPos[2]
    rospy.loginfo("camera... x:"+ str(cameraPos[0]) + " y:" + str(cameraPos[1]) + " z:" + str(cameraPos[2]))
    rospy.loginfo("beaconSE... x:"+ str(beaconPos[0]) + " y:" + str(beaconPos[1]) + " z:" + str(beaconPos[2]))
    rospy.loginfo("beaconKF... x:"+ str(beaconPos2[0]) + " y:" + str(beaconPos2[1]) + " z:" + str(beaconPos2[2]))
    row = [beaconPos[0], beaconPos[1], beaconPos[2], cameraPos[0], cameraPos[1], cameraPos[2]]

if __name__ == '__main__':
    atexit.register(exit_handler)
    rospy.init_node('beaconTest', anonymous=True)
    worldFrame = rospy.get_param("~worldFrame", "/world")
    rate = rospy.Rate(10) #  hz
    rospy.Subscriber("external_position", PointStamped, callback_pos_camera)
    rospy.Subscriber("log1", GenericLogData, callback_pos_beacons)
    rospy.Subscriber("log2", GenericLogData, callback_pos_beacons2) # another version of the position, but I think this one is worse

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
    
    msgPublisher = Thread(target=publisherThread)
    msgPublisher.start()


    timeAlloted = 10
    # take off
    positionMove([0,0,0,0],8)
    while (True):
        print_beacon_camera_diff()
        rospy.sleep(0.8)

    positionMove([0,0,0.4,0],3)
    rospy.loginfo("SHOULD BE IN AIR?!...")
    setpoint = [0,0,0.5,0]
    positionMove(setpoint, timeAlloted)
    rospy.loginfo("MOVING +y")
    print_beacon_camera_diff()
    setpoint = [0,0.5,0.5,0]
    positionMove(setpoint, timeAlloted)
    rospy.loginfo("MOVING -y")
    print_beacon_camera_diff()
    setpoint = [0,0,0.5,0]
    positionMove(setpoint, timeAlloted)
    rospy.loginfo("MOVING +x")
    print_beacon_camera_diff()
    setpoint = [0.5,0,0.5,0]
    positionMove(setpoint, timeAlloted)
    rospy.loginfo("MOVING -x")
    print_beacon_camera_diff()
    setpoint = [0,0,0.5,0]
    positionMove(setpoint, timeAlloted)
    #land
    positionMove([0,0,0.3,0.5],0.5)
    positionMove([0,0,0.2,0.3],0.1)
    print_beacon_camera_diff()
    rospy.loginfo("Done.")

    

