#!/usr/bin/env python

import rospy
import tf
from crazyflie_driver.msg import Position
from crazyflie_driver.msg import Hover
from crazyflie_driver.msg import GenericLogData
from std_msgs.msg import Empty
from crazyflie_driver.srv import UpdateParams
from threading import Thread

beaconPos = [0,0,0]
cameraPos = [0,0,0]
#x, y, z, yaw
currPos = [0,0,0,0] # current setpoint
STOP = False

def callback_pos_beacons(data):
    global beaconPos
    beaconPos[0] = data.values[0]
    beaconPos[1] = data.values[1]
    beaconPos[2] = data.values[2]

def callback_pos_camera(data):
    global cameraPos
    cameraPos[0] = data.point.x
    cameraPos[1] = data.point.x
    cameraPos[2] = data.point.x

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
    rospy.sleep(t)

def print_beacon_camera_diff():
    global cameraPos, beaconPos
    dx = cameraPos[0] - beaconPos[0]
    dy = cameraPos[1] - beaconPos[1]
    dz = cameraPos[2] - beaconPos[2]
    rospy.loginfo("dx:"+ str(dx) + " dy:" + str(dy) + " dz:" + str(dz))


if __name__ == '__main__':
    rospy.init_node('beaconTest', anonymous=True)
    worldFrame = rospy.get_param("~worldFrame", "/world")
    rate = rospy.Rate(10) #  hz
    rospy.Subscriber("external_position", PointStamped, callback_pos_camera)
    rospy.Subscriber("log2", GenericLogData, callback_pos_beacons)
    
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
    #to stop
    stop_pub = rospy.Publisher("cmd_stop", Empty, queue_size=1)
    stop_msg = Empty()

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
    rospy.sleep(0.5)
    
    msgPublisher = Thread(target=publisherThread)
    msgPublisher.start()

    rospy.loginfo("START TAKEOFF...")
    timeAlloted = 2
    # take off
    positionMove([0,0,0,0],1)
    positionMove([0,0,0.4,0],3)
    rospy.loginfo("SHOULD BE IN AIR?!...")
    setpoint = [0,0,0.5,0]
    positionMove(setpoint, timeAlloted)
    setpoint = [0,0.5,0.5,0]
    positionMove(setpoint, timeAlloted)
    setpoint = [0,0,0.5,0]
    positionMove(setpoint, timeAlloted)
    setpoint = [0.5,0,0.5,0]
    positionMove(setpoint, timeAlloted)
    #land
    positionMove([0,0,0,0],0.1)
    STOP = True
    stop_pub.publish(stop_msg)

