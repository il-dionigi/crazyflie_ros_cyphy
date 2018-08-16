#!/usr/bin/env python

import rospy
import tf
from crazyflie_driver.msg import Position
from crazyflie_driver.msg import Hover
from crazyflie_driver.msg import GenericLogData
from std_msgs.msg import Empty
from crazyflie_driver.srv import UpdateParams
from threading import Thread
import math

cfSetpoint = []
cfPos = [0,0,0,0]
MAX_DISTANCE = 1.22

def callback_cf_pos(data):
    global cfPos
    print("internal " + str(cfPos))
    cfPos[0] = data.values[0]
    cfPos[1] = data.values[1]
    cfPos[2] = data.values[2]

class Crazyflie:
    def __init__(self, prefix):
        self.prefix = prefix

        worldFrame = rospy.get_param("~worldFrame", "/world")
        self.rate = rospy.Rate(10)

        rospy.wait_for_service(prefix + '/update_params')
        rospy.loginfo("found update_params service")
        self.update_params = rospy.ServiceProxy(prefix + '/update_params', UpdateParams)

        self.setParam("kalman/resetEstimation", 1)
        self.setParam("flightmode/posSet", 1)

        self.pub = rospy.Publisher(prefix + "/cmd_setpoint", Position, queue_size=1)
        self.msg = Position()
        self.msg.header.seq = 0
        self.msg.header.stamp = rospy.Time.now()
        self.msg.header.frame_id = worldFrame
        self.msg.x = 0
        self.msg.y = 0
        self.msg.z = 0
        self.msg.yaw = 0

        self.stop_pub = rospy.Publisher(prefix + "/cmd_stop", Empty, queue_size=1)
        self.stop_msg = Empty()

    def setParam(self, name, value):
        rospy.set_param(self.prefix + "/" + name, value)
        self.update_params([name])

    def goToSetpoint(self, setpoint):
        self.msg.x = setpoint[0]
        self.msg.y = setpoint[1]
        self.msg.z = setpoint[2]
        self.msg.yaw = setpoint[3]
        self.msg.header.seq += 1
        self.msg.header.stamp = rospy.Time.now()
        self.pub.publish(self.msg)

def dist(v1, v2):
    return abs( (v1[0] - v2[0])**2 + (v1[1]-v2[1])**2 + (v1[2] - v1[2])**2 )**(0.5)

def slitherNext(height, r, step, steps):
    angle = 2 * math.pi * step / steps
    x = MAX_DISTANCE * step / steps
    y = r * math.sin(angle)
    yaw = 0
    return [x, y, height, yaw]

def landSequence(cf):
    tries = 0
    while (tries < 10):
        cf.goToSetpoint([0.0, 0.0, 0.4, 0])
        tries = tries + 1
    cf.goToSetpoint([0.0, 0.0, 0.4, 0])
    cf.alloff()

def cf_task(cf):
    global cfSetpoint, cfPos
    rate = rospy.Rate(10)
    cfPos = [0,0,0,0]
    cfSetpoint = []
    #start setpoint, go to 0.0, 0.0, 0.4
    cfSetpoint = [0.0, 0.0, 0.4, 0]
    #Take off
    cf.goToSetpoint(cfSetpoint)

    radius = 0.305
    currentStep = 0
    divisions = 300
    while(currentStep < (2 * divisions) ):
        print("goal " + str(cfSetpoint))
        cfSetpoint = slitherNext(cfSetpoint[2], radius, currentStep, divisions)
        currentStep = currentStep + 1
        cf.goToSetpoint(cfSetpoint)
        rate.sleep()
    landSequence(cf)
    return

if __name__ == '__main__':
    rospy.init_node('position', anonymous=True)

    cf = Crazyflie("cf")
    print("CF!")
    rospy.Subscriber("cf/log", GenericLogData, callback_cf_pos)
    cf_task(cf)