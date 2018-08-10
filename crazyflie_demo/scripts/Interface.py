#!/usr/bin/env python

import rospy
import tf
from crazyflie_driver.msg import Position
from std_msgs.msg import Empty
from crazyflie_driver.srv import UpdateParams
from threading import Thread

currPos = [0,0,0,0]    
def publishPos():
    global currPos
    while not rospy.is_shutdown():
        msgPos.x = currPos[0]
        msgPos.y = currPos[1]
        msgPos.yaw = currPos[3]
        msgPos.z = currPos[2]
        now = rospy.get_time()
        msgPos.header.seq += 1
        msgPos.header.stamp = rospy.Time.now()
        #rospy.loginfo("sending...(M)")
        #rospy.loginfo("x:"+ str(msgPos.x) + " y:" + str(msgPos.y) + " z:" + str(msgPos.z))
        #rospy.loginfo(msgPos.x)
        #rospy.loginfo(msgPos.y)
        #rospy.loginfo(msgPos.z)
        #rospy.loginfo(msgPos.yaw)
        pub.publish(msgPos)
        rate.sleep() 

def positionMove(pos, t):
    global currPos
    currPos = pos
    rospy.sleep(t)

if __name__ == '__main__':
    rospy.init_node('position', anonymous=True)
    worldFrame = rospy.get_param("~worldFrame", "/world")
    inp = 'x'
    while (inp not in "hHpP"):
        inp = raw_input("Would you like to use Hover mode (Hh) or Position mode (Pp): ")
    if (inp in "hH"):
        name = "cmd_setpoint"
        print("Using Position mode!")
    else:
        name = "cmd_hover" # will implemet later
        print("Currently not implemented, will use position mode!")
        name = "cmd_setpoint"
        print("Using Position mode!")
    rate = rospy.Rate(10) #  hz

    msgPos = Position()
    msgPos.header.seq = 0
    msgPos.header.stamp = rospy.Time.now()
    msgPos.header.frame_id = worldFrame
    msgPos.x = 0.0
    msgPos.y = 0.0
    msgPos.z = 0.0
    msgPos.yaw = 0.0

    pub = rospy.Publisher(name, Position, queue_size=1)

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
    while (inp not in "yY"):
        inp = raw_input("Ready to takeoff (yY)?: ")
    msgPublisher = Thread(target=publishPos)
    msgPublisher.start()
    rospy.loginfo("START TAKEOFF...")
    # take off
    positionMove([0,0,0.4,0],2)

    print("What would you like to do?\n")
    timeAlloted = 3
    showAll = False
    posMode = True
    while (inp not in "lL"):
        if (showAll):
            inp = raw_input('''
            (sS) Send a setpoint
            (cC) Change the current settings
            (lL) Land
            (x) Immediately abort
            ''')
        else:
            inp = raw_input('''
            (sS) Send a setpoint
            (cC) Change the current settings
            (lL) Land
            (x) Immediately abort
            (mM) Send a message
            (eE) Send encrypted message
            ''')
        if (inp in "xX"):
            stop_pub.publish(stop_msg)
            exit()
        elif (inp in "lL"):
            break
        elif (inp in "cC"):
            inp = raw_input('''
    Which setting would you like to change
        (tT) time allotted to each setpoint
        (hH) set to hover mode
        (pP) set to position mode
        (eE) enable advanced commands 
        (dD) disable advanced commands
            ''')
            if (inp in 'tT'):
                inp = raw_input("how much time should each command stay for (in seconds): ")
                try:
                    timeAllotted = float(inp)
                    print("Changed, now time is " + str(timeAllotted) + "s")
                except ValueError:
                    print("ERR: reverting to 3s")
            elif (inp in 'hH'):
                posMode = False
                print("currently not supported.")
                posMode = True
            elif (inp in 'pP'):
                posMode = True
                print("Position mode on!")
            elif (inp in 'eE'):
                showAll = True
                print("Done")
            elif (inp in 'dD'):
                showAll = False
                print("Done")
        elif (inp in 'sS'):
            inp = raw_input("Enter setpoint in comma-separated form: x,y,z,yaw\n")
            setpoint = [float(x) for x in inp.split(',')]
            print("your setpoint is: " + str(setpoint))
            positionMove(setpoint, timeAlloted)
        else:
            print("Currently not implemented.")

    # land, spend 2 secs
    positionMove([0,0,0,0],2)

    stop_pub.publish(stop_msg)

