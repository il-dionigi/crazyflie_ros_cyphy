#!/usr/bin/env python

import rospy
import tf
from crazyflie_driver.msg import Position
from crazyflie_driver.msg import Hover
from crazyflie_driver.msg import GenericLogData
from std_msgs.msg import Empty
from crazyflie_driver.srv import UpdateParams
from threading import Thread

internalPos = [0,0,0,0]
#x, y, z, yaw
currPos = [0,0,0,0]
#vx, vy, yawrate
vel = [0,0,0]
posMode = True
STOP = False

def callback_pos(data):
    global internalPos
    internalPos[0] = data.values[0]
    internalPos[1] = data.values[1]
    internalPos[2] = data.values[2]

def publisherThread():
    global internalPos
    sequence = 0
    global currPos, vel, posMode
    while not rospy.is_shutdown():
        if STOP:
            return
        if (posMode):
            msgPos.x = currPos[0]
            msgPos.y = currPos[1]
            msgPos.z = currPos[2]
            msgPos.yaw = currPos[3]
            msgPos.header.seq = sequence
            msgPos.header.stamp = rospy.Time.now()
            pubPos.publish(msgPos)
        else:
            msgHov.vx = vel[0]
            msgHov.vy = vel[1]
            msgHov.yawrate = vel[1]
            msgHov.zDistance = currPos[2]
            msgHov.header.seq = sequence
            msgHov.header.stamp = rospy.Time.now()
            pubHov.publish(msgHov)

        #rospy.loginfo("sending...(M)")
        #rospy.loginfo("x:"+ str(msgPos.x) + " y:" + str(msgPos.y) + " z:" + str(msgPos.z))
        #rospy.loginfo(msgPos.x)
        #rospy.loginfo(msgPos.y)
        #rospy.loginfo(msgPos.z)
        #rospy.loginfo(msgPos.yaw)
        sequence += 1
        rate.sleep() 

def positionMove(pos=[0,0,0,0], t=3):
    global currPos
    currPos = pos
    rospy.sleep(t)

def setVel(velocity=[0,0,0], t=1):
    global vel
    vel = velocity
    rospy.sleep(t)

if __name__ == '__main__':
    rospy.init_node('position', anonymous=True)
    worldFrame = rospy.get_param("~worldFrame", "/world")
    inp = 'x'
    while (inp not in "hHpP"):
        inp = raw_input("Would you like to use Hover mode (Hh) or Position mode (Pp): ")
    if (inp in "pP"):
        posMode = True
        print("Using Position mode!")
    else:
        posMode = False
        print("Using Hover mode!")
    rate = rospy.Rate(10) #  hz
    rospy.Subscriber("log1", GenericLogData, callback_pos)
    #x = rospy.get_param("kalman.stateX")
    #print(" GOT X. X IS: "+ str(x) + "\n*****\n*****\n")
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
    #for velocity/hover mode
    msgHov = Hover()
    msgHov.header.seq = 0
    msgHov.header.stamp = rospy.Time.now()
    msgHov.header.frame_id = worldFrame
    msgHov.vx = 0.0
    msgHov.vy = 0.0
    msgHov.zDistance = 0.0
    msgHov.yawrate = 0.0
    pubHov = rospy.Publisher("cmd_hover", Hover, queue_size=1)
    #to stop, don't really need this?
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
    msgPublisher = Thread(target=publisherThread)
    msgPublisher.start()
    rospy.loginfo("START TAKEOFF...")
    # take off
    positionMove([0,0,0.4,0],2)

    print("What would you like to do?\n")
    timeAlloted = 3
    showAll = False
    while (inp not in "xX"):
        if (not showAll):
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
            (vV) set velocities
            (mM) Send a message
            (eE) Send encrypted message
            (dD) Display internal position
            ''')
        if (inp in "xX"):
            STOP = True
            stop_pub.publish(stop_msg)
            exit()
        elif (inp in "lL"):
            setVel([0,0,0], 0)
            positionMove([0,0,0,0],2)
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
                print("Hover mode on!")
            elif (inp in 'pP'):
                posMode = True
                print("Position mode on!")
            elif (inp in 'eE'):
                showAll = False
                print("Done")
            elif (inp in 'dD'):
                showAll = True
                print("Done")
        elif (inp in 'sS'):
            inp = raw_input("Enter setpoint in comma-separated form: x,y,z,yaw\n")
            setpoint = [float(x) for x in inp.split(',')]
            print("your setpoint is: " + str(setpoint))
            positionMove(setpoint, timeAlloted)
        elif (inp in 'vV'):
            inp = raw_input("Enter velocity in comma-separated form: vx,vy,yawrate\n")
            velocity = [float(x) for x in inp.split(',')]
            print("your velocity is: " + str(velocity))
            setVel(velocity, timeAlloted)
        elif (inp in 'dD'):
            print("x: " + str(internalPos[0]) + "y: " + str(internalPos[1]) + "z: " + str(internalPos[2]))
        else:
            print("Currently not implemented.")

    stop_pub.publish(stop_msg)

