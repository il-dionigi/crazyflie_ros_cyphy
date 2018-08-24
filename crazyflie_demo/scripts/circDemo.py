#!/usr/bin/env python

#Drone 1: Forward/backward; (0,-0.6,0.4) <-> (0,0.6,0.4)
#Drone 2: Circle, center(0,0,0.4) radius(0.6). 
#Drone 2 stops + hovers when Drone 1 is near endpoints, (within 0.2?)
    #D2: PLACE AT x=+r. THEN GOES TO y=+r
    #D1: PLACE AT y=-r, THEN GOES TO y=+r

import rospy
import tf
from crazyflie_driver.msg import Position
from crazyflie_driver.msg import Hover
from crazyflie_driver.msg import GenericLogData
from std_msgs.msg import Empty
from crazyflie_driver.srv import UpdateParams
from threading import Thread
import math

cf2stop = False
cf1nextInteresect = []
cf1pos = [0,0,0,0]
cf2pos = [0,0,0,0]
cf1setpoint = [0,0,0,0]
maxERR = 1000

def callback_cf1pos(data):
    global cf1pos
    cf1pos[0] = data.values[0]
    cf1pos[1] = data.values[1]
    cf1pos[2] = data.values[2]

def callback_cf2pos(data):
    global cf2pos
    cf2pos[0] = data.values[0]
    cf2pos[1] = data.values[1]
    cf2pos[2] = data.values[2]


class Crazyflie:
    def __init__(self, prefix):
        self.prefix = prefix

        worldFrame = rospy.get_param("~worldFrame", "/world")
        self.rate = rospy.Rate(5)

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
    return abs( (v1[0] - v2[0])**2 + (v1[1]-v2[1])**2 )**(0.5)

def ylineNext(height, r, step, steps):
    x = 0
    direct = 1
    #step = 0 is intersect1, y=-r
    #step = steps/2 is intersect2, y=+r
    if step > steps/2:
        direct = -1
        step -= steps/2
    y = -r+r*4*step/steps
    y = y*direct
    return [x, y, height, 0]
    
def getDirect(step, steps):
    direct = 1
    if step > steps/2:
        direct = -1
    return direct

def circNext(height, r, step, steps):
    angle = 2*math.pi*step/steps
    x = r*math.cos(angle)
    y = r*math.sin(angle)
    yaw = angle
    return [x, y, height, yaw]

def getError(actual, theoretical):
    Err = [0,0,0,0]
    for i in range(len(Err)):
        Err[i] =  actual[i] - theoretical[i]
        Err[i] = int(Err[i]*1000)/1000.0
    return (Err)

def getErrCircle(actual, theoretical):
    guessPos = [0,0,0,0]
    err = getError(actual, theoretical)
    r = dist(err, [0,0,0,0])*2.0/3
    for i in range(len(guessPos)):
        guessPos[i] = (theoretical[i] + actual[i])/2
    return [guessPos, r]

def distErr(errCircleA, errCircleB):
    d = dist(errCircleA[0], errCircleB[0])
    d = d - errCircleA[1] - errCircleB[1]
    return max(0, d)

def cf2task(cf):
    global cf2stop, cf1nextInteresect, cf2pos, cf1pos, maxERR, cf1setpoint
    rate = rospy.Rate(5)
    #TEST POS
    cf2pos = [0,0,0,0]
    cf2setpoint = []
    #start setpoint, go to 0,6, 0, 0.6
    cf2setpoint = [0.6, 0, 0.4, 0]
    cf2nextIntersect = [0, 0.6, 0.4, 0]
    #Take off
    #cf.goToSetpoint([0, 0, 0.4, 0])
    radius = 0.6
    currentStep = 0
    divisions =  30
    stay = False
    ErrCirc2 = getErrCircle(cf2pos, cf2setpoint)
    #FOR TESTING PURPOSES:
    #CIRCLE STARTS AT x =+, THEN GOES TO y =+
    for i in range(3000):
        #print("internal/goal" + str(cf1pos) + "/[0,0,0,0]")
        print ("Sleeping T2 (drone)") 
        rate.sleep()
    for i in range(20):
        cf2setpoint = circNext(cf2setpoint[2], radius, currentStep, divisions)
        cf.goToSetpoint(cf2setpoint)
        rate.sleep()
    while(True):
        ErrCirc2 = getErrCircle(cf2pos, cf2setpoint)
        if cf2nextIntersect[1] > 0:
            print("circle going to y++")
        else:
            print("circle going to y--")
        error = dist(getError(cf2pos, cf2setpoint), [0,0,0,0])
        print("**c2 error " + str(error) + " r= " + str(ErrCirc2[1]))
        stay = False
        #get nextIntersect, but skew it so if it is too close it will keep on moving
        cf2nextIntersect[1] = -1*radius if divisions/4 < currentStep+divisions/20 < 3*divisions/4 else radius
        if cf2stop and cf2nextIntersect[1] == cf1nextInteresect[1]:
            d = distErr(ErrCirc2, [cf1nextInteresect, 0])
            if (d < 0.1):
                stay = True 
        d = dist(cf2pos, cf1pos)
        print("distance between drones: " + str(d))
        if d < 0.05:
            print("CRASH PREVENTION")
            cf2setpoint[2] = 0
        if error > maxERR:
            print("error is bad, circle will stay")
            stay = True
        if (stay):
            cf.goToSetpoint(cf2setpoint) #stay at position
        else:
            cf2setpoint = circNext(cf2setpoint[2], radius, currentStep, divisions)
            currentStep = (currentStep + 1 ) % divisions
            cf.goToSetpoint(cf2setpoint)
        #CIRCLE
        rate.sleep()
    return

def cf1task(cf):
    global cf2stop, cf1nextInteresect, cf1pos, maxERR, cf1setpoint
    rate = rospy.Rate(5)

    cf1pos = [0,0,0,0]
    #1=>going toward y=0.6, -1=>going toward y=-0.6
    direction = 1
    radius = 0.6
    divisions = 80
    currentStep = 0
    cf1nextInteresect = [0,0.6,0.4,0]
    stay = False
    cf1setpoint = ylineNext(cf1nextInteresect[2], radius, currentStep, divisions)
    ErrCirc1  = getErrCircle(cf1setpoint, cf1pos)
    #FOR TESTING PURPOSES:
    '''for i in range(3000):
        #print("internal/goal" + str(cf1pos) + "/[0,0,0,0]")
        print ("Sleeping T1 (drone)") 
        rate.sleep()'''
    #take off
    for i in range(20):
        #print("internal/goal" + str(cf1pos) + "/[0,0,0.4,0]") 
        print(cf1pos)
        rate.sleep()
    for i in range(4000):
        #print("internal/goal" + str(cf1pos) + "/[0,0,0.4,0]") 
        print(cf1pos)
        currentStep = (currentStep + 1) % divisions
        cf.goToSetpoint(cf1setpoint)
        if (i % (divisions/4) == 0):
            print("Dropping")
            for k in range(30):
                cf.goToSetpoint(cf1setpoint)
                rate.sleep()
            rate.sleep()

        rate.sleep()
    while(True):
        ErrCirc1  = getErrCircle(cf1setpoint, cf1pos)
        error = dist(getError(cf1pos, cf1setpoint), [0,0,0,0])
        print("*c1 error " + str(error) + " r= " + str(ErrCirc1[1]))
        stay = False
        if (error > maxERR):
            print("Error bad. line will stay")
            stay = True
        #print("internal/goal" + str(cf1pos) + "/" + str(cf1setpoint)) 
        cf1setpoint = ylineNext(cf1nextInteresect[2], radius, currentStep, divisions)
        direction = getDirect(currentStep, divisions)
        if not stay:
            currentStep = (currentStep + 1) % divisions
        cf.goToSetpoint(cf1setpoint)
        cf1nextInteresect[1] = direction*radius
        if (direction == 1):
            print("Line going to y+++")
        else:
            print("Line going to y---")
        #find out internal position, set cf1pos
        if ( distErr(ErrCirc1, [cf1nextInteresect, 0]) < 0.1):
            cf2stop = True
            print("cf2stop warn")
        elif cf2stop:
            cf1oldIntersect = cf1nextInteresect
            cf1oldIntersect[1] *= -1
            if distErr(ErrCirc1, [cf1nextInteresect, 0]) > 0.1:
                cf2stop = False
        rate.sleep()
    return

def cf1taskFake(cf):
    global cf2stop, cf1nextInteresect, cf1pos, maxERR, cf1setpoint
    rate = rospy.Rate(5)

    cf1pos = [0,0,0,0]
    #1=>going toward y=0.6, -1=>going toward y=-0.6
    direction = 1
    radius = 0.6
    divisions = 80
    currentStep = 0
    cf1nextInteresect = [0,0.6,0.4,0]
    stay = False
    cf1setpoint = ylineNext(cf1nextInteresect[2], radius, currentStep, divisions)
    ErrCirc1  = getErrCircle(cf1setpoint, cf1setpoint)
    #FOR TESTING PURPOSES:
    for i in range(30):
        #print("internal/goal" + str(cf1pos) + "/[0,0,0,0]")
        print ("Sleeping T1 (drone)") 
        rate.sleep()
    #take off
    while(True):
        ErrCirc1  = getErrCircle(cf1setpoint, cf1setpoint)
        error = dist(getError(cf1setpoint, cf1setpoint), [0,0,0,0])
        print("*c1 error " + str(error) + " r= " + str(ErrCirc1[1]))
        stay = False
        if (error > maxERR):
            print("Error bad. line will stay")
            stay = True
        #print("internal/goal" + str(cf1pos) + "/" + str(cf1setpoint)) 
        cf1setpoint = ylineNext(cf1nextInteresect[2], radius, currentStep, divisions)
        direction = getDirect(currentStep, divisions)
        if not stay:
            currentStep = (currentStep + 1) % divisions
        #cf.goToSetpoint(cf1setpoint)
        cf1nextInteresect[1] = direction*radius
        if (direction == 1):
            print("Line going to y+++")
        else:
            print("Line going to y---")
        #find out internal position, set cf1pos
        if ( distErr(ErrCirc1, [cf1nextInteresect, 0]) < 0.1):
            cf2stop = True
            print("cf2stop warn")
        elif cf2stop:
            cf1oldIntersect = cf1nextInteresect
            cf1oldIntersect[1] *= -1
            if distErr(ErrCirc1, [cf1nextInteresect, 0]) > 0.1:
                cf2stop = False
                print("cf2 can go ")
        rate.sleep()
    return


def cf2taskFake(cf):
    global cf2stop, cf1nextInteresect, cf2pos, cf1pos, maxERR, cf1setpoint
    rate = rospy.Rate(5)
    #TEST POS
    cf2pos = [0,0,0,0]
    cf2setpoint = []
    #start setpoint, go to 0,6, 0, 0.6
    cf2setpoint = [0.6, 0, 0.4, 0]
    cf2nextIntersect = [0, 0.6, 0.4, 0]
    #Take off
    #cf.goToSetpoint([0, 0, 0.4, 0])
    radius = 0.6
    currentStep = 0
    divisions =  30
    stay = False
    ErrCirc2 = getErrCircle(cf2setpoint, cf2setpoint)
    #FOR TESTING PURPOSES:
    #CIRCLE STARTS AT x =+, THEN GOES TO y =+
    for i in range(20):
        print("Sleeping t2")
        rate.sleep()
    while(True):
        ErrCirc2 = getErrCircle(cf2setpoint, cf2setpoint)
        if cf2nextIntersect[1] > 0:
            print("circle going to y++")
        else:
            print("circle going to y--")
        error = dist(getError(cf2setpoint, cf2setpoint), [0,0,0,0])
        stay = False
        #get nextIntersect, but skew it so if it is too close it will keep on moving
        cf2nextIntersect[1] = -1*radius if divisions/4 < currentStep+divisions/20 < 3*divisions/4 else radius
        if cf2stop and cf2nextIntersect[1] == cf1nextInteresect[1]:
            d = distErr(ErrCirc2, [cf1nextInteresect, 0])
            if (d < 0.1):
                stay = True 
        d = dist(cf2setpoint, cf1setpoint)
        print("distance between drones: " + str(d))
        if d < 0.05:
            print("CRASH PREVENTION")
            cf2setpoint[2] = 0
        if error > maxERR:
            print("error is bad, circle will stay")
            stay = True
        if (stay):
            cf.goToSetpoint(cf2setpoint) #stay at position
        else:
            cf2setpoint = circNext(cf2setpoint[2], radius, currentStep, divisions)
            currentStep = (currentStep + 1 ) % divisions
            cf.goToSetpoint(cf2setpoint)
        #CIRCLE
        rate.sleep()
    return

if __name__ == '__main__':
    rospy.init_node('position', anonymous=True)

    cf1 = Crazyflie("cf1")
    cf2 = Crazyflie("cf2")
    rospy.Subscriber("cf1/log1", GenericLogData, callback_cf1pos)
    rospy.Subscriber("cf2/log2", GenericLogData, callback_cf2pos)
    print("STARTING THREADS")
    #t1 = Thread(target=cf1task, args=(cf1,))
    #t2 = Thread(target=cf2task, args=(cf2,))

    t1 = Thread(target=cf1taskFake, args=(cf1,))
    t2 = Thread(target=cf2taskFake, args=(cf2,))

    t1.start()
    t2.start()
    t1.join()
    t2.join()



