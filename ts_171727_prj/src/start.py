#!/usr/bin/env python

#Standart Libs
import time     #TimeTracking
import math     #Simple Calculations
import tf       #Transforming of Coordinated
import copy     #Coping of Lists

#ros Libs
import rospy
import actionlib
##Output
from geometry_msgs.msg import Twist         # Import the Twist message from the geometry_msgs to move the bot
##Input
from sensor_msgs.msg import LaserScan       # Import Laser Data Message
from gazebo_msgs.msg import ModelStates     # Import Message for Odom
from goal_publisher.msg import PointArray   # Import Goal Message
from geometry_msgs.msg import Point         # Import Point Message

#Storage for Callback Data
global scandata
global modelstatesdata
global goals

global movex,movez,movexact,movezact

#Callbacks for Data Storage
def callback_laser(cb_para):
    global scandata
    scandata = cb_para
    return

def callback_modelstates(cb_para):
    global modelstatesdata
    modelstatesdata = cb_para
    return

def callback_goal(cb_para):
    global goals
    goals = cb_para
    return

#Calculate Distance between to Points
def calc_distancebetweenpoints(p1,p2):
    return abs(math.hypot(p2.x - p1.x, p2.y - p1.y))

#Calculate the Distance to the Goal from Current Position
def calc_distogoal(goal):
    return calc_distancebetweenpoints(get_selfpos(),goal)

#Get current Bot Pos
def get_selfpos():
    index=modelstatesdata.name.index("turtlebot3_burger")
    pos = modelstatesdata.pose[index]
    return pos.position

#get Heading of the Bot
def get_selfheading():
    index=modelstatesdata.name.index("turtlebot3_burger")
    pos = modelstatesdata.pose[index]
    return pos.orientation

#Returns the Difference (in Deg) between where the bot is looking and where the Target is.
def get_degtotarget(targetpoint):

    #Basicly Converts some Quaternion to simple degrees
    self_heading=get_selfheading()
    explicit_quat = [self_heading.x,self_heading.y,self_heading.z,self_heading.w]
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(explicit_quat)
    bot_deg=yaw*180 / math.pi
    if bot_deg<0:
        bot_deg+=360

    #Absolute Difference between bot and Target
    bottotarget_deg =math.degrees(math.atan2(targetpoint.y-get_selfpos().y , targetpoint.x-get_selfpos().x ))
    if bottotarget_deg<0:
        bottotarget_deg+=360

    unterschied = abs(bottotarget_deg) - abs(bot_deg)
    return unterschied

#Checks if the "Pos" Parameter is between Parameter a and b (all 3 are x,y,z Objects)
def check_isposbetween(a,b,pos):
    return abs(calc_distancebetweenpoints(a,pos) + calc_distancebetweenpoints(pos,b) - calc_distancebetweenpoints(a,b))<0.4

#Checks if the Front 45Deg. are free within the RangeLimit
def check_isfrontfree(rangelim):

    tempdata = LaserScan()
    tempdata = scandata                 #reading global scandata variable

    leftborder=len(tempdata.ranges)-len(tempdata.ranges)/32
    rightborder=len(tempdata.ranges)/32

    isfree=True
    i=0
    for wert in tempdata.ranges:
        if (i>leftborder or i<rightborder) and (wert<rangelim or wert<tempdata.range_min):
            isfree=False
            break

        i=i+1
    return isfree

#Checks if the Left 45Deg. are free within the RangeLimit
def check_isleftfree(rangelim):

    tempdata = LaserScan()
    tempdata = scandata                 #reading global scandata variable

    leftborder=(len(tempdata.ranges)/8)*1
    rightborder=(len(tempdata.ranges)/8)*2

    isfree=True
    i=0
    for wert in tempdata.ranges:
        if i>leftborder and i<rightborder and (wert<rangelim or wert<tempdata.range_min):
            isfree=False
            break

        i=i+1

    return isfree

#Sends the Move (Twist) Command to the Topic
def do_move(fwd,turn):
    success=True
    global movexact,movezact
    msg = Twist()

    #Slowly accelerate/slow down bot
    #if fwd under -1: indicates that the Forward speed should not be touched!
    if movexact<fwd and fwd > -1:
        movexact=movexact+0.05
    elif movexact>fwd and fwd > -1:
        movexact=movexact-0.05

    if movezact<turn:
        movezact=movezact+0.05
    elif movezact>turn:
        movezact=movezact-0.05

    #STOP NOW
    if fwd==0 and turn ==0:
        movexact=0
        movezact=0

    #Reverse is not Allowed! ;)
    if movexact<0:
        movexact=0

    msg.linear.x=round(float(movexact),2)
    msg.linear.y=0
    msg.linear.z=0
    msg.angular.z=round(float(movezact),2)
    msg.angular.x=0
    msg.angular.y=0
    pub.publish(msg)

    return success

#Turns the Bot, so its Front faces the Target
def do_turntotarget(targetpoint):
    while(abs(get_degtotarget(targetpoint))>5):
        #if to right turn left, otherwise turn right
        if get_degtotarget(targetpoint)>1:
            #the -99 indicates that the Forward speed should not be touched
            do_move(-99,0.5)
        else:
            do_move(-99,-0.5)

    return

#Moves the Bot to the target(point) while avoiding Obstacles
#Idea from Bug Algo v2 Src: https://www.cs.cmu.edu/~motionplanning/lecture/Chap2-Bug-Alg_howie.pdf
def do_movetotarget(targetpoint):

    startpoint = get_selfpos()

    #While no Obstacles are in the Way
    while(calc_distancebetweenpoints(get_selfpos(),targetpoint)>0.4):
        rospy.loginfo("Moving Forward Mode!")
        while check_isfrontfree(1.2) and calc_distancebetweenpoints(get_selfpos(),targetpoint)>0.3:
            do_turntotarget(targetpoint)

            #moveforward until obstacle
            do_move(0.7,0)
        rospy.loginfo("Obstacle in Front detected!")
        do_move(0,0)

        #Circumvent Obstacle
        rospy.loginfo("Circumvent Mode!")
        incircumventmode=True
        startdistanz=calc_distancebetweenpoints(get_selfpos(),targetpoint)
        while incircumventmode and calc_distancebetweenpoints(get_selfpos(),targetpoint)>0.3:
            if(check_isfrontfree(1.2) and check_isleftfree(1.5)):
                #turn left - wrong way! Turn around
                do_move(0.4,0.6)
            if(not check_isfrontfree(1.2) and check_isleftfree(1.5)):
                #hard right - not parralell to wall!
                do_move(0.4,-1)
            if(check_isfrontfree(1.2) and not check_isleftfree(1.5)):
                #forward - im Following the wall!
                do_move(0.6,0)
            if(not check_isfrontfree(1.2) and not check_isleftfree(1.5)):
                #hard right - at a corner! left the wall behind us!
                do_move(0.4,-1)

            #check if near the Line to the targetpoint, if so -> go directly to Target!
            incircumventmode=not(check_isposbetween(targetpoint,startpoint,get_selfpos()) and ((startdistanz-calc_distancebetweenpoints(get_selfpos(),targetpoint)) > 0.3))
        rospy.loginfo("Done Circumventing!")

        do_move(0,0)

    return

def main():

    #Initialize Global Variables used for the Slow Acceleration
    global movex,movez,movexact,movezact
    movex=0
    movez=0
    movexact=0
    movezact=0

    #Initialize the Callback Variables
    global scandata
    scandata = LaserScan()
    global modelstatesdata
    modelstatesdata = ModelStates()
    global goals
    goals = PointArray()
    mygoals = PointArray()

    #Wait until first Goal is published
    rospy.loginfo("Wait until first Goal is published")
    while(len(goals.goals)==0):
        time.sleep(1)

    rospy.loginfo("Found first Goals!")
    mygoals=copy.deepcopy(goals)

    while (len(mygoals.goals)>0):
        #Sort by goal distance what is the nearest Goal
        mygoals.goals.sort(key=calc_distogoal)
        #Then Select the nearest one and execute!
        rospy.loginfo("New Target x:"+str(mygoals.goals[0].x)+" y: "+str(mygoals.goals[0].y)+" z: "+str(mygoals.goals[0].z))
        do_movetotarget(mygoals.goals[0])
        mygoals.goals.pop(0)

    rospy.loginfo("Done with all Goals! Hurray!!!!")
    return 1

rospy.init_node('TurtelLogic')

pub = rospy.Publisher('cmd_vel', Twist) # Create a Publisher object, that will publish the movement
r = rospy.Rate(4)

rospy.Subscriber('scan', LaserScan, callback_laser) #Subscriber to Laserscan
rospy.Subscriber('/gazebo/model_states', ModelStates, callback_modelstates) #Subscriber to ModelStates
rospy.Subscriber('/goals', PointArray, callback_goal) #Subscriber to ModelStates

main()
