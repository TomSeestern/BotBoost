#!/usr/bin/env python

#Standart Libs
import rospy
import actionlib
import time
import math
import tf
import copy

#ros Libs
##Output
from geometry_msgs.msg import Twist         # Import the Twist message from the geometry_msgs to move the bot
##Input
from sensor_msgs.msg import LaserScan       # Import Laser Data Message
from gazebo_msgs.msg import ModelStates     # Import Message for Odom
from goal_publisher.msg import PointArray   # Import Goal Message
from geometry_msgs.msg import Point         # Import Point Message


global scandata
global modelstatesdata
global goals
global movex,movez,movexact,movezact

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

def is_between(a,b,currpos):
    return abs(calc_distancebetweenpoints(a,currpos) + calc_distancebetweenpoints(currpos,b) - calc_distancebetweenpoints(a,b))<0.4

def calc_distancebetweenpoints(p1,p2):
    return abs(math.hypot(p2.x - p1.x, p2.y - p1.y))

def calc_distogoal(goal):
    return calc_distancebetweenpoints(get_selfpos(),goal)

def get_selfpos():
    index=modelstatesdata.name.index("turtlebot3_burger")
    pos = modelstatesdata.pose[index]
    return pos.position

def get_selfheading():
    index=modelstatesdata.name.index("turtlebot3_burger")
    pos = modelstatesdata.pose[index]
    return pos.orientation

def frontisfree(meterlim):
    #Checks if the 360/5 in front of the bot are free
    tempdata = LaserScan()
    tempdata = scandata                 #reading global scandata variable

    leftborder=len(tempdata.ranges)-len(tempdata.ranges)/8
    rightborder=len(tempdata.ranges)/8

    #rospy.loginfo("scanning l: "+str(leftborder)+" r: "+str(rightborder)+" in deg: "+str(leftborder*tempdata.angle_increment)+" - "+str(leftborder*tempdata.angle_increment))
    isfree=True
    i=0
    for wert in tempdata.ranges:
        if (i>leftborder or i<rightborder) and (wert<meterlim or wert<tempdata.range_min):
            isfree=False
            #rospy.loginfo("Front of Robot is not free! Error bei index: "+str(i))
            break

        i=i+1
    return isfree


def leftisfree(meterlim):
    #Checks if the 360/5 in front of the bot are free
    tempdata = LaserScan()
    tempdata = scandata                 #reading global scandata variable

    leftborder=(len(tempdata.ranges)/8)*1
    rightborder=(len(tempdata.ranges)/8)*2

    #rospy.loginfo("scanning l: "+str(leftborder)+" r: "+str(rightborder)+" in deg: "+str(leftborder*tempdata.angle_increment)+" - "+str(leftborder*tempdata.angle_increment))
    isfree=True
    i=0
    for wert in tempdata.ranges:
        if i>leftborder and i<rightborder and (wert<meterlim or wert<tempdata.range_min):
            isfree=False
            #rospy.loginfo("right of Robot is not free! Error bei index: "+str(i))
            break

        i=i+1

    return isfree

def get_gradzumziel(targetpoint):

    self_heading=get_selfheading()
    explicit_quat = [self_heading.x,self_heading.y,self_heading.z,self_heading.w]
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(explicit_quat)
    bot_deg=yaw*180 / math.pi
    if bot_deg<0:
        bot_deg+=360

    #bottotarget_deg = math.degrees(math.atan2(targetpoint.y-get_selfheading().y , targetpoint.x - get_selfheading().x ))
    bottotarget_deg =math.degrees(math.atan2(targetpoint.y-get_selfpos().y , targetpoint.x-get_selfpos().x ))
    if bottotarget_deg<0:
        bottotarget_deg+=360

    unterschied = abs(bottotarget_deg) - abs(bot_deg)


    #print("Botdeg: "+str(bot_deg)+" toTrg "+str(bottotarget_deg)+" div "+str(unterschied))
    return unterschied

def move(fwd,turn):
    success=True
    global movexact,movezact
    msg = Twist()

    if movexact<fwd:
        movexact=movexact+0.05
    elif movexact>fwd:
        movexact=movexact-0.05

    if movezact<turn:
        movezact=movezact+0.01
    elif movezact>turn:
        movezact=movezact-0.01

    #STOP NOW
    if fwd==0 and turn ==0:
        movexact=0
        movezact=0

    if movexact<0:
        movexact=0


    msg.linear.x=round(float(movexact),2)
    msg.linear.y=0
    msg.linear.z=0
    msg.angular.z=round(float(movezact),2)
    msg.angular.x=0
    msg.angular.y=0
    pub.publish(msg)

    #print("gew. x: "+str(fwd) + " movexact: "+str(movexact)+" gew. z: "+str(turn) +" movezact: "+str(movezact))

    return success

def turn_totarget(targetpoint):


    while(abs(get_gradzumziel(targetpoint))>5):
        if get_gradzumziel(targetpoint)>1:
            move(0,0.2)
        else:
            move(0,-0.2)
        #print("grad zum ziel: "+str(get_gradzumziel(targetpoint))+" Entf z.Z. "+str(calc_distancebetweenpoints(get_selfpos(),targetpoint)))

    #move(0,0)
    return


def move_totarget(targetpoint):
    #Idea from Bug Algo v2 Src: https://www.cs.cmu.edu/~motionplanning/lecture/Chap2-Bug-Alg_howie.pdf

    startpoint = get_selfpos()

    while(calc_distancebetweenpoints(get_selfpos(),targetpoint)>0.4):

        #While no Obstacles are in the Way
        rospy.loginfo("Moving Forward Mode!")
        while frontisfree(0.5) and calc_distancebetweenpoints(get_selfpos(),targetpoint)>0.4:
            #turn to targetpoint
            turn_totarget(targetpoint)
            #print("grad zum ziel: "+str(get_gradzumziel(targetpoint))+" Entf z.Z. "+str(calc_distancebetweenpoints(get_selfpos(),targetpoint)))

            #moveforward until obstacle
            move(0.7,0)
        rospy.loginfo("Obstacle in Front detected!")
        move(0,0)

        #Circumvent Obstacle
        rospy.loginfo("Circumvent Mode!")
        incircumventmode=True
        startdistanz=calc_distancebetweenpoints(get_selfpos(),targetpoint)
        while incircumventmode and calc_distancebetweenpoints(get_selfpos(),targetpoint)>0.4:
            if(frontisfree(0.5) and leftisfree(0.8)):
                #turn left - wrong way! Turn around
                print("#1 Turn left Front:"+ str(frontisfree(0.5))+" Left: "+ str(leftisfree(0.8)))
                move(0.2,0.4)
            if(not frontisfree(0.5) and leftisfree(0.8)):
                #hard right - not parralell to wall!
                print("#2 Hard right! Front" + str(frontisfree(0.5))+" Left: "+ str(leftisfree(0.8)))
                move(0,-0.3)
            if(frontisfree(0.5) and not leftisfree(0.8)):
                #forward - im Following the wall!
                print("#3 Following the obstacle")
                move(0.7,0)
            if(not frontisfree(0.5) and not leftisfree(0.8)):
                #hard left - at a corner! left the wall behind us!
                print("#4 Hard right")
                move(0,-0.3)
            #check if nearer on Line to targetpoint
            incircumventmode=not(is_between(targetpoint,startpoint,get_selfpos()) and ((startdistanz-calc_distancebetweenpoints(get_selfpos(),targetpoint)) > 0.3))
        rospy.loginfo("Done Circumventing!")

        move(0,0)
        rospy.loginfo("Ziel erreicht!")


def main():

    global movex,movez,movexact,movezact
    movex=0
    movez=0
    movexact=0
    movezact =0

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
        rospy.loginfo("New Target x:"+str(mygoals.goals[0].x)+" y: "+str(mygoals.goals[0].y)+" z: "+str(mygoals.goals[0].z))
        move_totarget(mygoals.goals[0])
        mygoals.goals.pop(0)


    rospy.loginfo("Done with Main")
    return 1


rospy.init_node('TurtelLogic')

pub = rospy.Publisher('cmd_vel', Twist) # Create a Publisher object, that will publish the movement
r = rospy.Rate(4)

rospy.Subscriber('scan', LaserScan, callback_laser) #Subscriber to Laserscan
rospy.Subscriber('/gazebo/model_states', ModelStates, callback_modelstates) #Subscriber to ModelStates
rospy.Subscriber('/goals', PointArray, callback_goal) #Subscriber to ModelStates

main()

rospy.spin()
