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

def calc_distancebetweenpoints(p1,p2):
    return abs(math.hypot(p2.x - p1.x, p2.y - p1.y))

def calc_distogoal(goal):
    return calc_distancebetweenpoints(get_selfpos(),goal)

def get_selfpos():
    index=modelstatesdata.name.index("turtlebot3_burger")
    pos = modelstatesdata.pose[index]
    return pos.position

def main():

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
