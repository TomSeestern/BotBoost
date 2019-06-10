#!/usr/bin/env python

#Standart Libs
import rospy
import actionlib
import time
import math

#ros Libs
##Output
from geometry_msgs.msg import Twist         # Import the Twist message from the geometry_msgs to move the bot
##Input
from sensor_msgs.msg import LaserScan       # Import Laser Data Message
from gazebo_msgs.msg import ModelStates     # Import Message for Odom
from goal_publisher.msg import PointArray   # Import Goal Message


global scandata
global modelstatesdata
global goals

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

def main():

    global scandata
    scandata = LaserScan()
    global modelstatesdata
    modelstatesdata = ModelStates()
    global goals
    goals = PointArray()

    rospy.loginfo("Done with Main")
    return 1


rospy.init_node('TurtelLogic')

pub = rospy.Publisher('cmd_vel', Twist) # Create a Publisher object, that will publish the movement

rospy.Subscriber('scan', LaserScan, callback_laser) #Subscriber to Laserscan
rospy.Subscriber('/gazebo/model_states', ModelStates, callback_modelstates) #Subscriber to ModelStates
rospy.Subscriber('/goals', PointArray, callback_goal) #Subscriber to ModelStates

main()

rospy.spin()
