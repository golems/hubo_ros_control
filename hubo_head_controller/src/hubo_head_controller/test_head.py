#!/usr/bin/python

#################################################
#                                               #
#   Calder Phillips-Grafflin - WPI/ARC Lab      #
#                                               #
#   Laser pointing controller for DRCHubo       #
#                                               #
#   This provides one interface:                #
#                                               #
#   1.  LaserScanAction                         #
#                                               #
#################################################

import rospy
import math
from std_msgs.msg import *
from sensor_msgs.msg import *
import actionlib
from geometry_msgs.msg import *
import hubo_robot_msgs.msg as hrms
import hubo_sensor_msgs.msg as hsms

def test_head(x, y, z):
    head_client = actionlib.SimpleActionClient("/drchubo_point_head", hrms.PointHeadAction)
    head_client.wait_for_server()
    head_goal = hrms.PointHeadGoal()
    head_goal.pointing_frame = "/trinocular_base_frame"
    head_goal.pointing_axis.x = 1.0
    head_goal.target.header.frame_id = "/Body_Torso"
    head_goal.target.point.x = x
    head_goal.target.point.y = y
    head_goal.target.point.z = z
    head_goal.min_duration = rospy.Duration(5.0)
    print "Sending PointHeadGoal"
    head_client.send_goal(head_goal)
    head_client.wait_for_result()
    print "Action finished"

def test_lidar():
    laser_client = actionlib.SimpleActionClient("/drchubo_laser_scan", hsms.LaserScanAction)
    laser_client.wait_for_server()
    lidar_goal = hsms.LaserScanGoal()
    lidar_goal.min_angle = -0.4
    lidar_goal.max_angle = 0.4
    lidar_goal.tilt_rate = 0.25
    print "Sending LaserScanGoal"
    laser_client.send_goal(lidar_goal)
    laser_client.wait_for_result()
    print "Action finished"

if __name__ == '__main__':
    rospy.init_node("head_tester")
    test_head(-1.0, 0.0, 1.0)
    exit()
    test_lidar()
    test_head(-1.0, 0.0, 0.0)
    test_lidar()
    test_head(-1.0, 1.0, 0.0)
    test_lidar()
    exit()
    for h in range(-1,2):
        x = float(h)
        for i in range(-5,6):
            y = float(i)
            for j in range(-1,2):
                z = float(j)
                print "Testing head to [X,Y,Z]: [" + str(x) + "," + str(y) + "," + str(z) + "]"
                test_head(x,y,z)
                test_lidar()
    print "Testing done"
