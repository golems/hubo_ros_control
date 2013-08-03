#!/usr/bin/env python
# Jim Mainprice WPI

import rospy;
import roslib;
roslib.load_manifest('hubo_trajectory_interface')

from math import *
import random
import time

from std_msgs.msg import String
from hubo_robot_msgs.msg import *
from trajectory_msgs.msg import JointTrajectoryPoint

from copy import deepcopy
import sys

class TrajectoryPlayer():

    def __init__(self):
        
        self.hubo_traj = None
        self.dt = 0.05 # 20 Hz

        # ach trajectory mapping, this mapping differs from the internal ros mapping
        # defined as a global parameter (joints) the parameter server   
        self.hubo_ach_traj_joint_names = {       0 : 'RHY' ,  1 : 'RHR' ,  2 : 'RHP' ,  3 : 'RKN' ,  4 :  'RAP' ,  
                                                 5 : 'RAR' ,  6 : 'LHY' ,  7 : 'LHR' ,  8 : 'LHP' ,  9 : 'LKN' , 
                                                10 : 'LAP' , 11 : 'LAR' , 12 : 'RSP' , 13 : 'RSR' , 14 : 'RSY' , 
                                                15 : 'REB' , 16 : 'RWY' , 17 : 'RWR' , 18 : 'RWP' , 19 : 'LSP' , 
                                                20 : 'LSR' , 21 : 'LSY' , 22 : 'LEB' , 23 : 'LWY' , 24 : 'LWR' , 
                                                25 : 'LWP' , 26 : 'NKY' , 27 : 'NK1' , 28 : 'NK2' , 29 : 'WST' ,
                                                30 : 'RF1' , 31 : 'RF2' , 32 : 'RF3' , 33 : 'RF4' , 34 : 'RF5' ,  
                                                35 : 'LF1' , 36 : 'LF2' , 37 : 'LF3' , 38 : 'LF4' , 39 : 'LF5' }

        self.joint_mapping = None

        return
    

    def readfile(self,fname):

        print "parsing file"

        # open the file and reads the array
        f = open(fname,'r')
        array = []
        for line in f:
            array.append([float(x) for x in line.split()])
        f.close()

        if( len(array) == 0 ):
            print "Warning : empty trajectory"
            return

        print "filing message"

        self.hubo_traj = JointTrajectory()
        self.hubo_traj.header.stamp = rospy.Time.now()

        t = 0.0

        for line in array: # read all lines in file

            current_point = JointTrajectoryPoint()
            current_point.time_from_start = rospy.Duration(t)

            t += self.dt

            # Empty position buffers
            p_buffer = []

             # Fill in position buffers
            for p in range( len(line) ):

                try:
                    i = self.joint_mapping[ self.hubo_ach_traj_joint_names[p] ]
                except KeyError:
                    i = None
                if i is not None:
                    p_buffer.append(float(line[i]))

            # Empty velocity buffers
            v_buffer = []

            # Fill in velocity buffers
            v_buffer.append( (p_buffer[0]-p_buffer[1])/self.dt )
            for i in range( 1 , len(p_buffer)-1 ):
                v_buffer.append( (p_buffer[i+1]-p_buffer[i-1])/self.dt )
            v_buffer.append( (p_buffer[len(p_buffer)-1]-p_buffer[len(p_buffer)-2])/self.dt )

            # Empty accelerations buffers
            a_buffer = []

            # Fill in accelerations buffers
            a_factor = 10;
            a_buffer.append( a_factor*(v_buffer[0]-v_buffer[1])/self.dt )
            for i in range( 1 , len(v_buffer)-1 ):
                a_buffer.append( a_factor*(v_buffer[i+1]-v_buffer[i-1])/self.dt )
            a_buffer.append( a_factor*(v_buffer[len(v_buffer)-1]-v_buffer[len(v_buffer)-2])/self.dt )

            # Append trajectory point
            current_point.positions = deepcopy(p_buffer)
            current_point.velocities = deepcopy(v_buffer)
            current_point.accelerations = deepcopy(a_buffer)

            self.hubo_traj.points.append(current_point)
        
        return

if __name__ == "__main__":
    # This script presents the same interface as the huo-read-trajectory program
    # one can run this script from terminal passing the frequency
    # and the compliance mode
    filename = None
    compliance = False
    frequency = False
    play = False

    accepted_freq = [100, 50, 25, 10, 200, 500]

    if(len(sys.argv) >= 2):
        for index in range(1,len(sys.argv)):
            if(sys.argv[index] == "-radius" and index+1<len(sys.argv)):
                r = float(sys.argv[index+1])
            elif(sys.argv[index] == "-f"):
                frequency = float(sys.argv[index+1])
                if( frequency in accepted_freq ):
                    play = True
                else:
                    print "frequency is not in accepted"
                    print accepted_freq
                    play = True
            elif(sys.argv[index] == "-n"):
                taskwall = True
            elif(sys.argv[index] == "-wall"):
                wall = True
            elif(sys.argv[index] == "-demo"):
                demo = True

    rospy.init_node( "hubo_read_trajectory" )

    # Hard coded namespace
    ns = "/drchubo_fullbody_controller/drchubo_fullbody_controller_node/"

    joint_names = rospy.get_param( ns + "joints")

    joint_mapping = {}

    for i in range(0,len(joint_names)):
        joint_names[i] = joint_names[i].strip( '/' )
        joint_mapping[ joint_names[i] ] = int(i)
        
    print joint_mapping

#    joint_mapping = {}
#    for i in range(0,len(joint_names)):
#        ns = str("/drchubo_fullbody_controller/drchubo_fullbody_controller_node/mapping/") + joint_names[i]
#        h = rospy.get_param( ns + "/huboachid" )
#        print str(h) + " " + joint_names[i]
#        joint_mapping[ joint_names[i] ] = h

    play = TrajectoryPlayer()
    play.joint_mapping = joint_mapping
    play.readfile("../test_data/ach_final.traj")
    print "done!"

