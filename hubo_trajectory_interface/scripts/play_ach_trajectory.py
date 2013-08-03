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

hubo_body_joint_names = { 
0 : 'HPY' ,  
1 : 'RHY' ,  
2 : 'LHY' ,
3 : 'RHR' ,
4 : 'LHR' ,
5 : 'RHP' , 
6 : 'LHP' ,  
7 : 'RKP' ,        
8 : 'LKP' ,  
9 : 'RAP' ,      
10 : 'LAP' ,        
11 : 'RAR' ,
12 : 'LAR' ,
13 : 'RSP' ,
14 : 'LSP' ,   
15 : 'RSR' ,        
16 : 'LSR' ,
17 : 'RSY' ,
18 : 'LSY' ,      
19 : 'REP' , 
20 : 'LEP' ,
21 : 'RWY' ,
22 : 'LWY' ,
23 : 'RWP' ,    
24 : 'LWP' ,
25 : 'HNR' ,
26 : 'HNP' 
}

class TrajectoryPlayer():

    def __init__(self):
        
        self.hubo_traj = None
        self.dt = 0.05 # 20 Hz

        return

    def readfile(self,fname):

        # open the file and reads the array
        f = open(fname,'r')
        array = []
        for line in f:
            array.append([float(x) for x in line.split()])
        f.close()

        if( len(array) == 0 ):
            print "Warning : empty trajectory"
            return

        self.hubo_traj = JointTrajectory()
        self.hubo_traj.header.stamp = rospy.Time.now()

        t = 0.0

        for line in array: # read all lines in file

            #print line

            current_point = JointTrajectoryPoint()
            current_point.time_from_start = rospy.Duration(t)

            t += self.dt

            # Empty position buffers
            p_buffer = []

             # Fill in position buffers
            joint_id = 0
            for p in range( len(line) ):
                #if( p in joint_pos_offsets.keys() ):
                if( joint_id in hubo_body_joint_names.keys() ):
                    p_buffer.append(float(line[p]))

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

#        



#        dom = parseString(data) # Get the document object model



#        hubo_traj.points.append(current_point)
#        return hubo_traj
                        
#        except NameError, e:
#            print "Error - one of the necessary variables is not found:"
#            print e

#        # print "Finished playing trajectory file: " + fname
#        return 1

if __name__ == "__main__":
    # One can run this script from terminal passing a radius value in
    # Otherwise use the default value
    r = None
    play = False
    taskwall = False
    wall = False
    demo = False

    if(len(sys.argv) >= 2):
        for index in range(1,len(sys.argv)):
            if(sys.argv[index] == "-radius" and index+1<len(sys.argv)):
                r = float(sys.argv[index+1])
            elif(sys.argv[index] == "-play"):
                play = True
            elif(sys.argv[index] == "-taskwall"):
                taskwall = True
            elif(sys.argv[index] == "-wall"):
                wall = True
            elif(sys.argv[index] == "-demo"):
                demo = True

    rospy.init_node("hubo_read_trajectory")

    play = TrajectoryPlayer()
    play.readfile("../test_data/ach_final.traj")
    print "done!"  

