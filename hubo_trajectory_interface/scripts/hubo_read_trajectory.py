#!/usr/bin/env python
# Jim Mainprice WPI, august 2013

import rospy;
import roslib;
roslib.load_manifest('hubo_trajectory_interface')

from math import *
import random
import time

# Brings in the SimpleActionClient
import actionlib

from std_msgs.msg import String
from hubo_robot_msgs.msg import *

from copy import deepcopy
import sys

# Provides loader from ach format and playing through actionLib
class TrajectoryReader():

    def __init__( self, robot_name, frequency, joint_names, joint_mapping ):
        
        self.robot_name = robot_name
        self.joint_names = joint_names
        self.joint_mapping = joint_mapping
        self.hubo_traj = None
        self.dt = 1 / float(frequency) # 20 Hz (0.05)

        print "self.dt : " + str( self.dt )

        # Ach trajectory mapping. It differs from the internal ros mapping
        # which is defined as a global parameter (joints) in the parameter server   
        self.hubo_ach_traj_joint_names = {  0 : 'RHY' ,  1 : 'RHR' ,  2 : 'RHP' ,  3 : 'RKP' ,  4 : 'RAP' ,  
                                            5 : 'RAR' ,  6 : 'LHY' ,  7 : 'LHR' ,  8 : 'LHP' ,  9 : 'LKP' , 
                                           10 : 'LAP' , 11 : 'LAR' , 12 : 'RSP' , 13 : 'RSR' , 14 : 'RSY' , 
                                           15 : 'REP' , 16 : 'RWY' , 17 : 'RWR' , 18 : 'RWP' , 19 : 'LSP' , 
                                           20 : 'LSR' , 21 : 'LSY' , 22 : 'LEP' , 23 : 'LWY' , 24 : 'LWR' , 
                                           25 : 'LWP' , 26 : 'NKY' , 27 : 'NK1' , 28 : 'NK2' , 29 : 'TSY' ,
                                           30 : 'RF1' , 31 : 'RF2' , 32 : 'RF3' , 33 : 'RF4' , 34 : 'RF5' ,  
                                           35 : 'LF1' , 36 : 'LF2' , 37 : 'LF3' , 38 : 'LF4' , 39 : 'LF5' }
        return
    
    # Loads trajectory from file and stores it in a ROS message type
    # returns False if the trajectory had not be loaded properly
    def loadfile(self,fname):

        print "parsing file"

        # open the file and reads the array
        f = open(fname,'r')
        array = []
        for line in f:
            array.append([float(x) for x in line.split()])
        f.close()

        if( len(array) == 0 ):
            print "Warning : empty trajectory"
            return False

        print "filing message"

        self.hubo_traj = JointTrajectory()
        self.hubo_traj.header.stamp = rospy.Time.now()
        self.hubo_traj.joint_names = self.joint_names
        self.hubo_traj.compliance.joint_names = []

        t = 0.0

        for line in array: # reads all lines in the file

            # Ane configuration per line
            current_point = JointTrajectoryPoint()
            current_point.time_from_start = rospy.Duration(t)

            # Advance in time by dt
            t += float( self.dt )

            # ---------------------------------------
            # Fills position buffer
            p_buffer = [0.0] * len(self.joint_mapping)

            for p in range( len(line) ):

                try:
                    i = self.joint_mapping[ self.hubo_ach_traj_joint_names[p] ]
                except KeyError:
                    i = None
                if i is not None:
                    p_buffer[i] = float(line[p])
                else:
                    continue
            #print len(p_buffer)

            # ---------------------------------------
            # Fills velocity buffer using finite deferencing
            v_buffer = []
            v_buffer.append( (p_buffer[0]-p_buffer[1])/self.dt )
            for i in range( 1 , len(p_buffer)-1 ):
                v_buffer.append( (p_buffer[i+1]-p_buffer[i-1])/self.dt )
            v_buffer.append( (p_buffer[len(p_buffer)-1]-p_buffer[len(p_buffer)-2])/self.dt )

            # ---------------------------------------
            # Fills acceleration buffer using finite deferencing
            a_buffer = []
            a_factor = 10;
            a_buffer.append( a_factor*(v_buffer[0]-v_buffer[1])/self.dt )
            for i in range( 1 , len(v_buffer)-1 ):
                a_buffer.append( a_factor*(v_buffer[i+1]-v_buffer[i-1])/self.dt )
            a_buffer.append( a_factor*(v_buffer[len(v_buffer)-1]-v_buffer[len(v_buffer)-2])/self.dt )

            # Appends trajectory point
            current_point.positions = deepcopy(p_buffer)
            current_point.velocities = deepcopy(v_buffer)
            current_point.accelerations = deepcopy(a_buffer)

            self.hubo_traj.points.append(current_point)
        
        return True

    # Set compliance
    def setcompliance(self):

        joint_names = [] 
            
        # Left arm
        joint_names.append('LSP')
        joint_names.append('LSR')
        joint_names.append('LSY')
        joint_names.append('LEP')
        joint_names.append('LWY')
        joint_names.append('LWP')

        # Right arm
        joint_names.append('RSP')
        joint_names.append('RSR')
        joint_names.append('RSY')
        joint_names.append('REP')
        joint_names.append('RWY')
        joint_names.append('RWP')

        # Set Kp gains
        compliance_kp = [0.0]*len(joint_names)

        compliance_kp[0]  = 80
        compliance_kp[1]  = 80
        compliance_kp[2]  = 70
        compliance_kp[3]  = 40
        compliance_kp[4]  = 50
        compliance_kp[5]  = 40

        compliance_kp[6]  = 80
        compliance_kp[7]  = 80
        compliance_kp[8]  = 70
        compliance_kp[9]  = 40
        compliance_kp[10] = 50
        compliance_kp[11] = 40

        # Set Kd gains
        compliance_kd = [0.0]*len(joint_names)

        self.hubo_traj.compliance.joint_names = joint_names
        self.hubo_traj.compliance.compliance_kp = compliance_kp
        self.hubo_traj.compliance.compliance_kd = compliance_kp

        return

    # Sends the trajectory to the actionLib
    # returns when execution is finished
    def execute(self):

        if( self.hubo_traj is None ):
            print "cannot execute empty trajectory"
            return

        # Creates a SimpleActionClient, passing the type of action to the constructor.
        client = actionlib.SimpleActionClient('/drchubo_fullbody_controller/joint_trajectory_action', hubo_robot_msgs.msg.JointTrajectoryAction )
        
        # Waits until the action server has started
        print "waiting for action server..."
        client.wait_for_server()
        
        # Sends trajectory as a goal
        print "client started, sending trajectory!"
        res = None
        #rospy.sleep(1.0)
        self.hubo_traj.header.stamp = rospy.Time.now()
        traj_goal = JointTrajectoryGoal()
        traj_goal.trajectory = self.hubo_traj
        traj_goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(1.0)
        client.send_goal( traj_goal )

        print "Wait for result!"
        client.wait_for_result()
        res = client.get_result()
        print res

        return

if __name__ == "__main__":
    # This script presents the same interface as hubo-read-trajectory (ach based)
    # one can run this script from terminal passing 
    # frequency and the compliance as arguments
    file_name = None
    compliance = False
    frequency = 25
    play = False

    accepted_freq = [100, 50, 25, 17, 10, 200, 500]

    if(len(sys.argv) >= 2):

        for index in range(1,len(sys.argv)):

            if(sys.argv[index] == "-n" and index+1<len(sys.argv)):
                file_name = sys.argv[index+1]
                play = True
                try:
                    with open(file_name): pass
                except IOError:
                    play = False
                    print "file :" + file_name + " does not exist"

            elif(sys.argv[index] == "-f" and index+1<len(sys.argv)):
                frequency = int(sys.argv[index+1])
                if( frequency in accepted_freq ):
                    print "frequency is " + str(frequency)
                    play = True
                else:
                    print "frequency is not in accepted"
                    print accepted_freq
                    play = False

            elif(sys.argv[index] == "-c"):
                compliance = True

    if not play:
        print "error in arguments!!!"

    else: # All arguments are fine, plays trajectory

        rospy.init_node( "hubo_read_trajectory" )

        # Hard-coded namespace (you can change the robot name here)
        robot_name = "drchubo"
        ns = "/" + robot_name + "_fullbody_controller/hubo_trajectory_action/"

        # Gets joint mapping from parameter server
        joint_mapping = {}
        joint_names = rospy.get_param( ns + "joints")
        for i in range(0,len(joint_names)):
            joint_names[i] = joint_names[i].strip( '/' )
            joint_mapping[ joint_names[i] ] = int(i)

        print joint_mapping

        # Loads and executes the trajectory
        reader = TrajectoryReader( robot_name, frequency, joint_names, joint_mapping )

        if reader.loadfile( file_name ):
            if compliance:
                reader.setcompliance()
            #print reader.hubo_traj.compliance
            #print reader.hubo_traj.joint_names
            print "duration : " + str(reader.hubo_traj.points[-1].time_from_start.to_sec())
            reader.execute()
            print "done!"
        else:
            print "Could not load trajectory"

