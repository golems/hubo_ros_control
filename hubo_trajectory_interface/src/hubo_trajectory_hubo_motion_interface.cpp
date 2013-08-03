/*
 * Copyright (c) 2013, Drexel DARPA Robotics Challenge team
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/* Author: Jim Mainprice (WPI) */

#include <stdlib.h>
#include <vector>
// System includes to handle safe shutdown
#include <signal.h>
// ROS & includes
#include <ros/ros.h>
// Boost includes
#include <boost/thread.hpp>
// Message and action includes for Hubo actions
#include <hubo_robot_msgs/JointTrajectory.h>
#include <hubo_robot_msgs/JointTrajectoryState.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <rosgraph_msgs/Clock.h>
// Includes for ACH and hubo-motion-rt
#include <hubo_components/hubo_components/Hubo_Control.h>
// Trajectory structure
#include <hubo_trajectory_interface/hubo_trajectory.hpp>
// Boost includes
#include <boost/thread.hpp>

#include <iostream>
#include <sys/time.h>
#include <unistd.h>

// Thread for listening to the hubo state and republishing it
boost::thread* pub_thread;
boost::thread* traj_thread;

using std::cout;
using std::endl;

/*
 * These are the two values we don't quite know how to set - we actually want these
 * to be "slow" - i.e. small trajectory chunks at a comparatively slow rate, since
 * it allows for a useful "cancelling" of the trajectory to be done by simply not
 * sending any more chunks without needing explicit support for this in hubo motion.
 *
 * These values may need to be experimentally determined, and the SPIN_RATE parameter
 * may need to dynamically change based on the timings on the incoming trajectory.
*/

#define MAX_TRAJ_LENGTH 10 //Number of points in each trajectory chunk
static double SPIN_RATE = 40.0; //Rate in hertz at which to send trajectory chunks

//#define ON true
//#define OFF false

// Joint name and mapping storage
static std::vector<std::string> g_joint_names;
static std::map<std::string,int> g_joint_mapping;

// Trajectory storage
static hubo_robot_msgs::JointTrajectory g_ros_trajectory;
static int g_tid = 0;

// Publisher and subscriber
static ros::Publisher g_state_pub;
static ros::Publisher g_clock_pub;
static ros::Subscriber g_traj_sub;

static std::string filename = "test_data/valve_turning.traj";
static std::vector<int> g_all_joints;
static std::vector<double> error;

static Hubo_Control* hubo=NULL;
static Hubo::Trajectory g_hubo_traj;

// Execution state
static bool g_running = false;
static bool g_next_chunk_sent = false;
static bool g_wait_for_new_state = false;

/*
 * Given a string name of a joint, it looks it up in the list of joint names
 * to determine the joint index used in hubo ach for that joint. If the name
 * can't be found, it returns -1.
*/
int index_lookup(std::string joint_name)
{
    for ( int i=0; i<g_joint_names.size(); i++ )
    {
        if( g_joint_names[i] == joint_name )
        {
            return g_joint_mapping[g_joint_names[i]];
        }
    }
    return -1;
}

/*
 * Callback when the ROS trajectory is received chunks, chunks are transmitted over hubo-ach
 */
void trajectory_cb( const hubo_robot_msgs::JointTrajectory& traj )
{
    //cout << "trajectoryCB : " << traj << endl;
    // Callback to chunk and save incoming trajectories
    // Before we do anything, check if the trajectory is empty - this is a special "stop" value that flushes the current stored trajectory
    if (traj.points.size() == 0 )
    {
        ROS_INFO("Flushing current trajectory");
        return;
    }
    else if (traj.points.size() == 0)
    {
        ROS_WARN("Execution cancelled, NOT ABORTING DUE TO DEBUG MODE");
        return;
    }
    ROS_INFO("Reprocessing trajectory with %ld elements into chunks", traj.points.size());

    g_ros_trajectory = traj;
    g_ros_trajectory.points.clear();
    g_hubo_traj.clear();

    ros::Duration base_time(0.0);

    for ( int i=0; i< traj.points.size();i++ )
    {
        // Make sure the JointTrajectoryPoint gets retimed to match its new trajectory chunk
        trajectory_msgs::JointTrajectoryPoint cur_point = traj.points[i];

        // Retime based on the receiving times
        cur_point.time_from_start = cur_point.time_from_start - base_time;

        // Make sure position, velocity, and acceleration are all the same length
        int size = cur_point.positions.size();
        cur_point.velocities.resize(size);
        cur_point.accelerations.resize(size);

        // Store the current point in a ros trajectory
        g_ros_trajectory.points.push_back( cur_point );

        // Store the point as in linear peicewise trajectory
        Hubo::Milestone q;
        q.first = cur_point.time_from_start.toSec();
        q.second = cur_point.positions;
        g_hubo_traj.push_back( q );
    }

    ROS_INFO("Received a new trajectory with %ld elements", g_ros_trajectory.points.size());
}

/*
 * Gets the current time from hubo-ach
 */
double get_time()
{
    hubo->update(true);
    double time = hubo->getTime();
    cout << "get time : " << time << endl;
    return time;
    //    timeval tim;
    //    gettimeofday(&tim, NULL);
    //    double tu=tim.tv_sec+(tim.tv_usec/1000000.0);
    //    return tu;
}

/*
 * Gets the elapsed time to refernce
 */
double time_from_ref( const double& t_ref )
{
    return get_time() - t_ref;
}

/*
 * Loads the trajectory from a file
 * using the hubo-ach format that simply defines
 * a series of waypoints
 */
bool load_trajectory_from_file()
{
    //    hubo.setArmCompliance( LEFT, true );
    //    hubo.setArmCompliance( RIGHT, true );

    g_hubo_traj.load_from_file( filename, 25 ); // Play at 25 hertz
}

void set_arms_complience_on()
{
    ArmVector Kp, Kd;

    Kp << -1, -1, -1, -1, -1, -1, -1, -1, -1, -1;
    Kd << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

    hubo->setArmCompliance( RIGHT, ON, Kp, Kd );
    hubo->setArmCompliance( LEFT,  ON, Kp, Kd );
}

void set_arms_complience_off()
{
    hubo->setArmCompliance( RIGHT, OFF );
    hubo->setArmCompliance( LEFT,  OFF );
}

void set_nominal_vel_and_acc()
{
    ArmVector rArmSpeedDef, rLegSpeedDef, rArmAccDef, rLegAccDef;
    LegVector lArmSpeedDef, lLegSpeedDef, lArmAccDef, lLegAccDef;

    hubo->getArmNomSpeeds( RIGHT, rArmSpeedDef );
    hubo->getArmNomSpeeds( LEFT,  lArmSpeedDef );
    hubo->getLegNomSpeeds( RIGHT, rLegSpeedDef );
    hubo->getLegNomSpeeds( LEFT,   lLegSpeedDef );

    hubo->getArmNomAcc( RIGHT, rArmAccDef );
    hubo->getArmNomAcc( LEFT,  lArmAccDef );
    hubo->getLegNomAcc( RIGHT, rLegAccDef );
    hubo->getLegNomAcc( LEFT,  lLegAccDef );
}

/*
 * Executes a peicewise linear trajectory
 * way points are linearly interpolated, sends values at 200Hz through ach
 */
bool execute_linear_trajectory( const Hubo::Trajectory& traj, const std::vector<int>& active_joints )
{
    if( hubo == NULL )
    {
        ROS_WARN("hubo object not initilized");
        return 0;
    }

    ROS_INFO("Load trajectory ---------------------- ");
    ROS_INFO("  time length : %f", traj.get_length());
    ROS_INFO("  time nb of milestones : %d", traj.get_number_of_milestones());

    //    if(! conv.mapTrajectory( conv.mMaps.rs_map, conv.mMaps.hubo_map, traj ) )
    //    {
    //        return 0;
    //    }

    set_nominal_vel_and_acc();

    hubo->update(true);

    double dt = 0.005; // 200Hz
    double t_start = get_time();
    double t_length = traj.get_length();
    double t = time_from_ref( t_start );

    for(int i=0; i<int(active_joints.size()); i++)
    {
        int jnt = active_joints[i];
        hubo->setJointTrajCorrectness( jnt, 0.05 );
    }

    while( t <= t_length )
    {
        t = time_from_ref( t_start );

        hubo->update(true); // not necessary after getting time

        Hubo::Vector q_t0 = traj.get_config_at_time( t );
        Hubo::Vector q_t1 = traj.get_config_at_time( t + dt );

        for(int i=0; i<int(active_joints.size()); i++)
        {
            int jnt = active_joints[i];

            hubo->setJointTraj( jnt, q_t0[jnt], (q_t0[jnt]-q_t1[jnt])/dt );
            //hubo->passJointAngle( jnt, q_t0[jnt] );

            error[jnt] = q_t0[jnt] - hubo->getJointAngleState( jnt );
        }

        hubo->sendControls();

        while ( time_from_ref(t) < dt )
        {
            // usleep(100); // sleeps for 100  micro seconds
        }
    }

    set_nominal_vel_and_acc();
    g_running = false;
    return true;
}

/*
 * Main function that spins
 */
int main(int argc, char** argv)
{
    std::cout << "Starting JointTrajectoryAction controller interface node..." << std::endl;
    ros::init(argc, argv, "hubo_joint_trajectory_controller_interface_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    ROS_INFO("Attempting to start JointTrajectoryAction controller interface...");

    // Get all the active joint names
    XmlRpc::XmlRpcValue joint_names;
    if (!nhp.getParam("joints", joint_names))
    {
        ROS_FATAL("No joints given. (namespace: %s)", nhp.getNamespace().c_str());
        exit(1);
    }
    if (joint_names.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        ROS_FATAL("Malformed joint specification.  (namespace: %s)", nhp.getNamespace().c_str());
        exit(1);
    }
    for ( int i=0; i<int(joint_names.size()); i++ )
    {
        XmlRpc::XmlRpcValue &name_value = joint_names[i];
        if (name_value.getType() != XmlRpc::XmlRpcValue::TypeString)
        {
            ROS_FATAL("Array of joint names should contain all strings.  (namespace: %s)", nhp.getNamespace().c_str());
            exit(1);
        }
        g_joint_names.push_back((std::string)name_value);
    }

    g_all_joints.clear();

    // Gets the hubo ach index for each joint
    for ( int i=0; i<int(g_joint_names.size()); i++ )
    {
        std::string ns = std::string("mapping/") + g_joint_names[i];
        int h;
        nhp.param( ns + "/huboachid", h, -1);
        g_joint_mapping[g_joint_names[i]] = h;
        g_all_joints.push_back( h );
    }

    // Set up Hubo Control daemon (hubo-motion-rt)
    hubo = new Hubo_Control(false);

    // Set up state publisher
    std::string pub_path = nh.getNamespace() + "/state";
    g_state_pub = nh.advertise<hubo_robot_msgs::JointTrajectoryState>(pub_path, 1);

    // Set up clock publisher
    g_clock_pub = nh.advertise<rosgraph_msgs::Clock>("/clock", 1);

    // Set up the trajectory subscriber
    std::string sub_path = nh.getNamespace() + "/command";
    g_traj_sub = nh.subscribe( sub_path, 1, trajectory_cb );
    ROS_INFO("Loaded trajectory interface to hubo-motion-rt");

    g_running = false;

    // Spin until killed
    while (ros::ok())
    {
        if( !g_running && !g_hubo_traj.empty() )
        {
            g_running = true;
            // Spin up the thread for getting the trajectory execution status
            traj_thread = new boost::thread( &execute_linear_trajectory, g_hubo_traj, g_all_joints );
        }

        // Wait long enough before sending the next one and receiving the running signal
        ros::spinOnce();
        ros::Rate looprate(SPIN_RATE);
        looprate.sleep();
    }
    return 0;
}
