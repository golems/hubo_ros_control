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
#include <hubo_robot_msgs/ComplianceConfig.h>
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
 * We should add cancelation of the trajectory execution at this point
 *
 * These values may need to be experimentally determined, and the SPIN_RATE parameter
 * may need to dynamically change based on the timings on the incoming trajectory.
*/

#define MAX_TRAJ_LENGTH 10 //Number of points in each trajectory chunk
static double SPIN_RATE = 40.0; // Rate in hertz at which to send trajectory chunks

//#define ON true
//#define OFF false

class HuboMotionRtController
{
public:
    HuboMotionRtController( ros::NodeHandle &n ) : node_(n), nhp_("~")
    {
        g_tid = 0;
        running_;
        next_chunk_sent_;
        wait_for_new_state_;
        filename_ = "test_data/valve_turning.traj";
        hubo_=NULL;

        ROS_INFO( "Attempting to start JointTrajectoryAction controller interface..." );

        // Get all the active joint names
        XmlRpc::XmlRpcValue joint_names;
        if (!nhp_.getParam("joints", joint_names))
        {
            ROS_FATAL("No joints given. (namespace: %s)", nhp_.getNamespace().c_str());
            exit(1);
        }
        if (joint_names.getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
            ROS_FATAL("Malformed joint specification.  (namespace: %s)", nhp_.getNamespace().c_str());
            exit(1);
        }
        for ( int i=0; i<int(joint_names.size()); i++ )
        {
            XmlRpc::XmlRpcValue &name_value = joint_names[i];
            if (name_value.getType() != XmlRpc::XmlRpcValue::TypeString)
            {
                ROS_FATAL("Array of joint names should contain all strings.  (namespace: %s)", nhp_.getNamespace().c_str());
                exit(1);
            }
            joint_names_.push_back( std::string(name_value) );
        }

        all_joints_.clear();

        // Gets the hubo ach index for each joint
        for ( int i=0; i<int(joint_names_.size()); i++ )
        {
            std::string ns = std::string("mapping/") + joint_names_[i];
            int h;
            nhp_.param( ns + "/huboachid", h, -1);
            joint_mapping_[joint_names_[i]] = h;
            all_joints_.push_back( h );
            ROS_INFO( "joint_names_[%d] : %s\n", i, joint_names_[i].c_str() );
        }
        active_joints_ = all_joints_;

        // Set up Hubo Control daemon (hubo-motion-rt)
        hubo_ = new Hubo_Control(false);

        // Set up state publisher
        std::string pub_path = node_.getNamespace() + "/state";
        state_pub_ = node_.advertise<hubo_robot_msgs::JointTrajectoryState>(pub_path, 1);

        // Set up clock publisher
        clock_pub_ = node_.advertise<rosgraph_msgs::Clock>("/clock", 1);

        // Set up the trajectory subscriber
        std::string sub_path = node_.getNamespace() + "/command";
        traj_sub_ = node_.subscribe( sub_path, 1, &HuboMotionRtController::trajectory_cb, this );
        ROS_INFO("Loaded trajectory interface to hubo-motion-rt");

        running_ = false;

        // Spin up the thread for getting data from hubo and publishing it
        pub_thread = new boost::thread( &HuboMotionRtController::publish_loop, this );

        // Spin until killed
        while (ros::ok())
        {
            //ROS_INFO("loop in robot-motion-rt controller");
            if( !running_ && !hubo_traj_.empty() )
            {
                running_ = true;
                // starts the thread for trajectory execution
                traj_thread = new boost::thread( &HuboMotionRtController::execute_linear_trajectory, this );
            }

            // Wait long enough before sending the next one and receiving the running signal
            ros::spinOnce();
            ros::Rate looprate(SPIN_RATE);
            looprate.sleep();
        }
    }

    ~HuboMotionRtController()
    {
        //        pub_interface_command_.shutdown();
        //        sub_interface_state_.shutdown();
        //        watchdog_timer_.stop();
    }

    void shut_down()
    {
        hubo_traj_.clear();
        ROS_INFO("Starting safe shutdown...");

        //pub_thread->join();
        ROS_INFO("All threads done");

        ach_close(&chan_hubo_ctrl_state_pub_);
        ROS_INFO("ach channels closed shutting down!");

        ros::shutdown();
    }

private:
    ros::NodeHandle node_;
    ros::NodeHandle nhp_;

    // Joint name and mapping storage
    std::vector<std::string> joint_names_;
    std::map<std::string,int> joint_mapping_;

    // Compliance
    std::vector<std::string> compliant_joint_names_;
    std::vector<double> compliance_kp_;
    std::vector<double> compliance_kd_;

    // Trajectory storage
    int g_tid;

    // Publisher and subscriber
    ros::Publisher state_pub_;
    ros::Publisher clock_pub_;
    ros::Subscriber traj_sub_;

    std::string filename_;
    std::vector<int> all_joints_;
    std::vector<int> active_joints_;
    std::vector<double> error_;

    Hubo_Control* hubo_;
    Hubo::Trajectory hubo_traj_;
    ach_channel_t chan_hubo_ctrl_state_pub_;

    // Execution state
    bool running_;
    bool next_chunk_sent_;
    bool wait_for_new_state_;

    //! This runs in a second thread in the background.
    //! The thread grabs the latest states from the hubo's joints and joint setpoints
    //! and repacks them into the ROS messages sent to the trajectory action server.
    void publish_loop()
    {
        ROS_INFO("publishLoop");

        ach_status_t r;
        r = ach_open( &chan_hubo_ctrl_state_pub_,  CTRL_CHAN_STATE, NULL );
        if (r != ACH_OK)
        {
            ROS_FATAL("Could not open ACH channel: CTRL_CHAN_STATE !");
            exit(1);
        }

        hubo_ctrl_state_t H_ctrl_state;
        memset(&H_ctrl_state, 0, sizeof(H_ctrl_state));

        // Loop until node shutdown
        while (ros::ok())
        {
            size_t fs;

            // Get latest state from HUBO-MOTION (this is used to populate the desired values)
            ach_status_t r = ach_get( &chan_hubo_ctrl_state_pub_,  &H_ctrl_state, sizeof(H_ctrl_state), &fs, NULL, ACH_O_LAST );

            if( r != ACH_OK && r != ACH_MISSED_FRAME && r != ACH_STALE_FRAMES )
            {
                ROS_ERROR("get ach channel for H_ctrl_state failed in [publishing loop] : %s" , ach_result_to_string(r) );
                //continue;
            }
            else if (fs != sizeof(H_ctrl_state))
            {
                //ROS_ERROR("Hubo ref size error! [publishing loop] with ach channel %s" , ach_result_to_string(r));
                continue;
            }

            // Publish the latest hubo state back out
            hubo_robot_msgs::JointTrajectoryState cur_state;
            cur_state.header.stamp = ros::Time::now();
            // Set the names
            cur_state.joint_names = joint_names_;
            size_t num_joints = cur_state.joint_names.size();
            // Make the empty states
            trajectory_msgs::JointTrajectoryPoint cur_setpoint;
            trajectory_msgs::JointTrajectoryPoint cur_actual;
            trajectory_msgs::JointTrajectoryPoint cur_error;
            // Resize the states
            cur_setpoint.positions.resize(num_joints);
            cur_setpoint.velocities.resize(num_joints);
            cur_setpoint.accelerations.resize(num_joints);
            cur_actual.positions.resize(num_joints);
            cur_actual.velocities.resize(num_joints);
            cur_actual.accelerations.resize(num_joints);
            cur_error.positions.resize(num_joints);
            cur_error.velocities.resize(num_joints);
            cur_error.accelerations.resize(num_joints);
            // Fill in the setpoint and actual & calc the error_ in the process
            for (size_t i=0; i<num_joints; i++)
            {
                // Fill in the setpoint and actual data
                // Values that we don't have data for are set to NAN
                int hubo_index = index_lookup( cur_state.joint_names[i] );
                if (hubo_index >= 0)
                {
                    cur_setpoint.positions[i]       = H_ctrl_state.requested_pos[hubo_index];
                    cur_setpoint.velocities[i]      = H_ctrl_state.requested_vel[hubo_index];
                    cur_setpoint.accelerations[i]   = H_ctrl_state.requested_acc[hubo_index];

                    cur_actual.positions[i]         = H_ctrl_state.actual_pos[hubo_index];
                    cur_actual.velocities[i]        = H_ctrl_state.actual_vel[hubo_index];
                    cur_actual.accelerations[i]     = H_ctrl_state.actual_acc[hubo_index];
                    // Calc the error
                    cur_error.positions[i] =        cur_setpoint.positions[i] -     cur_actual.positions[i];
                    cur_error.velocities[i] =       cur_setpoint.velocities[i] -    cur_actual.velocities[i];
                    cur_error.accelerations[i] =    cur_setpoint.accelerations[i] - cur_actual.accelerations[i];
                }
            }

            // Publish State (pack them together)
            cur_state.desired = cur_setpoint;
            cur_state.actual = cur_actual;
            cur_state.error = cur_error;
            state_pub_.publish( cur_state );

            // Publish Time
            hubo_->update(true);
            rosgraph_msgs::Clock clockmsg;
            clockmsg.clock = ros::Time( hubo_->getTime());
            clock_pub_.publish( clockmsg );
            //ROS_INFO("TIME is : %f sec", H_state.time );
        }
    }

    //! Given a string name of a joint, it looks it up in the list of joint names
    //! to determine the joint index used in hubo ach for that joint. If the name
    //! can't be found, it returns -1.
    int index_lookup( const std::string& joint_name )
    {
        for ( int i=0; i<joint_names_.size(); i++ )
        {
            if( joint_names_[i] == joint_name )
            {
                return joint_mapping_[joint_names_[i]];
            }
        }
        return -1;
    }

    //! Callback when the ROS trajectory is received chunks, chunks are transmitted over hubo-ach
    void trajectory_cb( const hubo_robot_msgs::JointTrajectory& ros_traj )
    {
        ROS_INFO("Trajectory received");

        // Callback to chunk and save incoming trajectories
        // Before we do anything, check if the trajectory is empty - this is a special "stop" value that flushes the current stored trajectory
        if (ros_traj.points.size() == 0 )
        {
            ROS_INFO("Flushing current trajectory");
            return;
        }
        else if (ros_traj.points.size() == 0)
        {
            ROS_WARN("Execution cancelled, NOT ABORTING DUE TO DEBUG MODE");
            return;
        }
        ROS_INFO("Reprocessing trajectory with %ld elements into chunks", ros_traj.points.size());

        // Set trajectory
        hubo_traj_.clear();

        // Set compliance
        compliant_joint_names_ = ros_traj.compliance.joint_names;
        compliance_kp_ = ros_traj.compliance.compliance_kp;
        compliance_kd_ = ros_traj.compliance.compliance_kd;

        ros::Duration base_time(0.0);

        // Get hubo current config
        hubo_->update(true);
        Hubo::Milestone q_cur;
        q_cur.second.resize( HUBO_JOINT_COUNT );
        for ( size_t i=0; i<q_cur.second.size(); i++)
            q_cur.second[i] = hubo_->getJointAngleState( i );

        // Process all points
        for ( size_t i=0; i<ros_traj.points.size();i++ )
        {
            // Make sure the JointTrajectoryPoint gets retimed to match its new trajectory chunk
            trajectory_msgs::JointTrajectoryPoint cur_point = ros_traj.points[i];

            // Retime based on the receiving times
            cur_point.time_from_start = cur_point.time_from_start - base_time;

            // Make sure position, velocity, and acceleration are all the same length
            int size = cur_point.positions.size();
            cur_point.velocities.resize(size);
            cur_point.accelerations.resize(size);

            // Store the point as in linear peicewise trajectory
            Hubo::Milestone q;
            q.first = cur_point.time_from_start.toSec();
            q.second = q_cur.second;

            // Now, overwrite with the commands in the current trajectory
            for ( size_t j=0; j<cur_point.positions.size(); j++)
            {
                int index = index_lookup( ros_traj.joint_names[j] );
                if (index != -1)
                {
                    q.second[index] = cur_point.positions[j];
                    //processed.velocities[index] = cur_point.velocities[i];
                    //processed.accelerations[index] = cur_point.accelerations[i];
                }
            }

            Hubo::print_vector( q.second );

            hubo_traj_.push_back( q );
        }

        ROS_INFO("Received a new trajectory with %ld elements", ros_traj.points.size());
    }

    //! Gets the current time from hubo-ach
    double get_time()
    {
        hubo_->update(true);
        double time = hubo_->getTime();
        //cout << "get time : " << time << endl;
        return time;
        //    timeval tim;
        //    gettimeofday(&tim, NULL);
        //    double tu=tim.tv_sec+(tim.tv_usec/1000000.0);
        //    return tu;
    }

    //! Gets the elapsed time to refernce
    double time_from_ref( const double& t_ref )
    {
        return get_time() - t_ref;
    }

    //! Loads the trajectory from a file
    //! using the hubo-ach format that simply defines
    //! a series of waypoints
    bool load_trajectory_from_file()
    {
        hubo_traj_.load_from_file( filename_, 25 ); // Play at 25 hertz
    }

    void set_arms_complience_on()
    {
        for( size_t i=0;compliant_joint_names_.size(); i++ )
        {
            int index = index_lookup( compliant_joint_names_[i] );
            if (index != -1)
            {
                hubo_->setJointCompliance( index, ON, compliance_kp_[i], compliance_kd_[i] );
            }
        }
    }

    void set_arms_complience_off()
    {
        hubo_->setArmCompliance( RIGHT, OFF );
        hubo_->setArmCompliance( LEFT,  OFF );
    }

    void set_nominal_vel_and_acc()
    {
        ArmVector rArmSpeedDef, rLegSpeedDef, rArmAccDef, rLegAccDef;
        LegVector lArmSpeedDef, lLegSpeedDef, lArmAccDef, lLegAccDef;

        hubo_->getArmNomSpeeds( RIGHT, rArmSpeedDef );
        hubo_->getArmNomSpeeds( LEFT,  lArmSpeedDef );
        hubo_->getLegNomSpeeds( RIGHT, rLegSpeedDef );
        hubo_->getLegNomSpeeds( LEFT,   lLegSpeedDef );

        hubo_->getArmNomAcc( RIGHT, rArmAccDef );
        hubo_->getArmNomAcc( LEFT,  lArmAccDef );
        hubo_->getLegNomAcc( RIGHT, rLegAccDef );
        hubo_->getLegNomAcc( LEFT,  lLegAccDef );
    }

    //! Executes a peicewise linear trajectory
    //! way points are linearly interpolated, sends values at 200Hz through ach
    bool execute_linear_trajectory()
    {
        if( hubo_ == NULL )
        {
            ROS_WARN("hubo object not initilized");
            return 0;
        }

        ROS_INFO("Load trajectory ---------------------- ");
        ROS_INFO("  time length : %f", hubo_traj_.get_length());
        ROS_INFO("  time nb of milestones : %d", hubo_traj_.get_number_of_milestones());

        //    if(! conv.mapTrajectory( conv.mMaps.rs_map, conv.mMaps.hubo_map, traj ) )
        //    {
        //        return 0;
        //    }

        set_nominal_vel_and_acc();

        const double dt_ = 0.005; // 200Hz
        double t_start = get_time();
        double t_length = hubo_traj_.get_length();
        double t = time_from_ref( t_start );
        double t_end;

        for(int i=0; i<int(active_joints_.size()); i++)
        {
            int jnt = active_joints_[i];
            hubo_->setJointTrajCorrectness( jnt, 0.05 );
        }

        while( t <= t_length )
        {
            cout << "t : " << t << endl;
            Hubo::Vector q_t0 = hubo_traj_.get_config_at_time( t );
            Hubo::Vector q_t1 = hubo_traj_.get_config_at_time( t + dt_ );

            error_.resize( HUBO_JOINT_COUNT, 0.0 );

            //Hubo::print_vector( q_t0 );

            for(int i=0; i<int(active_joints_.size()); i++)
            {
                int jnt = active_joints_[i];

                //hubo_->setJointTraj( jnt, q_t0[jnt], (q_t1[jnt]-q_t0[jnt])/dt );
                hubo_->passJointAngle( jnt, q_t0[jnt] );

                error_[jnt] = q_t0[jnt] - hubo_->getJointAngleState( jnt );
            }

            hubo_->sendControls();

            do
            {
                t_end =  time_from_ref( t_start );
                cout << "get new time" << endl;
            }
            while( t_end - t < dt_ );

            t = t_end;
        }

        hubo_traj_.clear();
        set_nominal_vel_and_acc();
        running_ = false;
        return true;
    }
};

//! This needs to be global so the signal handler can use it
HuboMotionRtController* g_hmrc;

//! Signal handler to catch SIGINT (the shutdown command) and attempt to safely shutdown the trajectory interface
void shutdown(int signum)
{
    ROS_WARN("Attempting to shutdown node...");
    if (g_hmrc != NULL)
    {
        //g_hmrc->shutdown(signum);
    }
    else
    {
        ROS_WARN("HJTAS not yet loaded, aborting the load process and shutting down");
    }
    ros::shutdown();
}

//! Main function that spins the ros node
int main(int argc, char** argv)
{
    ROS_INFO("Starting HuboMotionRtContorler action server node...");
    ros::init( argc, argv, "hubo_joint_trajectory_hubo_motion_interface_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle node;
    // Register a signal handler to safely shutdown the node
    signal(SIGINT, shutdown);
    ROS_INFO("Attempting to start HuboMotionRtContorler action server...");
    g_hmrc = new HuboMotionRtController( node );
    return 0;
}
