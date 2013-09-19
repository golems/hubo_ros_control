Testing sequence

First make sure that the root account is set up for ROS (i.e. that the root bashrc includes the following lines at the end) :

    # ROS setting
    source /opt/ros/groovy/setup.bash
    export ROS_HOSTNAME=192.168.0.202
    export ROS_MASTER_URI=http://192.168.0.212:11311/
    source /home/hubo/ros_workspace/devel/setup.bash

In the root account of body :

    roslaunch hubo_trajectory_interface hubo_trajectory_action_hubo_motion.launch

In another shell in the hubo account of the body :

    roscd hubo_trajectory_interface/scripts
    ./hubo_read_trajectory.py -f 50 -n ../test_data/ach_final.traj
