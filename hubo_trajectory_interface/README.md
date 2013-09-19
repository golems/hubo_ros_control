Testing sequence

In the root account of body :

    roslaunch hubo_trajectory_interface hubo_trajectory_action_hubo_motion.launch

In another shell in the hubo account of the body :

    roscd hubo_trajectory_interface/scripts
    ./hubo_read_trajectory.py -f 50 -n ../test_data/ach_final.traj
