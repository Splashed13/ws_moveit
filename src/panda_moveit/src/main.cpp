#include "../include/panda_moveit/PickAndPlace.hpp"

// ensure main is placed outside the namespace
int main(int argc, char **argv)
{
    ros::init(argc, argv, "pick_and_place");

    std::string arg1, arg2;
    if (argc > 1) arg1 = argv[1];
    if (argc > 2) arg2 = argv[2];

    PickandPlace pnp(arg1, arg2);

    // ROS spinning must be running for the MoveGroupInterface to get information
    // about the robot's state. One way to do this is to start an AsyncSpinner beforehand.
    ros::AsyncSpinner spinner(5);
    spinner.start();

    // add a short sleep so the node can finish initializing
    ros::Duration(0.5).sleep();

    
    // Run pick and place operations
    pnp.run();

    // Shutdown the node and join the thread back before exiting
    ros::shutdown();

    return 0;
}






