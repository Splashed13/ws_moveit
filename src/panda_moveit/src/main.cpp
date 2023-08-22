#include "../include/panda_moveit/PickAndPlace.hpp"

// ensure main is placed outside the namespace
int main(int argc, char **argv)
{
    ros::init(argc, argv, "pick_and_place");
    
    // Create a NodeHandle to interface with ROS system
    ros::NodeHandle nh("~");

    std::string arg1, arg2;
    
    // Check if the parameter exists and if not, set a default value
    if (!nh.getParam("scene", arg1)) {
        arg1 = "";  // Set default value
    }
    if (!nh.getParam("approach", arg2)) {
        arg2 = "";  // Set default value
    }

    PickandPlace pnp(arg1, arg2, nh);

    // ROS spinning must be running for the MoveGroupInterface to get information
    // about the robot's state. One way to do this is to start an AsyncSpinner beforehand.
    ros::AsyncSpinner spinner(3);
    spinner.start();

    // add a short sleep so the node can finish initializing
    ros::Duration(0.5).sleep();
    
    if (arg1 == "gazebo"){
        pnp.gazebo();
    }else if (arg1 == "testing"){
        pnp.testing();
    }else{
        pnp.rviz();
    }
    // Shutdown the node and join the thread back before exiting
    ros::shutdown();

    return 0;
}
