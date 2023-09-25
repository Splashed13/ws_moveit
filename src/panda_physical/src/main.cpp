#include "../include/Manipulate.hpp"

int main(int argc, char** argv){
    ros::init(argc, argv, "move_group_interface_test");
    // This means that any topics, services, and parameters that are created with this `NodeHandle` will be in the `/trolley/panda` namespace.
    ros::NodeHandle nh("/trolley/panda");

    // Set the MONITORED_PLANNING_SCENE_TOPIC parameter
    nh.setParam("/move_group/monitored_planning_scene", "/trolley/panda/move_group/monitored_planning_scene");

    while(ros::ok()){
        ROS_INFO("Waiting for move_group_interface_test node to initialise...");
        if (ros::service::exists("/trolley/panda/move_group/get_loggers", false)){
            break;
        }
        ros::Duration(0.5).sleep();
    }

    Manipulate manipulate(nh);

    ros::AsyncSpinner spinner(3);
    spinner.start();

    bool status = manipulate.initialise();
    if (status == false){
        ROS_ERROR("Failed to initialise Manipulate class");
        ros::shutdown();
        return 1;
    }

    // pause by waiting for a keypress for debugging
    ROS_INFO("Press enter to continue...");
    std::cin.get();

    ros::shutdown();
    return 0;

}