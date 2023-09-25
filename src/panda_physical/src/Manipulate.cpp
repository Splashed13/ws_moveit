#include "../include/Manipulate.hpp"

Manipulate::Manipulate(ros::NodeHandle& nh)
{
    // initialise the move group interface for the arm
    move_group_interface_arm = std::make_unique<moveit::planning_interface::MoveGroupInterface>(PLANNING_GROUP_ARM);
    move_group_interface_arm->setPlanningTime(PLANNING_TIME);
    move_group_interface_arm->setNumPlanningAttempts(10);
    //move_group_interface_arm->setPlannerId("RRTConnectkConfigDefault");
    move_group_interface_arm->setMaxVelocityScalingFactor(0.5);
    move_group_interface_arm->setMaxAccelerationScalingFactor(0.5);

    // initialise the move group interface for the gripper
    move_group_interface_gripper = std::make_unique<moveit::planning_interface::MoveGroupInterface>(PLANNING_GROUP_GRIPPER);
    move_group_interface_gripper->setPlanningTime(PLANNING_TIME);
    move_group_interface_gripper->setNumPlanningAttempts(10);
    //move_group_interface_gripper->setPlannerId("RRTConnectkConfigDefault");
    move_group_interface_gripper->setMaxVelocityScalingFactor(0.5);
    move_group_interface_gripper->setMaxAccelerationScalingFactor(0.5);
}

bool Manipulate::initialise(){
    try{
    // assign the joint model group of panda arm to declared raw pointer (used for visual tools (trajectory tracking))
    const moveit::core::JointModelGroup* joint_model_group_arm = move_group_interface_arm->getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);
    // Print out the planning frame for the arm
    ROS_INFO("Planning frame: %s", move_group_interface_arm->getPlanningFrame().c_str());

    // Get the link names for the arm and concatenate them into a single string, then print
    std::vector<std::string> linkNames = move_group_interface_arm->getLinkNames();
    std::string linkNamesArm = boost::algorithm::join(linkNames, ", ");
    ROS_INFO("Arm links: %s", linkNamesArm.c_str());

    // Get the joint names for the arm and concatenate them into a single string, then print
    std::vector<std::string> jointNamesArm = move_group_interface_arm->getJoints();
    // Get the joint names for the gripper and concatenate them into a single string, then print
    std::vector<std::string> jointNamesGripper = move_group_interface_gripper->getJoints();
    std::string jointNamesGripperString = boost::algorithm::join(jointNamesGripper, ", ");
    ROS_INFO("Gripper joints names: %s", jointNamesGripperString.c_str());

    // Get the list of all available planning groups, concatenate them into a single string, then print
    std::vector<std::string> planningGroups = move_group_interface_arm->getJointModelGroupNames();
    std::string planningGroupsString = boost::algorithm::join(planningGroups, ", ");
    ROS_INFO("Available Planning Groups: %s", planningGroupsString.c_str());

    // Get the current joint values of the arm and print
    std::vector<double> jointValuesArm;
    jointValuesArm = move_group_interface_arm->getCurrentJointValues();
    std::vector<std::string> strValues;
    for (const auto &joint : jointValuesArm)
    {
        strValues.push_back(std::to_string(joint));
    }
    std::string jointValuesString = boost::algorithm::join(strValues, ", ");
    ROS_INFO("Home joint values: %s", jointValuesString.c_str());

    return true;
    
    }catch(std::exception& e){
        ROS_ERROR_STREAM("Error: " << e.what());
        return false;
    }

}