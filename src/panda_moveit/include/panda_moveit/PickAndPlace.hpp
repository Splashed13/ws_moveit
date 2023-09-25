#include <ros/ros.h>
#include <vector>
#include <cmath>
#include <math.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Geometry>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <tf2_eigen/tf2_eigen.h>
#include <sensor_msgs/JointState.h>
#include <boost/algorithm/string/join.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <actionlib/client/simple_action_client.h>
#include <franka_gripper/GraspAction.h>

class PickandPlace
{
private:
    ros::Publisher pose_point_pub;

    // Add an action client for the franka_gripper grasp action
    actionlib::SimpleActionClient<franka_gripper::GraspAction> grasp_action_client;

    const double PLANNING_TIME = 15.0;

    const std::vector<double> OPEN_GRIPPER = {0.035, 0.035};
    const std::vector<double> CLOSE_GRIPPER = {0.011, 0.011};
    const double end_effector_palm_length = 0.058 * 1.8;

    const std::string PLANNING_GROUP_ARM = "panda_manipulator"; // panda_arm shouldnt be used - check earlier commits if need it
    const std::string PLANNING_GROUP_GRIPPER = "panda_hand"; 

    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_arm;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_gripper;

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit_visual_tools::MoveItVisualTools visual_tools;
    const moveit::core::JointModelGroup* joint_model_group_arm;

    std::vector<double> home_joint_values;
    std::vector<double> depth_camera_position = {0.0, 0.4, 0.425};
    std::vector<double> depth_camera_rotation = {0.0, 0.26, -0.58875};

    geometry_msgs::Pose object_pose;

    std::string scene;
    std::string approach;

    // target orientation of the end effector
    geometry_msgs::Quaternion starting_orientation;

    // home joint positions {0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785};


    std::vector<double> ee_rotation = {0.0, 0.0, 0.0};
    std::vector<double> ee_position = {0.0, 0.0, 0.0};

    // map of gazebo cube positions, key is name of the cube, value is a vector of doubles 
    // containing the x, y, z, width and desired applied force of the gripper
    std::map<std::string, std::vector<double>> cube_information = {
        {"cube1", {0.6, 0.0, 0.317, 0.024, 10}},
        {"cube2", {0.6, 0.1, 0.3195, 0.029, 10}},
        {"cube3", {0.6, -0.1, 0.3155, 0.021, 10}}
    };

public:
    PickandPlace(std::string scene_, std::string approach_, ros::NodeHandle& nh);
    void writeRobotDetails(void);
    void createCollisionObjectBox(std::string id, std::vector<double> dimensions, std::vector<double> position, double rotation_z);
    void createCollisionScene(void);
    void clean_scene(void);
    geometry_msgs::Pose calculate_target_pose(std::vector<double> translation, std::vector<double> rotation, bool relative=false);
    void add_pose_arrow(geometry_msgs::Pose target_pose);
    std::vector<double> get_current_ee_position(void);
    bool plan_and_execute_pose(geometry_msgs::Pose target_pose);
    void add_pose_point(geometry_msgs::Point position);
    void determine_grasp_pose(void);
    void remove_pose_arrow(void);
    bool get_object_pose(std::string object_name);
    void go_to_home_position(void);
    void user_input_pose(void);    
    void go_to_zero_state(void);
    bool send_grasp_goal(double width, double epsilon_inner, double epsilon_outer, double speed, double force);

    void open_gripper(void);
    bool close_gripper(std::vector<double> gripper_width);
    
    bool pick(std::vector<double> position, double z_offset, double width, double speed, double force);
    bool place(void);   

    void rviz(void);
    void gazebo(void);

    bool move_cartesian_path_z(double z_postition);

    geometry_msgs::Pose user_defined_pose_vertical(void);

    void keyboard(void);

    int keyboard_options(void);

};
