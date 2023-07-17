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

class PickandPlace
{
private:
    ros::NodeHandle nh;
    ros::Publisher pose_point_pub;
    const double PLANNING_TIME = 15.0;
    
    const double gripper_max = 0.04;
    const double gripper_min = 0.0;

    // rotation max degrees
    const double rotation_max = 180.0;
    const double rotation_min = -180.0;

    const std::vector<double> OPEN_GRIPPER = {0.035, 0.035};
    const std::vector<double> CLOSE_GRIPPER = {0.011, 0.011};
    const double end_effector_palm_length = 0.058 * 1.8;

    const std::string PLANNING_GROUP_ARM = "panda_manipulator";
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


    std::vector<double> ee_rotation = {0.0, 0.0, 0.0};
    std::vector<double> ee_position = {0.0, 0.0, 0.0};

public:
    PickandPlace(std::string scene_, std::string approach_);
    void writeRobotDetails(void);
    void createCollisionObjectBox(std::string id, std::vector<double> dimensions, std::vector<double> position, double rotation_z);
    void createCollisionScene(void);
    void clean_scene(void);
    geometry_msgs::Pose calculate_target_pose(std::vector<double> translation, std::vector<double> rotation, double ee_rotation_world_z = 0.0, double pre_approach_distance = 0.0);
    void add_pose_arrow(geometry_msgs::Point point, Eigen::Matrix3d desired_pose_R);
    std::vector<double> get_current_ee_position(void);
    bool plan_and_execute_pose(geometry_msgs::Pose target_pose);
    void add_pose_point(geometry_msgs::Point position);
    void determine_grasp_pose(void);
    void remove_pose_arrow(void);
    bool get_object_pose(std::string object_name);
    void go_to_home_position(void);
    void user_input_pose(void);    
    void go_to_zero_state(void);
    void open_gripper(void);

    void run(void);
    void test(void);
};
