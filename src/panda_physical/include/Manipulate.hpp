#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <boost/algorithm/string/join.hpp>

// barebones class for manipulating the robot via a keyboard interface
class Manipulate
{
    private:
        const std::string PLANNING_GROUP_ARM = "panda_arm_hand"; 
        const std::string PLANNING_GROUP_GRIPPER = "panda_hand"; 
        const double PLANNING_TIME = 15.0;

        std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_arm;
        std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_gripper;

    public:
        Manipulate(ros::NodeHandle& nh);
        bool initialise(void);
};
