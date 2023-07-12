// Include the header file for the "pick_and_place" functionality
#include "../include/panda_moveit/PickAndPlace.hpp"

// Define the namespace to group related code together
    PickandPlace::PickandPlace() : 
    // identifiers responsible for initializing the declared variables
    nh("pnp"),
    visual_tools("panda_link0")
    {    
        // Set up a publisher for the pose point visualization marker
        pose_point_pub = nh.advertise<visualization_msgs::Marker>("pose_point", 10);

        move_group_interface_arm = std::make_unique<moveit::planning_interface::MoveGroupInterface>(PLANNING_GROUP_ARM);
        move_group_interface_gripper = std::make_unique<moveit::planning_interface::MoveGroupInterface>(PLANNING_GROUP_GRIPPER);


        // Set the planning time for the arm movement
        move_group_interface_arm->setPlanningTime(PLANNING_TIME);

        // load the planning scene monitor
        visual_tools.loadPlanningSceneMonitor();

    }

    void PickandPlace::writeRobotDetails()
    {
        // Print out the planning frame for the arm
        ROS_INFO("Planning frame: %s", move_group_interface_arm->getPlanningFrame().c_str());

        // Get the link names for the arm and concatenate them into a single string, then print
        std::vector<std::string> linkNames = move_group_interface_arm->getLinkNames();
        std::string linkNamesArm = boost::algorithm::join(linkNames, ", ");
        ROS_INFO("Arm links: %s", linkNamesArm.c_str());

        // Get the joint names for the arm and concatenate them into a single string, then print
        std::vector<std::string> jointNamesArm = move_group_interface_arm->getJoints();
        std::string jointNamesArmString = boost::algorithm::join(jointNamesArm, ", ");
        ROS_INFO("Arm joint names: %s", jointNamesArmString.c_str());

        // Get the joint names for the gripper and concatenate them into a single string, then print
        std::vector<std::string> jointNamesGripper = move_group_interface_gripper->getJoints();
        std::string jointNamesGripperString = boost::algorithm::join(jointNamesGripper, ", ");
        ROS_INFO("Gripper joints names: %s", jointNamesGripperString.c_str());

        // Get the list of all available planning groups, concatenate them into a single string, then print
        std::vector<std::string> planningGroups = move_group_interface_arm->getJointModelGroupNames();
        std::string planningGroupsString = boost::algorithm::join(planningGroups, ", ");
        ROS_INFO("Available Planning Groups: %s", planningGroupsString.c_str());

    
        // Get the home joint values and print them out
        home_joint_values = move_group_interface_arm->getCurrentJointValues();
        std::vector<std::string> strValues;
        for (const auto &joint : home_joint_values)
        {
            strValues.push_back(std::to_string(joint));
        }
        std::string jointValuesString = boost::algorithm::join(strValues, ", ");
        ROS_INFO("Home joint values: %s", jointValuesString.c_str());

        // assign the joint model group of panda arm to declared raw pointer
        joint_model_group_arm = move_group_interface_arm->getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);


    }


    void PickandPlace::createCollisionObject(std::string id, std::vector<double> dimensions, std::vector<double> position, double rotation_z)
    {
        // Create collision object
        moveit_msgs::CollisionObject collision_object;
        collision_object.header.frame_id = "panda_link0";
        collision_object.id = id;

        // Convert rotation to quaternion
        double rotation_radians = rotation_z * M_PI / 180.0;
        tf2::Quaternion quaternion;
        quaternion.setRPY(0.0, 0.0, rotation_radians);

        // Define a box to add to the world.
        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = dimensions[0];
        primitive.dimensions[primitive.BOX_Y] = dimensions[1];
        primitive.dimensions[primitive.BOX_Z] = dimensions[2];

        // Define a pose for the box (specified relative to frame_id)
        geometry_msgs::Pose box_pose;
        box_pose.position.x = position[0];
        box_pose.position.y = position[1];
        box_pose.position.z = position[2];
        box_pose.orientation = tf2::toMsg(quaternion);

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;

        std::vector<moveit_msgs::CollisionObject> collision_objects;
        collision_objects.push_back(collision_object); 

        // Add the collision object into the world
        // (using a vector that could contain additional objects)
        planning_scene_interface.addCollisionObjects(collision_objects);

        // ROS_INFO_NAMED added object frame.id to world
        ROS_INFO("Added %s into the world", id.c_str());
    }

    
    void PickandPlace::createCollisionScene()
    {
        // Floor
        std::vector<double> floor_dimensions = {2.5, 2.5, 0.01};
        std::vector<double> floor_position = {0.0, 0.0, -0.01};
        createCollisionObject("floor", floor_dimensions, floor_position, 0.0);

        // Box 1
        std::vector<double> box1_dimensions = {0.2, 0.4, box1.at("z_height")};
        std::vector<double> box1_position = {box1.at("x_pos"), box1.at("y_pos"), box1.at("z_height") / 2.0};
        createCollisionObject("box1", box1_dimensions, box1_position, 0.0);

        // Box 2
        std::vector<double> box2_dimensions = {0.3, 0.2, box2.at("z_height")};
        std::vector<double> box2_position = {box2.at("x_pos"), box2.at("y_pos"), box2.at("z_height") / 2};
        createCollisionObject("box2", box2_dimensions, box2_position, 0.0);

        // Rod
        std::vector<double> rod_dimensions = {0.02, 0.02, rod_height};
        std::vector<double> rod_position = {box1.at("x_pos"), box1.at("y_pos"), rod_height / 2.0 + box1.at("z_height")};
        createCollisionObject("rod", rod_dimensions, rod_position, 45.0);
    }

    void PickandPlace::clean_scene()
    {
        std::vector<std::string> object_ids;
        object_ids.push_back("floor");
        object_ids.push_back("box1");
        object_ids.push_back("box2");
        object_ids.push_back("rod");
        planning_scene_interface.removeCollisionObjects(object_ids);
    }

    Eigen::Matrix3d PickandPlace::eulerXYZ_to_rotation_matrix(std::vector<double> euler_angles){
        std::vector<double> rotation_rads = {
            euler_angles[0] * M_PI / 180.0,
            euler_angles[1] * M_PI / 180.0,
            euler_angles[2] * M_PI / 180.0
        };

        Eigen::Matrix3d Rx, Ry, Rz;
        Rx = (Eigen::AngleAxisd(rotation_rads[0], Eigen::Vector3d::UnitX())).toRotationMatrix();
        Ry = (Eigen::AngleAxisd(rotation_rads[1], Eigen::Vector3d::UnitY())).toRotationMatrix();
        Rz = (Eigen::AngleAxisd(rotation_rads[2], Eigen::Vector3d::UnitZ())).toRotationMatrix();
        
        return Rx * Ry * Rz;

    }


    geometry_msgs::Pose PickandPlace::calculate_target_pose(std::vector<double> translation, std::vector<double> rotation)
    {
        // rotation is about the relative frame of the end effector
        Eigen::Matrix4d homogeneous_mat_arm = Eigen::Matrix4d::Identity();
        //Eigen::Matrix3d R_ee = eulerXYZ_to_rotation_matrix(rotation);
        homogeneous_mat_arm.block<3, 3>(0, 0) = eulerXYZ_to_rotation_matrix(rotation);
        homogeneous_mat_arm(0, 3) = translation[0];
        homogeneous_mat_arm(1, 3) = translation[1];
        homogeneous_mat_arm(2, 3) = translation[2];

        // Create a homogeneous transformation matrix for the end effector with a rotation 
        // to align with panda_hand TF frame (conventional gripper axis)
        Eigen::Matrix4d homogeneous_trans_end_effector = Eigen::Matrix4d::Identity();
        // Eigen::Matrix3d Rz = (Eigen::AngleAxisd(-45*M_PI/180.0, Eigen::Vector3d::UnitZ())).toRotationMatrix();
        // homogeneous_trans_end_effector.block<3, 3>(0, 0) = Rz;
        homogeneous_trans_end_effector(2, 3) = end_effector_palm_length;

        // // relative rotation of end effector descirbed with homogeneous transformation matrix
        // Eigen::Matrix4d homogeneous_mat_ee = Eigen::Matrix4d::Identity();
        // homogeneous_mat_ee.block<3, 3>(0, 0) = R_ee;

        // Multiply the homogeneous transformation matrix of the arm by the inverse of the homogeneous transformation matrix of the end effector
        Eigen::Matrix4d homogeneous_mat = homogeneous_mat_arm; // * homogeneous_trans_end_effector; // * homogeneous_mat_ee; // .inverse();

        // retreive rotation matrix from homogeneous_mat as Eigen::Matrix3d
        Eigen::Matrix3d R = homogeneous_mat.block<3, 3>(0, 0);

        // Create a quaternion from euler angles
        tf2::Quaternion quaternion;
        // get a quaternion from the rotation matrix R
        Eigen::Quaterniond eigen_quat(R);
        // convert eigen quaternion to tf quaternion
        tf2::convert(tf2::toMsg(eigen_quat), quaternion);

        // Create message types for the pose target
        geometry_msgs::Quaternion orientation;
        orientation = tf2::toMsg(quaternion);

        geometry_msgs::Point position;
        position.x = homogeneous_mat(0, 3);
        position.y = homogeneous_mat(1, 3);
        position.z = homogeneous_mat(2, 3);

        // Set the target pose message
        geometry_msgs::Pose pose_target;
        pose_target.position = position;
        pose_target.orientation = orientation;

        // add pose point for testing purposes
        //add_pose_point(position);

        add_pose_arrow(homogeneous_mat);

        // add pose arrow
        //add_pose_arrow(pose_target.position, rotation[2]*M_PI/180.0);   
        
        // for testing purposes
        //add_pose_arrow(pose_target.position, 0.0);
        
        return pose_target;
    }

    void PickandPlace::add_pose_point(geometry_msgs::Point desired_position)
    {
        // Publish a marker at the desired pose
        visualization_msgs::Marker marker;
        marker.ns = "point";
        marker.id = 0;
        marker.header.frame_id = "panda_link0";
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.02;
        marker.scale.y = 0.02;
        marker.scale.z = 0.02;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;

        marker.pose.position = desired_position;

        pose_point_pub.publish(marker);

        ROS_INFO("Added pose point");
    }

    void PickandPlace::add_pose_arrow(Eigen::Matrix4d ee_pose)
    {
        // Publish a marker at the desired pose
        visualization_msgs::Marker marker;
        marker.ns = "arrow";
        marker.id = 0;
        marker.header.frame_id = "panda_link0";
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = end_effector_palm_length;
        marker.scale.y = 0.02;
        marker.scale.z = 0.02;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;

        // tf2::Quaternion quaternion;
        // quaternion.setRPY(0, 0, z_rotation);
        // geometry_msgs::Pose Pose;
        // Pose.position = desired_position;
        // Pose.orientation = tf2::toMsg(quaternion);
        // marker.pose = Pose;

        // rotation of arrow to align with palm frame with respect to the world frame
        Eigen::Matrix4d homogeneous_mat = Eigen::Matrix4d::Identity();
        homogeneous_mat.block<3,3>(0, 0) = eulerXYZ_to_rotation_matrix({0.0, -90.0, 0.0});

        // multiply the homogeneous transformation matrix of the arrow by the homogeneous transformation matrix of the end effector
        Eigen::Matrix4d homogeneous_mat_arrow = homogeneous_mat * ee_pose;

        // retrieve pose from homogeneous matrix
        geometry_msgs::Pose pose;
        pose.position.x = ee_pose(0, 3);
        pose.position.y = ee_pose(1, 3);
        pose.position.z = ee_pose(2, 3);

        // convert rotation matrix to quaternion
        Eigen::Matrix3d R = homogeneous_mat_arrow.block<3, 3>(0, 0);
        tf2::Quaternion quaternion;
        Eigen::Quaterniond eigen_quat(R);
        tf2::convert(tf2::toMsg(eigen_quat), quaternion);

        // set orientation of arrow
        pose.orientation = tf2::toMsg(quaternion);

        // set pose of arrow
        marker.pose = pose;
         
        // Publish the marker
        pose_point_pub.publish(marker);

        // print pose arrow successfully published
        ROS_INFO("Pose arrow successfully published");
    }

    void PickandPlace::determine_grasp_pose()
    {   
        // create homogeneous transformation matrix for the depth camera relative to the arm base
        Eigen::Matrix4d homogeneous_mat_cam = Eigen::Matrix4d::Identity();
        Eigen::Matrix3d Rz1, Ry, Rz2;
        Rz1 = (Eigen::AngleAxisd(depth_camera_rotation[0], Eigen::Vector3d::UnitX())).toRotationMatrix();
        Ry = (Eigen::AngleAxisd(depth_camera_rotation[1], Eigen::Vector3d::UnitY())).toRotationMatrix();
        Rz2 = (Eigen::AngleAxisd(depth_camera_rotation[2], Eigen::Vector3d::UnitZ())).toRotationMatrix();
        Eigen::Matrix3d R = Rz1 * Ry * Rz2;
        homogeneous_mat_cam.block<3, 3>(0, 0) = R;
        homogeneous_mat_cam(0, 3) = depth_camera_position[0];
        homogeneous_mat_cam(1, 3) = depth_camera_position[1];
        homogeneous_mat_cam(2, 3) = depth_camera_position[2];

        // create homogeneous transformation matrix for the object relative to the depth camera
        Eigen::Matrix4d homogeneous_mat_object = Eigen::Matrix4d::Identity();
        homogeneous_mat_object(0, 3) = -0.01496;
        homogeneous_mat_object(1, 3) = -0.07034;
        homogeneous_mat_object(2, 3) = 0.7206;

        // create homogeneous transformation matrix for the object relative to the arm base
        Eigen::Matrix4d homogeneous_mat_object_arm = homogeneous_mat_cam * homogeneous_mat_object;

        // retrieve point object from homogeneous transformation matrix
        geometry_msgs::Point position;
        position.x = homogeneous_mat_object_arm(0, 3);
        position.y = homogeneous_mat_object_arm(1, 3);
        position.z = homogeneous_mat_object_arm(2, 3);

        // print the position of the object relative to the arm base
        ROS_INFO("Object position relative to the arm base: x = %f, y = %f, z = %f", position.x, position.y, position.z);

        // add pose arrow for object
        // add_pose_arrow(position, 1.57);       
 
    }

    void PickandPlace::go_to_zero_state(void)
    {
        // move all joints to zero state
        std::vector<double> joint_group_positions = {0, 0, 0, 0, 0, 0, 0};
        move_group_interface_arm->setJointValueTarget(joint_group_positions);
        move_group_interface_arm->move();

    }


    void PickandPlace::remove_pose_arrow()
    {
        visualization_msgs::Marker marker;
        marker.ns = "arrow";
        marker.id = 0;
        marker.action = visualization_msgs::Marker::DELETE;

        // Publish the marker
        pose_point_pub.publish(marker);

        // print pose arrow successfully removed
        ROS_INFO("Pose arrow successfully removed");
    }


    std::vector<double> PickandPlace::get_rod_position()
    {
        auto object_poses = planning_scene_interface.getObjectPoses({"rod"});   
        if (object_poses.count("rod") == 0)
        {
            throw std::runtime_error("Rod not found in object_poses");
        }

        auto rod_pose = object_poses["rod"];
        return {rod_pose.position.x, rod_pose.position.y, rod_pose.position.z};
    }

    void PickandPlace::go_to_home_position()
    {
        // uses the home_joint_value variable to go to the home position of the robot
        move_group_interface_arm->setJointValueTarget(home_joint_values);

        // move to arm to the target pose
        bool success = (move_group_interface_arm->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        // print if the arm was able to move to the target pose
        ROS_INFO("Moving to home position %s", success ? "" : "FAILED");

        // move the arm to the target pose
        move_group_interface_arm->move();
    
    }

    void PickandPlace::open_gripper(){
        // Set the joint value target for the gripper
        move_group_interface_gripper->setJointValueTarget(OPEN_GRIPPER);
        bool success = (move_group_interface_gripper->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO("Opening gripper %s", success ? "" : "FAILED");
        move_group_interface_gripper->move();

    }

    void PickandPlace::plan_and_execute_pose(geometry_msgs::Pose pose_target){
        // set the pose target
        move_group_interface_arm->setPoseTarget(pose_target);

        // move to arm to the target pose
        bool success = (move_group_interface_arm->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        // print if the arm was able to move to the target pose
        ROS_INFO("Planning to pose target %s", success ? "" : "FAILED");

        ROS_INFO("Visualizing plan as trajectory line");
        visual_tools.publishTrajectoryLine(plan.trajectory_, joint_model_group_arm);
        visual_tools.trigger();


        ROS_INFO("Press any button to travel to target pose");
        std::cin.ignore();
        move_group_interface_arm->execute(plan);

        // clear the visual tools
        visual_tools.deleteAllMarkers();

    }

    std::vector<double> PickandPlace::get_current_ee_position(void){

        return {move_group_interface_arm->getCurrentPose().pose.position.x, move_group_interface_arm->getCurrentPose().pose.position.y, move_group_interface_arm->getCurrentPose().pose.position.z};

    }


    void PickandPlace::run_basic_pnp()
    {
        // Write robot details
        writeRobotDetails();

        // open gripper
        open_gripper();

        // Create collision scene
        //createCollisionScene();

        // Get rod position
        //std::vector<double> rod_position = get_rod_position();
        
        // // just the rod no box for testing purposes
        std::vector<double> rod_position = {box1.at("x_pos"), box1.at("y_pos"), rod_height / 2.0 + box1.at("z_height")};
        ROS_INFO("Rod position: %f, %f, %f", rod_position[0], rod_position[1], rod_position[2]);
        
        // set pose target (to change angle of the FLAT gripper, change the Y in RPY)
        //geometry_msgs::Pose desired_pose = calculate_target_pose(rod_position, {45, 90, 45});

        // horizontal approach
        geometry_msgs::Pose desired_pose = calculate_target_pose(rod_position, {0, 180, 0});

        // vertical approach
        // geometry_msgs::Pose desired_pose = calculate_target_pose(rod_position, {0, 180, 0});


        //go_to_zero_state();

        // print position of end effector
        ROS_INFO("End effector position: x = %f, y = %f, z = %f", move_group_interface_arm->getCurrentPose().pose.position.x, move_group_interface_arm->getCurrentPose().pose.position.y, move_group_interface_arm->getCurrentPose().pose.position.z);
    

        ROS_INFO("Pose Calculated - Press any button to continue");
        std::cin.ignore();

        plan_and_execute_pose(desired_pose);

        // press any button to return home
        ROS_INFO("Press any button to return home");
        std::cin.ignore();

        // reset
        go_to_home_position();
        clean_scene();
        remove_pose_arrow();

        ROS_INFO("Simulation Complete - Press any button to exit");
        std::cin.ignore();
   
    }
    
    void PickandPlace::test(){
        writeRobotDetails();
        open_gripper();
        go_to_zero_state(); 
        ROS_INFO("Simulation Complete - Press any button to exit");
        std::cin.ignore();

        // determine_grasp_pose();
        // ROS_INFO("Simulation Complete - Press any button to exit");
        // std::cin.ignore();
        // remove_pose_arrow();
    }



// ensure main is placed outside the namespace
int main(int argc, char **argv)
{
    ros::init(argc, argv, "pick_and_place");

    // Instantiate the PickandPlace class
    PickandPlace pnp;

    // ROS spinning must be running for the MoveGroupInterface to get information
    // about the robot's state. One way to do this is to start an AsyncSpinner beforehand.
    ros::AsyncSpinner spinner(5);
    spinner.start();

    // add a short sleep so the node can finish initializing
    ros::Duration(0.5).sleep();

    // Run pick and place operations
    pnp.run_basic_pnp();

    // Shutdown the node and join the thread back before exiting
    ros::shutdown();

    return 0;
}