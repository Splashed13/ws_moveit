// Include the header file for the "pick_and_place" functionality
#include "../include/panda_moveit/PickAndPlace.hpp"
#include "../include/panda_moveit/Utilities.hpp"

// Define the namespace to group related code together
    PickandPlace::PickandPlace(std::string scene_, std::string approach_) : 
    // identifiers responsible for initializing the declared variables
    nh("pnp"),
    visual_tools("panda_link0"),
    grasp_action_client("/franka_gripper/grasp", true)
    {    
        // Set up a publisher for the pose point visualization marker
        pose_point_pub = nh.advertise<visualization_msgs::Marker>("pose_point", 10);

        move_group_interface_arm = std::make_unique<moveit::planning_interface::MoveGroupInterface>(PLANNING_GROUP_ARM);
        move_group_interface_gripper = std::make_unique<moveit::planning_interface::MoveGroupInterface>(PLANNING_GROUP_GRIPPER);

        // Set the planning time for the arm movement
        move_group_interface_arm->setPlanningTime(PLANNING_TIME);

        // load the planning scene monitor
        visual_tools.loadPlanningSceneMonitor();

        // allocate scene_ and approach_ to declared variables
        scene = scene_;
        approach = approach_;

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

        starting_orientation = move_group_interface_arm->getCurrentPose().pose.orientation;

        // move_group_interface_arm->setPlannerId("RRTConnectkConfigDefault");
      

    }


    void PickandPlace::createCollisionObjectBox(std::string id, std::vector<double> dimensions, std::vector<double> position, double rotation_z)
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

    
    void PickandPlace::createCollisionScene(){   
        if(scene == "rod"){
            // Floor
            std::vector<double> floor_dimensions = {2.5, 2.5, 0.01};
            std::vector<double> floor_position = {0.0, 0.0, -0.01};
            createCollisionObjectBox("floor", floor_dimensions, floor_position, 0.0);

            // Box 1
            std::vector<double> box1_dimensions = {0.2, 0.4, 0.2};
            std::vector<double> box1_position = {0.6, 0.2, box1_dimensions[2] / 2.0};
            createCollisionObjectBox("box1", box1_dimensions, box1_position, 0.0);

            // Box 2
            std::vector<double> box2_dimensions = {0.3, 0.2, 0.1};
            std::vector<double> box2_position = {0, 0.6, box2_dimensions[2] / 2};
            createCollisionObjectBox("box2", box2_dimensions, box2_position, 0.0);

            // Rod
            std::vector<double> rod_dimensions = {0.02, 0.02, 0.2};
            std::vector<double> rod_position = {box1_position[0], box1_position[1], rod_dimensions[2] / 2.0 + box1_dimensions[2]};
            createCollisionObjectBox("rod", rod_dimensions, rod_position, 45.0);
            ROS_INFO("Basic Pick and Place scene created");

        }else if(scene == "cube"){
            // Table 
            std::vector<double> box1_dimensions = {0.2, 0.4, 0.2};
            std::vector<double> box1_position = {0.6, 0, box1_dimensions[2] / 2.0};
            createCollisionObjectBox("table", box1_dimensions, box1_position, 0.0);

            // Basic Cube
            std::vector<double> cube_dimensions = {0.02, 0.02, 0.02};
            std::vector<double> cube_position = {box1_position[0], 0, box1_dimensions[2] + cube_dimensions[2] / 2.0};
            createCollisionObjectBox("cube", cube_dimensions, cube_position, 0.0);


        }else{
            ROS_INFO("Empty scene");
        }

    }

    void PickandPlace::clean_scene(){   
        std::vector<std::string> object_ids;
        if(scene == "rod"){
            object_ids.push_back("box1");
            object_ids.push_back("box2");
            object_ids.push_back("rod");
            planning_scene_interface.removeCollisionObjects(object_ids);

        }else if(scene == "cube"){
            object_ids.push_back("table");
            object_ids.push_back("cube");
            planning_scene_interface.removeCollisionObjects(object_ids);

        }else{
            ROS_INFO("Empty scene- No objects to clean");
        }   

    }


    geometry_msgs::Pose PickandPlace::calculate_target_pose(std::vector<double> translation, std::vector<double> rotation, double ee_rotation_world_z, double pre_approach_distance)
    {   
        Eigen::Matrix4d homogeneous_mat;
        // Eigen::Matrix3d arrow_rotation;
        // rotation is about the relative frame of the end effector
        Eigen::Matrix4d homogeneous_mat_arm = Eigen::Matrix4d::Identity();
        homogeneous_mat_arm.block<3, 3>(0, 0) = Utilities::eulerXYZ_to_rotation_matrix(rotation);
        homogeneous_mat_arm(0, 3) = translation[0];
        homogeneous_mat_arm(1, 3) = translation[1];
        homogeneous_mat_arm(2, 3) = translation[2];

        // arrow_rotation = homogeneous_mat_arm.block<3, 3>(0, 0);

        // if planning group arm is panda_arm
        if(PLANNING_GROUP_ARM == "panda_arm"){
            // Create a homogeneous translation matrix to account for the end effector (use when using panda arm)
            Eigen::Matrix4d homogeneous_trans_end_effector = Eigen::Matrix4d::Identity();
            homogeneous_trans_end_effector(2, 3) = end_effector_palm_length + pre_approach_distance;
            homogeneous_mat = homogeneous_mat_arm * homogeneous_trans_end_effector.inverse(); 

        }else{
            Eigen::Matrix4d homogeneous_end_effector = Eigen::Matrix4d::Identity();
            homogeneous_end_effector(2, 3) = pre_approach_distance;
            homogeneous_mat = homogeneous_mat_arm * homogeneous_end_effector.inverse();    
        }
        
        // Convert the homogeneous transformation matrix to a pose
        geometry_msgs::Pose pose_target = Utilities::homogeneous_matrix_to_pose(homogeneous_mat);

        // add desired pose arrow
        add_pose_arrow(pose_target);
        
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

    // The main adjustment here is in the rotation of the arrow marker. 
    // First, the target pose's orientation is converted to an Eigen quaternion. 
    // Then a rotation quaternion is created for a -90 degree rotation around the y-axis. 
    // Finally, the marker's orientation quaternion is obtained by multiplying the end effector's quaternion by the rotation quaternion. 
    // The resulting quaternion is then converted back to a geometry_msgs/Quaternion and set as the marker's orientation.
    void PickandPlace::add_pose_arrow(geometry_msgs::Pose target_pose)
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

        // Adjust the orientation of the arrow to align with the end effector
        Eigen::Quaterniond q_end_effector(target_pose.orientation.w, target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z);
        Eigen::Quaterniond q_rotation(Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitY()));  // -90 degrees around y-axis
        Eigen::Quaterniond q_marker = q_end_effector * q_rotation;
        target_pose.orientation.w = q_marker.w();
        target_pose.orientation.x = q_marker.x();
        target_pose.orientation.y = q_marker.y();
        target_pose.orientation.z = q_marker.z();

        // Adjust the position of the arrow to make it end at the pose position
        Eigen::Matrix3d arrow_rot = q_marker.toRotationMatrix();
        Eigen::Vector3d displacement = arrow_rot.col(0) * end_effector_palm_length; // displacement along x-axis
        target_pose.position.x -= displacement.x();
        target_pose.position.y -= displacement.y();
        target_pose.position.z -= displacement.z();

        // set pose of arrow
        marker.pose = target_pose;
        
        // Publish the marker
        pose_point_pub.publish(marker);

        // print pose arrow successfully published
        ROS_INFO("Pose arrow successfully published");
    }

    void PickandPlace::determine_grasp_pose()
    {   
        // create homogeneous transformation matrix for the depth camera relative to the arm base
        Eigen::Matrix4d homogeneous_mat_cam = Eigen::Matrix4d::Identity();
        homogeneous_mat_cam.block<3, 3>(0, 0) = Utilities::eulerXYZ_to_rotation_matrix(depth_camera_rotation, false);
        homogeneous_mat_cam(0, 3) = depth_camera_position[0];
        homogeneous_mat_cam(1, 3) = depth_camera_position[1];
        homogeneous_mat_cam(2, 3) = depth_camera_position[2];

        // create homogeneous transformation matrix for the object relative to the depth camera
        Eigen::Matrix4d homogeneous_mat_object = Eigen::Matrix4d::Identity();
        homogeneous_mat_object(0, 3) = -0.01496;
        homogeneous_mat_object(1, 3) = -0.07034;
        homogeneous_mat_object(2, 3) = 0.7206;

        // create homogeneous transformation matrix for the object relative to the arm base
        Eigen::Matrix4d homogeneous_mat_base_object = homogeneous_mat_cam * homogeneous_mat_object;

        // retrieve point object from homogeneous transformation matrix
        geometry_msgs::Point position;
        position.x = homogeneous_mat_base_object(0, 3);
        position.y = homogeneous_mat_base_object(1, 3);
        position.z = homogeneous_mat_base_object(2, 3);

        // print the position of the object relative to the arm base
        ROS_INFO("Object position relative to the arm base: x = %f, y = %f, z = %f", position.x, position.y, position.z);
 
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


    bool PickandPlace::get_object_pose(std::string object_name)
    {
        auto object_poses = planning_scene_interface.getObjectPoses({object_name});   
        if (object_poses.count(object_name) == 0)
        {
            //ROS_ERROR("%s not found in object_poses", object_name.c_str());
            return false;  // Return an empty vector or some error value
        }

        object_pose = object_poses[object_name];
        return true;
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

    bool PickandPlace::plan_and_execute_pose(geometry_msgs::Pose pose_target){
        // set the pose target
        move_group_interface_arm->setPoseTarget(pose_target);

        // move to arm to the target pose
        bool success = (move_group_interface_arm->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        // print if the arm was able to move to the target pose
        ROS_INFO("Planning to pose target %s", success ? "" : "FAILED");

        if (success){
            ROS_INFO("Visualizing plan as trajectory line");
            visual_tools.publishTrajectoryLine(plan.trajectory_, joint_model_group_arm);
            visual_tools.trigger();


            // ROS_INFO("Press any button to travel to target pose");
            // std::cin.ignore();
            move_group_interface_arm->execute(plan);
            
            // clear Sphere and Path namespaces from the visual tools rviz topic
            visual_tools.deleteAllMarkers();
            visual_tools.trigger();

            // clear the pose arrow 
            // remove_pose_arrow();

            return true;

        }else{
            return false;
        }

    }
    
    // pick with hardcoded values first cube dimensions 0.013 x 0.013 x 0.013
    bool PickandPlace::pick(std::vector<double> position, double z_offset, double width, double speed, double force){

        geometry_msgs::Pose target_pose;
        // pre grasp
        target_pose.orientation = starting_orientation;
        target_pose.position.x = position[0];
        target_pose.position.y = position[1];
        target_pose.position.z = position[2] + 0.1;
        
        if(!plan_and_execute_pose(target_pose)){
            return false;
        }

        // target_pose.orientation = starting_orientation;
        // target_pose.position.x = position[0];
        // target_pose.position.y = position[1];

        // grasp
        target_pose.position.z = position[2] + z_offset; // can have position[2]+value to have ee arrive higher above object default? (0.01)

        double epsilon_inner = 0.005;
        double epsilon_outer = 0.005;

        if(!plan_and_execute_pose(target_pose)){
            return false;
        }
        if(!send_grasp_goal(width, epsilon_inner, epsilon_outer, speed, force)){     // stone: 0.03, 0.1 -speed , 10N
            return false;
        }

        // lift cube up by 0.15m
        target_pose.position.z = position[2] + 0.15; 

        if(!plan_and_execute_pose(target_pose)){
            return false;
        }

        return true;

        // put into chatgpt if necessary 
        // control_msgs::GripperCommandAction(width, max_effort): 
        // This is a standard gripper action recognized by MoveIt!, a ROS package for motion planning. 
        // If the max_effort parameter is greater than zero, 
        // the gripper will try to grasp an object of the desired width. If max_effort is zero, 
        // the gripper will just move to the desired width without attempting to grasp anything.
           
    }

    bool PickandPlace::place(){
        // set pose position to the table2 position plus a height of 0.01m (0.0, 0.6, 0.322 + 0.01) and 
        // have the gripper rotated 90 degrees about it starting orientation z-axis
        geometry_msgs::Pose target_pose;

        // First, set the position
        target_pose.position.x = 0.0;
        target_pose.position.y = 0.6;
        target_pose.position.z = 0.322 + 0.15;

        // Then, set the orientation. First, convert the 90 degrees rotation about the Z-axis to radians.
        double theta = -90.0 * M_PI / 180.0; 

        // Now, use the quaternion representation for a rotation about the Z-axis.
        // Assuming that starting_orientation is a quaternion representing the initial orientation
        tf2::Quaternion q_starting(starting_orientation.x, starting_orientation.y, starting_orientation.z, starting_orientation.w);

        // Create a quaternion representing the rotation around the Z-axis
        tf2::Quaternion q_rot;
        q_rot.setRPY(0, 0, theta);

        // The final orientation is the product of the initial orientation and the rotation
        // Note that quaternion multiplication is not commutative. The order of multiplication matters. 
        // In this case, q_starting * q_rot results in a rotation of q_rot in the coordinate frame of q_starting. 
        // If you want to rotate in the world coordinate frame, you should reverse the multiplication order.
        tf2::Quaternion q_final = q_starting * q_rot;

        target_pose.orientation.x = q_final.x();
        target_pose.orientation.y = q_final.y();
        target_pose.orientation.z = q_final.z();
        target_pose.orientation.w = q_final.w();

        // move to target pose
        if(!plan_and_execute_pose(target_pose)){
            return false;
        }

        // open gripper
        open_gripper();

        return true;

    }

    bool PickandPlace::send_grasp_goal(double width, double epsilon_inner, double epsilon_outer, double speed, double force)
    {
        franka_gripper::GraspGoal goal;

        // Fill in goal (adjust according to your needs)
        goal.width = width;
        goal.epsilon.inner = epsilon_inner;
        goal.epsilon.outer = epsilon_outer;
        goal.speed = speed;
        goal.force = force;

        // Send goal
        grasp_action_client.sendGoal(goal);

        // Wait for the action to return
        bool finished = grasp_action_client.waitForResult(ros::Duration(30.0));

        if (finished)
        {
            actionlib::SimpleClientGoalState state = grasp_action_client.getState();
            ROS_INFO("Action finished: %s", state.toString().c_str());
            return true;
        }
        else
        {
            ROS_INFO("Action did not finish before the time out.");
            return false;
        }
    }


    bool PickandPlace::close_gripper(std::vector<double> gripper_width){
        // Set the joint value target for the gripper
        move_group_interface_gripper->setJointValueTarget(gripper_width);
        bool success = (move_group_interface_gripper->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO("Closing gripper %s", success ? "" : "FAILED");
        move_group_interface_gripper->move();

        return success;
    }

    std::vector<double> PickandPlace::get_current_ee_position(void){

        return {move_group_interface_arm->getCurrentPose().pose.position.x, move_group_interface_arm->getCurrentPose().pose.position.y, move_group_interface_arm->getCurrentPose().pose.position.z};

    }

    

    void PickandPlace::user_input_pose(void){
        do {
            ROS_INFO("Enter RPY in degrees (-180 to 180) separated by space: ");
            std::cin >> ee_rotation[0] >> ee_rotation[1] >> ee_rotation[2];
            std::cin.clear();  // Clear the error flags
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');  // Clear the buffer
        } while (std::cin.fail() || 
                !(ee_rotation[0] >= -180 && ee_rotation[0] <= 180 &&
                ee_rotation[1] >= -180 && ee_rotation[1] <= 180 &&
                ee_rotation[2] >= -180 && ee_rotation[2] <= 180));

        do {
            ROS_INFO("Enter position in meters as x y z separated by space: ");
            std::cin >> ee_position[0] >> ee_position[1] >> ee_position[2];
            std::cin.clear();  // Clear the error flags
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');  // Clear the buffer
        } while (std::cin.fail());
    }



    void PickandPlace::run()
    {
        geometry_msgs::Pose desired_pose;

        // Write robot details
        writeRobotDetails();

        // open gripper
        open_gripper();

        // create collision scene
        createCollisionScene();

        do{
            if(!get_object_pose(scene)){
                ROS_INFO("Object not found - Enter pose manually");
                user_input_pose();
                desired_pose = calculate_target_pose(ee_position, ee_rotation);

            }else{
                std::vector<double> object_position = {object_pose.position.x, object_pose.position.y, object_pose.position.z};
                ROS_INFO("Object position: %f, %f, %f", object_position[0], object_position[1], object_position[2]);
                
                // set pose target (to change angle of the FLAT gripper, change the Y in RPY) - 'panda_arm' as planning group
                //geometry_msgs::Pose desired_pose = calculate_target_pose(object_position, {45, 90, 45});
            
                if(approach == "horizontal"){
                    // horizontal approach
                    desired_pose = calculate_target_pose(object_position, {0.0, 90.0, 0.0});
                }else if(approach == "vertical"){
                    // vertical approach
                    desired_pose = calculate_target_pose(object_position, {180.0, 0, 0});
                }

                ROS_INFO("Pose Calculated - Press any button to continue");
                std::cin.ignore();
                }

        }while(!plan_and_execute_pose(desired_pose));

        // press any button to return home
        ROS_INFO("Press any button to return home");
        std::cin.ignore();

        // reset
        go_to_home_position();
        clean_scene();

        ROS_INFO("Simulation Complete - Press any button to exit");
        std::cin.ignore();

    }
    
    void PickandPlace::test(){
        writeRobotDetails();
        open_gripper();

        // Iterate over cube_information map
        for(const auto& item : cube_information) { 
            // item.first is the key (i.e., "cube1", "cube2", "cube3")
            // item.second is the value (i.e., the vector of doubles)
            const std::vector<double>& value = item.second;

            if(pick({value[0],value[1],value[2]}, 0.001, value[3], 0.1, value[4])){ //0.0025
                ROS_INFO("Pick successful for %s", item.first.c_str());
            }else{
                ROS_INFO("Pick failed for %s", item.first.c_str());
            }

            if(place()){
                ROS_INFO("Place successful for %s", item.first.c_str());
            }else{
                ROS_INFO("Place failed for %s", item.first.c_str());
            }
        }

        // determine_grasp_pose();
        ROS_INFO("Simulation Complete - Press any button to exit");
        std::cin.ignore();
    }
