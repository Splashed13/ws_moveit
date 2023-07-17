#include "../include/panda_moveit/Utilities.hpp"

Utilities::Utilities(){}

Eigen::Matrix3d Utilities::eulerXYZ_to_rotation_matrix(std::vector<double> euler_angles, bool degrees){
    if (degrees){
        euler_angles[0] = euler_angles[0] * M_PI / 180.0;
        euler_angles[1] = euler_angles[1] * M_PI / 180.0;
        euler_angles[2] = euler_angles[2] * M_PI / 180.0;
    }

    Eigen::Matrix3d Rx, Ry, Rz;
    Rx = (Eigen::AngleAxisd(euler_angles[0], Eigen::Vector3d::UnitX())).toRotationMatrix();
    Ry = (Eigen::AngleAxisd(euler_angles[1], Eigen::Vector3d::UnitY())).toRotationMatrix();
    Rz = (Eigen::AngleAxisd(euler_angles[2], Eigen::Vector3d::UnitZ())).toRotationMatrix();
    
    return Rx * Ry * Rz;

}


geometry_msgs::Pose Utilities::homogeneous_matrix_to_pose(Eigen::Matrix4d homogeneous_matrix){
    geometry_msgs::Pose pose;
    pose.position.x = homogeneous_matrix(0, 3);
    pose.position.y = homogeneous_matrix(1, 3);
    pose.position.z = homogeneous_matrix(2, 3);

    Eigen::Quaterniond quaternion(homogeneous_matrix.block<3, 3>(0, 0));
    pose.orientation.x = quaternion.x();
    pose.orientation.y = quaternion.y();
    pose.orientation.z = quaternion.z();
    pose.orientation.w = quaternion.w();

    return pose;
}
