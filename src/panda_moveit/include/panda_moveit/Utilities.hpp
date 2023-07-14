#include <Eigen/Geometry>
#include <vector>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <math.h>


class Utilities
{
private:
    /* data */
public:
    Utilities();
    static Eigen::Matrix3d eulerXYZ_to_rotation_matrix(std::vector<double> euler_angles, bool degrees = true);
    static geometry_msgs::Pose homogeneous_matrix_to_pose(Eigen::Matrix4d homogeneous_matrix);

};

