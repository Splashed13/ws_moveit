import open3d as o3d
import numpy as np


class PointCloudBoundingBox(object):
    def __init__(self):
        self.pcd = o3d.io.read_point_cloud("/home/tho868/ws_moveit/src/panda_moveit/PointCloudData/test_pcd.pcd")

    def get_centre(self, draw=False) -> np.ndarray:
        # Convert Open3D.o3d.geometry.PointCloud to numpy array
        xyz = np.asarray(self.pcd.points)
        rgb = np.asarray(self.pcd.colors)

        # Assuming that the floor is the plane with the highest points along the y-axis, you can filter the points like so:
        threshold_y = 0.9  # This threshold should be set according to your specific needs
        mask = xyz[:,1] > np.max(xyz[:,1]) - threshold_y

        threshold_z = 0.25  # 0.7
        # extend the mask to the z-axis
        mask = np.logical_and(mask, xyz[:,2] > np.max(xyz[:,2]) - threshold_z)

        # Apply the mask
        filtered_xyz = xyz[np.asarray(mask)]
        filtered_rgb = rgb[np.asarray(mask)]

        # Create a new point cloud from the filtered points
        cropped_pcd = o3d.geometry.PointCloud()
        cropped_pcd.points = o3d.utility.Vector3dVector(filtered_xyz)
        cropped_pcd.colors = o3d.utility.Vector3dVector(filtered_rgb)



        # Convert Open3D.o3d.geometry.PointCloud to numpy array
        xyz_ = np.asarray(cropped_pcd.points)
        rgb_ = np.asarray(cropped_pcd.colors)

        # Assuming that red is represented as (1, 0, 0) in the RGB color space,
        # you can create a mask to filter the cube points
        threshold = 0.3  # This threshold should be set according to your specific needs
        mask = rgb_[:,0] > threshold  # Red channel should be higher than the threshold
        mask = np.logical_and(mask, rgb_[:,1] < threshold)  # Green channel should be lower than the threshold
        mask = np.logical_and(mask, rgb_[:,2] < threshold)  # Blue channel should be lower than the threshold

        # Apply the mask
        cube_xyz = xyz_[np.asarray(mask)]
        cube_rgb = rgb_[np.asarray(mask)]

        # Create a new point cloud from the cube points
        cube_pcd = o3d.geometry.PointCloud()
        cube_pcd.points = o3d.utility.Vector3dVector(cube_xyz)
        cube_pcd.colors = o3d.utility.Vector3dVector(cube_rgb)

        # # Calculate the bounding box
        bbox = cube_pcd.get_oriented_bounding_box()
        bbox.color = np.array([0, 1, 0])  # set the color of the bounding box to green

        if draw:
            # Visualize the point cloud along with the coordinate frame and bounding box
            coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
            o3d.visualization.draw_geometries([cube_pcd, coord_frame, bbox])

        return bbox.get_center()


    def top_view(self, pcd: o3d.geometry.PointCloud):
        # Compute the centroid of the point cloud
        centroid = np.mean(np.asarray(pcd.points), axis=0)

        # Define a rotation matrix. Here, we rotate the point cloud by 90 degrees around the x-axis
        theta = -np.radians(90)  # Convert the angle to radians
        rotation_matrix = np.array([[1, 0, 0],
                                    [0, np.cos(theta), -np.sin(theta)],
                                    [0, np.sin(theta), np.cos(theta)]])

        # Subtract the centroid from the point cloud, apply the rotation, then add the centroid back
        pcd.points = o3d.utility.Vector3dVector(
            (np.asarray(pcd.points) - centroid).dot(rotation_matrix) + centroid
        )

if __name__ == '__main__':
    pcd = PointCloudBoundingBox()
    position = pcd.get_centre(draw=True)
    print(position)
