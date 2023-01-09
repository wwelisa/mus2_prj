import open3d as o3d
import numpy as np


def visualize(mesh):
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(mesh)
    vis.run()
    vis.destroy_window()

def display_inlier_outlier(cloud, ind):
    inlier_cloud = cloud.select_by_index(ind)
    outlier_cloud = cloud.select_by_index(ind, invert=True)

    print("Showing outliers (red) and inliers (gray): ")
    outlier_cloud.paint_uniform_color([1, 0, 0])
    inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud],
                                      zoom=0.3412,
                                      front=[0.4257, -0.2125, -0.8795],
                                      lookat=[2.6172, 2.0475, 1.532],
                                      up=[-0.0694, -0.9768, 0.2024])


def main():
    #get obj as triangle mesh
    textured_mesh = o3d.io.read_triangle_mesh("models/WIP/shoe_bottom.obj")

    #visualize(textured_mesh)

    #get vertices and color -> save as np array
    points = np.asarray(textured_mesh.vertices)
    points_color = np.asarray(textured_mesh.vertex_colors)

    #convert from np array to a pcd 
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(points_color)


    Top = o3d.io.read_point_cloud("models/shoeTop.pcd")
    #Top = Top.voxel_down_sample(voxel_size=0.05)
    Top.paint_uniform_color([1.0, 0, 0])

    pcd_smaller = pcd.scale(0.6, center=pcd.get_center())
    R = pcd_smaller.get_rotation_matrix_from_xyz((np.pi / 2, np.pi / 2, np.pi / 2))
    pcd_smaller.rotate(R, center=(0, 0, 0))

    o3d.visualization.draw_geometries([pcd_smaller, Top],
                                    zoom=0.3412,
                                    front=[0.4257, 0.2125, 0.8795],
                                    lookat=[2.6172, 2.0475, 1.532],
                                    up=[-0.0694, -0.9768, 0.2024])
    #####################################################################################

    #downsample the pc to reduce time 
    pcd_reduced = pcd_smaller.voxel_down_sample(voxel_size=0.03)

    #save the point cloud
    #o3d.io.write_point_cloud("models/shoeTop.pcd", pcd)


    o3d.visualization.draw_geometries([pcd_reduced],
                                    zoom=0.3412,
                                    front=[0.4257, 0.2125, 0.8795],
                                    lookat=[2.6172, 2.0475, 1.532],
                                    up=[-0.0694, -0.9768, 0.2024])
    
    #remove the oulier std_ratio -> smaller = more aggressive
    
    cl, ind = pcd_reduced.remove_statistical_outlier(nb_neighbors=500, std_ratio=0.9)

    o3d.visualization.draw_geometries([cl, Top],
                                    zoom=0.3412,
                                    front=[0.4257, 0.2125, 0.8795],
                                    lookat=[2.6172, 2.0475, 1.532],
                                    up=[-0.0694, -0.9768, 0.2024])

    #save the cleaned up pc
    o3d.io.write_point_cloud("models/shoeBottom.pcd", cl)


main()
