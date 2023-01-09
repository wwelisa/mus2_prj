import open3d as o3d
import numpy as np

def main():

    pcd = o3d.io.read_point_cloud("models/shoeBottom.pcd")


    plane_model, inliers = pcd.segment_plane(distance_threshold=0.2,
                                            ransac_n=3,
                                            num_iterations=5)
    
    inlier_cloud = pcd.select_by_index(inliers)
    
    inlier_cloud.paint_uniform_color([1.0, 0, 0])
    
    outlier_cloud = pcd.select_by_index(inliers, invert=True)
    
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud],
                                    zoom=0.3412,
                                    front=[0.4257, 0.2125, 0.8795],
                                    lookat=[2.6172, 2.0475, 1.532],
                                    up=[-0.0694, -0.9768, 0.2024])

    #o3d.io.write_point_cloud("models/shoeBottom_removed.pcd", outlier_cloud)

main()
