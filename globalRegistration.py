import open3d as o3d
import numpy as np
import copy
import math


def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)

    mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.6, origin=[1, 0, 0])

    o3d.visualization.draw_geometries([source_temp, target_temp],
                                      zoom=2.0,
                                      front=[0.4257, -0.2125, -0.8795],
                                      lookat=[2.6172, 2.0475, 1.532],
                                      up=[-0.0694, -0.9768, 0.2024])


def preprocess_point_cloud(pcd, voxel_size):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh      


def prepare_dataset(voxel_size):
    print(":: Load two point clouds and disturb initial pose.")

    demo_icp_pcds = o3d.data.DemoICPPointClouds()
    source = o3d.io.read_point_cloud("models/shoeTop_removed.pcd")
    source = source.scale(0.88, center=source.get_center())
    target = o3d.io.read_point_cloud("models/shoeBottom_removed.pcd")
    trans_init = np.asarray([[1.0, 0.0, 0.0, 0.0], 
                             [0.0, 1.0, 0.0, 0.0],
                             [0.0, 0.0, 1.0, 0.0], 
                             [0.0, 0.0, 0.0, 1.0]])
    source.transform(trans_init)
    #draw_registration_result(source, target, np.identity(4))

    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    return source, target, source_down, target_down, source_fpfh, target_fpfh


def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    print(":: RANSAC registration on downsampled point clouds.")
    print("   Since the downsampling voxel size is %.3f," % voxel_size)
    print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
    return result


def main():


    voxel_size = 0.05  # means 5cm for the dataset
    source, target, source_down, target_down, source_fpfh, target_fpfh = \
            prepare_dataset(voxel_size)

    result = execute_global_registration(source_down, target_down,
                                               source_fpfh, target_fpfh,
                                               voxel_size)
    print(result)
    draw_registration_result(source, target,  result.transformation)

    rx = 0
    ry = 0
    rz = 20

    Rx = np.asarray([[ 1, 0, 0,                         0.01388155],
                    [0,  math.cos(rx),  -math.sin(rx), -0.00485227],
                    [ 0,   math.sin(rx), math.cos(rx), -0.59054986],
                    [ 0. ,         0. ,         0.,          1.]])

    Ry = np.asarray([[ math.cos(ry), 0, math.sin(ry),   0],
                    [0,  1,  0,                         0],
                    [ -math.sin(ry),   0, math.cos(ry), 0],
                    [ 0. ,         0. ,         0.,          1.]])

    Rz = np.asarray([[math.cos(rz),  -math.sin(rz), 0,      0],
                    [math.sin(rz), math.cos(rz),    0,    0],
                    [ 0,  0,  1,                           0],
                    [ 0. ,         0. ,         0.,          1.]])
    trans = np.matmul(Rx,Ry,Rz)

    transform = np.asarray([[  0.60246117,  0.0344336,  -0.79740508, -0.06299544],
                            [-0.34736023,  0.91080345 ,-0.2231097,  -0.22075178],
                            [ 0.71859683,  0.41140175 , 0.56068457, -0.4915422],
                            [ 0. ,         0. ,         0.,          1.,        ]])


    draw_registration_result(source, target,  transform)

    print(result.transformation)

main()