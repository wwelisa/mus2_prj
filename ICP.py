import open3d as o3d
import numpy as np
import copy


def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    #source_temp.paint_uniform_color([1, 0.706, 0])
    #target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp],
                                      zoom=1.0,
                                      front=[0.9288, -0.2951, -0.2242],
                                      lookat=[1.6784, 2.0612, 1.4451],
                                      up=[-0.3402, -0.9189, -0.1996])


def visualize(mesh):
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(mesh)
    vis.run()
    vis.destroy_window()



def main():
    
    demo_icp_pcds = o3d.data.DemoICPPointClouds()
    source = o3d.io.read_point_cloud("models/shoeTop_removed.pcd")
    source = source.scale(0.9, center=source.get_center())
    target = o3d.io.read_point_cloud("models/shoeBottom_removed.pcd")
    threshold = 0.02
    trans_init = np.asarray([[  0.60246117,  0.0344336,  -0.79740508, -0.06299544],
                            [-0.34736023,  0.91080345 ,-0.2231097,  -0.22075178],
                            [ 0.71859683,  0.41140175 , 0.56068457, -0.4915422],
                            [ 0. ,         0. ,         0.,          1.,        ]])

    draw_registration_result(source, target, trans_init)

    
    print("Initial alignment")
    evaluation = o3d.pipelines.registration.evaluate_registration(
        source, target, threshold, trans_init)
    
    print(evaluation)

    print("Apply point-to-point ICP")
    reg_p2p = o3d.pipelines.registration.registration_icp(
                source, target, threshold, trans_init,
                o3d.pipelines.registration.TransformationEstimationPointToPoint())
    print(reg_p2p)

    print("Transformation is:")
    print(reg_p2p.transformation)
    draw_registration_result(source, target, reg_p2p.transformation)

    
main()
