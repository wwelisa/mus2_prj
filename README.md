# mus2_prj

This repository contains the scripts to complete the second part of the mus2 *Project 1: Object Reconstruction with a Mobile Phone* . 

## Usage
A 3D model in the .obj format should be placed in the models/WIP folder.
Next the script **PreProcess.py** should be used to 
1.	Convert the .obj to a .pcd 
2.	Down sample the point cloud
3.	Change the size of the object to match the other one

To read in the right files the ```o3d.io.read_triangle_mesh()``` and ```o3d.io.read_point_cloud()``` need the correct input parameters.
In ```o3d.io.write_point_cloud()``` the name of the output file should be set.

**ransac.py** removes the ground plane of the .pcd. Again, the right input and output files need to be set.

**globalRegistration.py** is not strictly necessary, it creates a initial transformation matrix to use for ICP.
If the naming convention was kept nothing needs to be changed here. The script prints a transformation to use for ICP.

**ICP.py** , if the naming convention was kept nothing needs to be changed here. The script outputs creates a new transformation matrix to match the two point clouds.


## Used Resources

http://www.open3d.org/docs/release/python_api/open3d.io.read_triangle_mesh.html

http://www.open3d.org/docs/release/python_api/open3d.io.read_point_cloud.html

http://www.open3d.org/docs/release/tutorial/pipelines/global_registration.html

http://www.open3d.org/docs/release/tutorial/pipelines/icp_registration.html

