
import open3d as o3d   #导入open3d
import numpy as np

pcd = o3d.io.read_point_cloud("/Users/apple/Desktop/3D_indoor/project2/cloud_bin_0.ply")
print(pcd)
o3d.visualization.draw_geometries([pcd])


print("Downsample the point cloud with a voxel of 0.05")
downpcd = o3d.geometry.voxel_down_sample(pcd,voxel_size=0.05)
print(downpcd)
o3d.visualization.draw_geometries([downpcd])