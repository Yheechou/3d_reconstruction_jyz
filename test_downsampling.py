
import open3d as o3d   #导入open3d
import numpy as np

pcd = o3d.io.read_point_cloud("data/cloud_training1_1.ply")
print(pcd)
o3d.visualization.draw_geometries([pcd])


print("Downsample the point cloud with a voxel of 0.05")
downpcd = o3d.geometry.voxel_down_sample(pcd,voxel_size=0.01)
print(downpcd)
o3d.visualization.draw_geometries([downpcd])