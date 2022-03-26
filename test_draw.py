
import open3d as o3d   #导入open3d
import numpy as np

pcd0 = o3d.io.read_point_cloud("cloud_traning1_1.ply")
pcd1 = o3d.io.read_point_cloud("cloud_traning1_2.ply")
pcd2 = o3d.io.read_point_cloud("cloud_bin_2.ply")

print(pcd0)
print("pcd0的形状为")
temp = np.asarray(pcd0.points)
print(temp.shape)
print("pcd1的形状为")
temp = np.asarray(pcd1.points)
print(temp.shape)
print("pcd2的形状为")
temp = np.asarray(pcd2.points)
print(temp.shape)

o3d.visualization.draw_geometries([pcd0])
o3d.visualization.draw_geometries([pcd1])
o3d.visualization.draw_geometries([pcd2])


