
import open3d as o3d   #导入open3d
import numpy as np
pcd0 = o3d.io.read_point_cloud("cloud_bin_0.ply")
pcd1 = o3d.io.read_point_cloud("cloud_bin_1.ply")
pcd2 = o3d.io.read_point_cloud("cloud_bin_2.ply")

o3d.visualization.draw_geometries([pcd0])
o3d.visualization.draw_geometries([pcd1])
o3d.visualization.draw_geometries([pcd2])

# o3d.io.write_point_cloud("new.ply", pcd2)
