
import open3d as o3d   #导入open3d
import numpy as np
pcd = o3d.io.read_point_cloud("cloud_bin_0.pcd")

xyz_load = np.asarray(pcd.points)
rgb_load = np.asarray(pcd.colors)
nxyz_load=np.asarray(pcd.normals)

# help(pcd)
print(xyz_load.shape)
print("------------")
print(rgb_load.shape)
print(nxyz_load.shape)

pcd2 = o3d.geometry.PointCloud()
pcd2.points = o3d.utility.Vector3dVector(xyz_load)
pcd2.colors = o3d.utility.Vector3dVector(rgb_load)

o3d.visualization.draw_geometries([pcd2])

# o3d.io.write_point_cloud("new.ply", pcd2)
