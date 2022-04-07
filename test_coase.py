
import open3d as o3d
import numpy as np
import copy
from time import *
import math

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    #source_temp.paint_uniform_color([1, 0.706, 0])
    #target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])

def preprocess_point_cloud(pcd, voxel_size):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = o3d.geometry.voxel_down_sample(pcd, voxel_size)

    radius_normal = voxel_size * 2
    # print(":: Estimate normal with search radius %.3f." % radius_normal)
    o3d.geometry.estimate_normals(pcd_down, o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    # print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh


def prepare_dataset(voxel_size, theta, souce_path, target_path):
    print(":: Load two point clouds and disturb initial pose.")
    source = o3d.io.read_point_cloud(souce_path)
    target = o3d.io.read_point_cloud(target_path)
    trans_init = np.asarray([[np.cos(theta), 0.0, np.sin(theta), 0.0], 
                             [0.0, 1.0, 0.0, 0.0],
                             [-np.sin(theta), 0.0, np.cos(theta), 0.0], 
                             [0.0, 0.0, 0.0, 1.0]])
    source.transform(trans_init)
    # draw_registration_result(source, target, np.identity(4))

    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    return source, target, source_down, target_down, source_fpfh, target_fpfh

def execute_global_registration(source_down, target_down, source_fpfh,
                               target_fpfh, voxel_size):
   distance_threshold = voxel_size * 1.5
   # print(":: RANSAC registration on downsampled point clouds.")
   # print("   Since the downsampling voxel size is %.3f," % voxel_size)
   # print("   we use a liberal distance threshold %.3f." % distance_threshold)
   result = o3d.registration.registration_ransac_based_on_feature_matching(
       source_down, target_down, source_fpfh, target_fpfh, distance_threshold,
       o3d.registration.TransformationEstimationPointToPoint(False), 4, [
           o3d.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
           o3d.registration.CorrespondenceCheckerBasedOnDistance(
               distance_threshold)
       ], o3d.registration.RANSACConvergenceCriteria(4000000, 500))
   return result

if __name__ == "__main__":

    source_path = "to_yunqi_offline_data/info_room_ply/1_room_2.ply"
    target_path = "to_yunqi_offline_data/info_room_ply/0_room_0.ply"

    voxel_size = 0.1 # means 5cm for this dataset
    source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(voxel_size, math.pi*0, source_path, target_path)

    begin_time = time() # time consumption

    result_ransac = execute_global_registration(source_down, target_down,
                                            source_fpfh, target_fpfh,
                                            voxel_size)

    end_time = time()
    print(end_time-begin_time)

    print(result_ransac)
    draw_registration_result(source, target, np.identity(4))
    draw_registration_result(source, target, result_ransac.transformation)

