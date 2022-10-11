import cv2
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
import copy
import os
import csv
from tqdm import tqdm
from sklearn.neighbors import NearestNeighbors

pcd_list = []
pcd_down_list = []
trans_list = []


def depth_image_to_point_cloud_o3d(path_rgb, path_depth, intrinsic):
    if os.path.isfile(path_rgb):
        rgb_raw = o3d.io.read_image(path_rgb)
        depth_raw = o3d.io.read_image(path_depth)
        depth_raw = np.asarray(depth_raw, dtype=np.float32)
        depth_raw = depth_raw/255*10*1000
        depth_raw = o3d.geometry.Image(depth_raw)
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
            rgb_raw, depth_raw, convert_rgb_to_intensity=False)
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
            rgbd_image, intrinsic)
        # (x, -y, -z)
        pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0],
                      [0, 0, -1, 0], [0, 0, 0, 1]])
        # remove ceiling
        pcd = pcd.select_by_index(
            np.where(np.asarray(pcd.points)[:, 1] < 0.5)[0])
        pcd_list.append(pcd)


def depth_image_to_point_cloud_own(path_rgb, path_depth, width, height, fx, fy, cx, cy, depth_scale):

    points_list = []
    colors_list = []
    rgb_raw = cv2.imread(path_rgb, cv2.IMREAD_COLOR)
    rgb_raw = cv2.cvtColor(rgb_raw, cv2.COLOR_BGR2RGB)
    depth_raw = cv2.imread(path_depth, cv2.IMREAD_GRAYSCALE)
    depth_raw = depth_raw/255*10*depth_scale
    pcd = o3d.geometry.PointCloud()
    for v in range(height):
        for u in range(width):
            z = depth_raw[v, u]/depth_scale
            if z > 0:  # if z=0,may see same black point in the trajectory
                x = (u - cx) * z / fx
                y = (v - cy) * z / fy
                points_list.append([x, y, z])
                colors_list.append(rgb_raw[v, u]/255)

    pcd.points = o3d.utility.Vector3dVector(np.asarray(points_list))
    pcd.colors = o3d.utility.Vector3dVector(np.asarray(colors_list))
    # (x,-y,-z)
    pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0],
                   [0, 0, -1, 0], [0, 0, 0, 1]])
    # remove ceiling
    pcd = pcd.select_by_index(
        np.where(np.asarray(pcd.points)[:, 1] < 0.5)[0])
    pcd_list.append(pcd)


def prepare_dataset(source, target, voxel_size):
    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    return source, target, source_down, target_down, source_fpfh, target_fpfh


def preprocess_point_cloud(pcd, voxel_size):
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh


def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
    distance_threshold = voxel_size*1.5  # 1.5
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


def local_icp_algorithm_o3d(source, target, init_trans, voxel_size):
    '''
    origin name is refine_registration
    '''
    distance_threshold = voxel_size * 0.4
    result = o3d.pipelines.registration.registration_icp(
        source, target, distance_threshold, init_trans,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    return result


def create_traj_line(num):
    lines = np.zeros((num-1, 2), dtype=int)
    for i in range(num-1):
        lines[i, :] = [i, i+1]
    return lines


def get_GT_traj(GT_path):
    GT_traj = []
    with open(GT_path, newline='') as csvfile:
        rows = csv.reader(csvfile)
        for row in rows:
            GT_traj.append(row)

    center = GT_traj[0]
    final_GT_traj = []
    for i in range(len(GT_traj)):
        x = ((float(GT_traj[i][0])-float(center[0])))
        y = ((float(GT_traj[i][1])-float(center[1])))
        z = ((float(GT_traj[i][2])-float(center[2])))
        final_GT_traj.append([x, y, z])

    GT_lines = create_traj_line(len(final_GT_traj))

    return final_GT_traj, GT_lines


def best_fit_transform(p, q):
    '''
    reference prove link: https://zhuanlan.zhihu.com/p/107218828?utm_id=0&fbclid=IwAR01v2O0yjiI7-cQI7cNL_Rari89D1m2C_J59AMUed0ivesceEmNRKbBAEM
    reference code link: https://www.796t.com/content/1548524898.html?fbclid=IwAR23KWasUAWxo_If4McOL2vCJZWP8mr_zs0BIP7UbquPS20biyBLWmaIRBY
    p is (N,3)
    q is (M,3)
    '''
    mean_p = np.mean(p, axis=0)  # (N,3)
    mean_q = np.mean(q, axis=0)  # (M,3)
    x = p - mean_p
    y = q - mean_q
    # because tr(ABC)=tr(CAB)=tr(BCA) => tr((Y.T)RX)= tr(RX(Y.T)), make S = X(Y.T)
    # need attention shape
    S = np.dot(x.T, y)

    # do SVD
    U, sigma, VT = np.linalg.svd(S)
    R = np.dot(VT.T, U.T)  # get rotation matrix

    # special reflection case
    if np.linalg.det(R) < 0:
        VT[:, 2] *= -1
        R = np.dot(VT.T, U.T)

    t = mean_q - np.dot(R, mean_p.T).T  # get translate matrix (1,3)

    trans = np.zeros((4, 4))        # homogeneous
    trans[3, 3] = 1
    trans[0:3, 0:3] = R
    trans[0:3, 3] = t

    return trans


def local_icp_algorithm_own(source, target, init_trans, voxel_size, max_iteration):
    source_points = np.ones(
        (np.asarray(source.points).shape[0], 4))   # shape = (n,4)
    target_points = np.ones(
        (np.asarray(target.points).shape[0], 4))   # shape = (m,4)
    source_points[:, 0:3] = np.asarray(source_down.points)
    target_points[:, 0:3] = np.asarray(target_down.points)

    threshold = voxel_size  # *0.4
    source_origin_points = copy.deepcopy(source_points)
    #source_points = copy.deepcopy(source)
    #target_points = copy.deepcopy(target)
    source_points = np.dot(init_trans, source_points.T)
    source_points = source_points.T

    prev_error = 0
    for i in tqdm(range(max_iteration)):
        # find the nearest neighbors index and distance between the current source and target points

        neigh = NearestNeighbors(
            n_neighbors=1, radius=threshold, algorithm='auto')
        neigh.fit(target_points[:, 0:3])
        NN_dist, NN_index = neigh.kneighbors(source_points[:, 0:3])
        NN_index = NN_index.reshape(-1)
        NN_dist = NN_dist.reshape(-1)
        valid = NN_dist < threshold
        source_points = source_points[valid]
        target_points = target_points[NN_index]
        target_points = target_points[valid]
        source_origin_points = source_origin_points[valid]

        # compute the transformation between the current source and nearest target points
        trans = best_fit_transform(
            source_points[:, 0:3], target_points[:, 0:3])
        # update the current source
        source_points = np.dot(trans, source_points.T).T

        # check error
        mean_error = np.sum(NN_dist) / len(NN_dist)
        if abs(prev_error - mean_error) < 0.0001:
            print(i)
            break
        prev_error = mean_error

    trans = best_fit_transform(
        source_origin_points[:, 0:3], source_points[:, 0:3])

    return trans


if __name__ == "__main__":
    floor = "floor2"
    width = 512
    height = 512
    fx = 256
    fy = 256
    cx = 256
    cy = 256
    depth_scale = 1000  # 1000
    intrinsic = o3d.camera.PinholeCameraIntrinsic(
        width=512, height=512, fx=256, fy=256, cx=256, cy=256)
    voxel_size = 0.05
    trans_o3d = np.identity(4)
    trans_list.append(trans_o3d)

    trans_by_o3d = []
    trans_by_o3d.append(np.identity(4))

    # get length of file in rgb
    DIR = "task2_data/"+floor+"/rgb"
    size = len([name for name in os.listdir(DIR) if os.path.isfile(
        os.path.join(DIR, name))])

    print("read file to get pcd")
    for i in tqdm(range(size)):
        path_rgb = "task2_data/"+floor+"/rgb/rgb_"+str(i)+".png"
        path_depth = "task2_data/"+floor+"/depth/depth_"+str(i)+".png"
        #depth_image_to_point_cloud_o3d(path_rgb, path_depth, intrinsic)
        depth_image_to_point_cloud_own(
            path_rgb, path_depth, width, height, fx, fy, cx, cy, depth_scale)

    print("o3d icp to get transformation")
    # for i in tqdm(range(size)):
    for i in range(size):
        print(i, "th get transformation")
        if i > 0:
            source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(
                pcd_list[i], pcd_list[i-1], voxel_size)
            result_ransac = execute_global_registration(source_down, target_down,
                                                        source_fpfh, target_fpfh,
                                                        voxel_size)

            '''
            o3d local icp algorithm
            '''
            # result_icp = local_icp_algorithm_o3d(
            #    source_down, target_down, result_ransac.transformation, voxel_size)
            # trans_list.append(result_icp.transformation)

            '''
            my own local icp algrithm
            '''

            trans = local_icp_algorithm_own(
                source_down, target_down, result_ransac.transformation, voxel_size, 50)
            trans_list.append(trans)

            if i == 1:
                pcd_down_list.append(target_down)
                pcd_down_list.append(source_down)
            else:
                pcd_down_list.append(source_down)

    # add all pcd together
    for i in range(1, len(trans_list)):
        trans_list[i] = trans_list[i-1] @ trans_list[i]

    estimated_traj = []
    print("add all pcd together to get final pcd and estimated trajectory")
    for i in tqdm(range(len(trans_list))):
        if i == 0:
            final_pcd = pcd_down_list[i]
            #final_pcd = pcd_list[i]
        else:
            final_pcd = final_pcd + \
                pcd_down_list[i].transform(trans_list[i])
            # final_pcd = final_pcd + \
            #    pcd_list[i].transform(trans_list[i])
        estimated_traj.append(trans_list[i][0:3, 3])

    estimated_lines = create_traj_line(len(estimated_traj))
    estimated_colors = [[1, 0, 0] for i in range(len(estimated_lines))]
    estimated_line_set = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(estimated_traj),
        lines=o3d.utility.Vector2iVector(estimated_lines),
    )
    estimated_line_set.colors = o3d.utility.Vector3dVector(estimated_colors)

    GT_path = 'task2_data/'+floor+'/GT/GT.csv'
    GT_traj, GT_lines = get_GT_traj(GT_path)
    GT_colors = [[0, 0, 0] for i in range(len(GT_lines))]
    GT_line_set = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(GT_traj),
        lines=o3d.utility.Vector2iVector(GT_lines),
    )
    GT_line_set.colors = o3d.utility.Vector3dVector(GT_colors)

    L2_dis = np.mean(np.linalg.norm(
        np.array(estimated_traj) - np.array(GT_traj), axis=1))
    print("mean of L2 distance between estimated_traj and GT_traj:", L2_dis)

    o3d.visualization.draw_geometries(
        [final_pcd, GT_line_set, estimated_line_set])
    print("end")
