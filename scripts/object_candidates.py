from copy import copy
from math import pi

import numpy as np
import open3d as o3d
import transforms3d as tf
from sileane import SileanDataset


def spherical_uniform_unitvector():
    phi = np.random.uniform(0, np.pi * 2)
    costheta = np.random.uniform(-1, 1)
    theta = np.arccos(costheta)
    x = np.sin(theta) * np.cos(phi)
    y = np.sin(theta) * np.sin(phi)
    z = np.cos(theta)
    return np.array((x, y, z))


def add_transform_noise(T, t_sd, r_sd):
    Tn = np.eye(4)
    tn = spherical_uniform_unitvector() * np.random.normal(0, t_sd)
    Rn = tf.axangles.axangle2mat(axis=spherical_uniform_unitvector(), angle=r_sd)
    Tn[0:3, 0:3] = Rn
    Tn[0:3, 3] = tn
    return T.dot(Tn)


def generate_object_candidates(gt_pose_list, n, t_sd, r_sd):
    random_idxs = np.random.randint(0, len(gt_pose_list), n)
    object_candidates = []
    for ri in random_idxs:
        object_candidates.append(add_transform_noise(gt_pose_list[ri], t_sd, r_sd))
    return object_candidates


def get_mesh_list(pose_list, mesh):
    return [copy(mesh).transform(p) for p in pose_list]


if __name__ == "__main__":
    sd = SileanDataset("/home/jens/masterData/Siléane-Dataset")
    obj = sd.bunny

    cp = obj.camera_params.getPinholeCameraParameters()
    depth = obj.depth_image(17)
    pcd = obj.pcd(17, True)

    gt_pose_list = obj.gt_pose_list(17, occlusion_limit=0.7)
    object_candidates = generate_object_candidates(gt_pose_list=gt_pose_list, n=40, t_sd=0.05, r_sd=pi / 4.0)
    meshes = get_mesh_list(object_candidates, obj.mesh)

    print(len(gt_pose_list), len(object_candidates))

    meshes.append(pcd)
    o3d.visualization.draw_geometries(meshes)
