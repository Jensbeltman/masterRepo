import json
import os

import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
import transforms3d as tf


def rtT(rotation, translation):
    T = np.eye(4)
    T[0:3, 0:3] = tf.quaternions.quat2mat(rotation)
    T[0:3, 3] = translation
    return T


class SileaneCameraParams():
    def __init__(self, width=None, height=None, fu=None, fv=None, cu=None, cv=None, clip_start=None, clip_end=None,
                 location=None, rotation=None):
        self.width = width
        self.height = height
        self.fu = fu
        self.fv = fv
        self.cu = cu
        self.cv = cv
        self.clip_start = clip_start
        self.clip_end = clip_end
        self.location = location
        self.rotation = rotation

    def get_intrinsic(self):
        if any([v == None for v in [self.width, self.height, self.fu, self.fv, self.cu, self.cv]]):
            print("Intrinsic parameters where none no intrinsic object where created")
        else:
            return o3d.camera.PinholeCameraIntrinsic(self.width, self.height, self.fu, self.fv, self.cu, self.cv)

    def get_extrinsic(self):
        if any([v == None for v in [self.location, self.rotation]]):
            print("Extrinsic parameters where none no extrinsic object where created")
        else:
            return rtT(self.rotation, self.location)

    def getPinholeCameraParameters(self):
        pinholeCameraParameters = o3d.camera.PinholeCameraParameters()
        pinholeCameraParameters.intrinsic = self.get_intrinsic()
        pinholeCameraParameters.extrinsic = self.get_extrinsic()
        return pinholeCameraParameters


def dirPath(path):
    if os.path.isdir(path):
        return path
    else:
        return None


def get_sileane_camera_params(txtfile, verbose=False):
    f = open(txtfile, "r")
    lines = f.readlines()

    scp = SileaneCameraParams()
    scp.width = int(lines[0].split("\t")[1].replace("\n", ""))
    scp.height = int(lines[1].split("\t")[1].replace("\n", ""))
    scp.fu = float(lines[2].split("\t")[1].replace("\n", ""))
    scp.fv = float(lines[3].split("\t")[1].replace("\n", ""))
    scp.cu = float(lines[4].split("\t")[1].replace("\n", ""))
    scp.cv = float(lines[5].split("\t")[1].replace("\n", ""))
    scp.clip_start = float(lines[6].split("\t")[1].replace("\n", ""))
    scp.clip_end = float(lines[7].split("\t")[1].replace("\n", ""))
    scp.location = [float(v.replace("\n", "")) for v in lines[8].split("\t")[1:4]]
    scp.rotation = [float(v.replace("\n", "")) for v in lines[9].split("\t")[1:5]]
    scp.R = tf.quaternions.quat2mat(scp.rotation)
    scp.t = np.array(scp.location)
    scp.T = np.eye(4)
    scp.T[0:3, 0:3] = scp.R
    scp.T[0:3, 3] = scp.t

    if verbose:
        print(
            "Read sileane camera parameters:\n width {}\n height {}\n fu {}\n fv {}\n cu {}\n cv {}\n clip_start {}\n clip_end {}\n location {}\n rotation {}".format(
                scp.width, scp.height, scp.fu, scp.fv, scp.cu, scp.cv, scp.clip_start, scp.clip_end, scp.location,
                scp.rotation))

    return scp


class SileanObject(object):
    def __init__(self, dataset_path, object_name):
        self.__name__ = object_name
        self.__path = os.path.join(dataset_path, object_name)

        self.__depth_path = dirPath(os.path.join(self.__path, "depth"))
        self.__depth_gt_path = dirPath(os.path.join(self.__path, "depth_gt"))
        self.__segmentation_path = dirPath(os.path.join(self.__path, "segmentation"))
        self.__rgb_path = dirPath(os.path.join(self.__path, "rgb"))
        self.__gt_path = dirPath(os.path.join(self.__path, "gt"))

        self._filenames = [fn for fn in self.get_filenames(self.__depth_path)]
        self.nr_images = len(self._filenames)

        self.__depth_paths = (os.path.join(self.__depth_path, fn + ".PNG") for fn in self._filenames)
        self.__depth_gt_paths = (os.path.join(self.__depth_gt_path, fn + ".PNG") for fn in self._filenames)
        self.__segmentation_paths = (os.path.join(self.__segmentation_path, fn + ".PNG") for fn in self._filenames)
        self.__rgb_paths = (os.path.join(self.__rgb_path, fn + ".PNG") for fn in self._filenames)

        self.depth_images = (i for i in self.images(self.__depth_paths))
        self.depth_gt_images = (i for i in self.images(self.__depth_gt_paths))
        self.segmentation_images = (i for i in self.images(self.__segmentation_paths))
        self.rgb_images = (i for i in self.images(self.__rgb_paths))

        self.__gt_filenames = [filename for filename in os.listdir(self.__gt_path) if filename.endswith(".json")]
        self.camera_params = get_sileane_camera_params(os.path.join(self.__path, "camera_params.txt"))
        self.mesh = o3d.io.read_triangle_mesh(os.path.join(self.__path, "mesh.ply"))

    def get_filenames(self, imagefolder_path):
        if imagefolder_path is None:
            print("No filenames generated depth path not found for object \"{}\"".format(self.__name__))
        else:
            dirlist = [n for n in os.listdir(imagefolder_path)]
            dirlist.sort()
            for filename in dirlist:
                if filename.endswith(".PNG"):
                    yield os.path.splitext(filename)[0]

    def images(self, path_gen):
        for image_path in path_gen:
            yield mpimg.imread(image_path)

    def depth_image(self, n):
        return mpimg.imread(os.path.join(self.__depth_path, self._filenames[n] + ".PNG"))

    def depth_gt_image(self, n):
        return mpimg.imread(os.path.join(self.__depth_gt_path, self._filenames[n] + ".PNG"))

    def segmentation_image(self, n):
        return mpimg.imread(os.path.join(self.__segmentation_path, self._filenames[n] + ".PNG"))

    def rgb_image(self, n):
        return mpimg.imread(os.path.join(self.__rgb_path, self._filenames[n] + ".PNG"))

    def gt_pose_list_gen(self, occlusion_limit=1.0):
        for fn in self.__gt_filenames:
            list_of_dicts = json.loads(open(os.path.join(self.__gt_path, fn + ".json"), 'r').read())
            yield [np.vstack((np.hstack((np.array(d['R']), np.array(d['t']).reshape(3, 1))), np.array([0, 0, 0, 1])))
                   for d in list_of_dicts if d['occlusion_rate'] < occlusion_limit]

    def gt_pose_list(self, n, occlusion_limit=1.0):
        list_of_dicts = json.loads(open(os.path.join(self.__gt_path, self._filenames[n] + ".json"), 'r').read())
        return [np.vstack((np.hstack((np.array(d['R']), np.array(d['t']).reshape(3, 1))), np.array([0, 0, 0, 1]))) for d
                in list_of_dicts if d['occlusion_rate'] < occlusion_limit]

    def pcd(self, n, gt=False):
        pcd = o3d.geometry.PointCloud()
        depth = None
        if gt:
            depth = self.depth_gt_image(n)
        else:
            depth = self.depth_image(n)

        depth_range = self.camera_params.clip_end - self.camera_params.clip_start
        z = self.camera_params.clip_start + depth_range * depth

        c, r = np.meshgrid(np.arange(self.camera_params.width), np.arange(self.camera_params.height), sparse=True)
        valid = (depth < 1.0)
        x = np.where(valid, z * (c - self.camera_params.cu) / self.camera_params.fu, 0)
        y = np.where(valid, z * (r - self.camera_params.cv) / self.camera_params.fv, 0)
        pcd.points = o3d.utility.Vector3dVector(np.dstack((x, y, z)).reshape(-1, 3))
        pcd.transform(self.camera_params.T)
        return pcd


class SileanDataset(object):
    def __init__(self, path):
        self.path = path
        self.brick = SileanObject(self.path, "brick")
        self.candlestick = SileanObject(self.path, "candlestick")
        self.gear = SileanObject(self.path, "gear")
        self.markers_bump = SileanObject(self.path, "markers_bump")
        self.markers_flat = SileanObject(self.path, "markers_flat")
        self.pepper = SileanObject(self.path, "pepper")
        self.bunny = SileanObject(self.path, "bunny")
        self.coffee_cup = SileanObject(self.path, "coffee_cup")
        self.juice = SileanObject(self.path, "juice")
        self.markers_clutter = SileanObject(self.path, "markers_clutter")
        self.markers_flat_simulation = SileanObject(self.path, "markers_flat_simulation")
        self.tless_20 = SileanObject(self.path, "tless_20")
        self.objects = [self.brick, self.candlestick, self.gear, self.markers_bump, self.markers_flat, self.pepper,
                        self.bunny, self.coffee_cup, self.juice, self.markers_clutter, self.markers_flat_simulation,
                        self.tless_20]


if __name__ == "__main__":
    sd = SileanDataset("/home/jens/masterData/Siléane-Dataset")

    image = sd.markers_flat.depth_image(17)
    ##o3d.visualization.draw_geometries((image))
    plt.imshow(image, cmap='gray')
    plt.show()
