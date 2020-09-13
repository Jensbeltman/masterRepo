import os
import open3d as o3d

class SileaneCameraParams():
    def __init__(self, width=None, height=None, fu=None, fv=None, cu=None, cv=None, clip_start=None, clip_end=None, location=None, rotation=None):
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
        if any([v == None for v in [self.width,self.height,self.fu,self.fv,self.cu,self.cv]]):
            print("Intrinsic parameters where none no intrinsic object where created")
        else:
            return o3d.camera.camera.PinholeCameraIntrinsic(self.width,self.height,self.fu,self.fv,self.cu,self.cv)
    
    def get_extrinsic(self):
        if any([v == None for v in [self.location,self.rotation]]):
            print("Extrinsic parameters where none no extrinsic object where created")
        else:
            return o3d.camera.camera.PinholeCameraExtrinsic(self.location,self.rotation)
        
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

def get_sileane_camera_params(txtfile):
    f = open(txtfile,"r")
    lines = f.readlines()

    scp = SileaneCameraParams()
    scp.width = int(lines[0].split("\t")[1].replace("\n",""))
    scp.height = int(lines[1].split("\t")[1].replace("\n",""))
    scp.fu = float(lines[2].split("\t")[1].replace("\n",""))
    scp.fv = float(lines[3].split("\t")[1].replace("\n",""))
    scp.cu = float(lines[4].split("\t")[1].replace("\n",""))
    scp.cv = float(lines[5].split("\t")[1].replace("\n",""))
    scp.clip_start = float(lines[6].split("\t")[1].replace("\n",""))
    scp.clip_end = float(lines[7].split("\t")[1].replace("\n",""))
    scp.location = [float(v.replace("\n","")) for v in lines[8].split("\t")[1:4]]
    scp.rotation = [float(v.replace("\n","")) for v in lines[9].split("\t")[1:5]]
    
    print("Read sileane camera parameters:\n width {}\n height {}\n fu {}\n fv {}\n cu {}\n cv {}\n clip_start {}\n clip_end {}\n location {}\n rotation {}".format(scp.width,scp.height,scp.fu,scp.fv,scp.cu,scp.cv,scp.clip_start,scp.clip_end,scp.location,scp.rotation))

    return scp

def depth_to_pcd(intrinsic,image_file,output_file):
    depth_img = o3d.io.read_image(image_file)

    pcd = o3d.geometry.PointCloud.create_from_depth_image(depth_img,intrinsic,)

    o3d.io.write_point_cloud(output_file,pcd)
    
    print("Converted image {} to pointcloud {}".format(os.path.basename(image_file),os.path.basename(output_file)) )


def depth_to_pcd_sileane_dir(directory):
    depth_dir = directory+"/depth"
    depth_gt_dir = directory+"/depth_gt"
    intrinsic_file = directory+"/camera_params.txt"
    sileaneCameraParams = get_sileane_camera_params(intrinsic_file)

    for d in [depth_dir,depth_gt_dir]:
        if os.path.isdir(d):
            for filename in os.listdir(d):
                if filename.endswith(".PNG"):
                    image_filename = d+"/"+filename
                    pcd_filename = d+"/"+filename.split('.')[0]+".pcd"
                    depth_to_pcd(intrinsic,image_filename,pcd_filename)
        else:
            print("Directory {} not found".format(d))


class SileanObject(object):
    def __init__(self,dataset_path,object_name):
        self.__name__ = object_name
        self.__path = os.path.join(dataset_path,object_name)

        self.__depth_path = dirPath(os.path.join(self.__path,"depth"))
        self.__depth_gt_path = dirPath(os.path.join(self.__path,"depth_gt"))
        self.__segmentation_path = dirPath(os.path.join(self.__path,"segmentation"))
        self.__rgb_path = dirPath(os.path.join(self.__path,"rgb"))

        self.depth_paths = (p for p in self.image_paths(self.__depth_path))
        self.depth_gt_paths = (p for p in self.image_paths(self.__depth_gt_path))
        self.segmentation_paths = (p for p in self.image_paths(self.__segmentation_path))
        self.rgb_paths = (p for p in self.image_paths(self.__rgb_path))

        self.depth_images = (i for i in self.images(self.__depth_path))
        self.depth_gt_images = (i for i in self.images(self.__depth_gt_path))
        self.segmentation_images = (i for i in self.images(self.__segmentation_path))
        self.rgb_images = (i for i in self.images(self.__rgb_path))
        
        self.gt_poses = []
        self.camera_params = get_sileane_camera_params(os.path.join(self.__path,"camera_params.txt"))
        self.mesh = o3d.io.read_triangle_mesh(os.path.join(self.__path,"mesh.ply"))

    def image_paths(self,imagefolder_path):
        if imagefolder_path is None:
            print("No image paths found for object \"{}\" the requested information might not be available for this object type".format(self.__name__))
        else:
            for filename in os.listdir(imagefolder_path):
                if filename.endswith(".PNG"):
                    yield os.path.join(imagefolder_path,filename)

    def images(self,imagefolder_path):
        for image_path in self.image_paths(imagefolder_path):
            yield o3d.io.read_image(image_path)
    


class SileanDataset(object):
    def __init__(self,path):
        self.path = path
        self.brick = SileanObject(self.path,"brick")
        self.candlestick = SileanObject(self.path,"candlestick")
        self.gear = SileanObject(self.path,"gear")
        self.markers_bump = SileanObject(self.path,"markers_bump")
        self.markers_flat = SileanObject(self.path,"markers_flat")
        self.pepper = SileanObject(self.path,"pepper")
        self.bunny = SileanObject(self.path,"bunny")
        self.coffee_cup = SileanObject(self.path,"coffee_cup")
        self.juice = SileanObject(self.path,"juice")
        self.markers_clutter = SileanObject(self.path,"markers_clutter")
        self.markers_flat_simulation = SileanObject(self.path,"markers_flat_simulation")
        self.tless_20 = SileanObject(self.path,"tless_20")
        self.objects = [self.brick,self.candlestick,self.gear,self.markers_bump,self.markers_flat,self.pepper,self.bunny,self.coffee_cup,self.juice,self.markers_clutter,self.markers_flat_simulation,self.tless_20]

if __name__ == "__main__":
    sd = SileanDataset("/home/jens/masterData/Siléane-Dataset")

    print(sd.__dict__)
    for o in sd.objects:
        print(o.__name__)
        for p in o.depth_gt_paths:
            print(p)

  