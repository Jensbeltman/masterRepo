import open3d as o3d 
import numpy as np
from sileane import SileanDataset, SileanObject, SileaneCameraParams

if __name__ == "__main__":
    sd = SileanDataset("/home/jens/masterData/Siléane-Dataset")

    print(sd.__dict__)
    for o in sd.objects:
        print(o.__name__)
        for p in o.depth_gt_paths:
            print(p)

  