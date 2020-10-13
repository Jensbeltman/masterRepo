import open3d as o3d
import numpy as np
from sileane import SileanDataset
import matplotlib.pyplot as plt
from copy import copy
from geneticalgorithm import geneticalgorithm as ga
from time import time
from geneticalgorithm.geneticalgorithm import geneticalgorithm

if __name__ == "__main__":
    sd = SileanDataset("/home/jens/masterData/Siléane-Dataset")
    obj = sd.bunny
    voxel_size = 0.001
    occlusion_limit = 1.0

    testImageNr = obj._filenames.index("bunny_3_032")
    print(obj._filenames[testImageNr])

    algorithm_param = {'max_num_iteration': 100, \
                       'population_size': 100, \
                       'mutation_probability': 0.005, \
                       'elit_ratio': 0.1, \
                       'crossover_probability': 0.5, \
                       'parents_portion': 0.3, \
                       'crossover_type': 'uniform', \
                       'max_iteration_without_improv': None}

    source = obj.mesh.sample_points_uniformly(100000, use_triangle_normal=True).voxel_down_sample(voxel_size)
    target = obj.pcd(testImageNr, gt=True).voxel_down_sample(voxel_size)

    # plane_model, inliers = target.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=2000)
    # target =  target.select_by_index(inliers,invert=True)

    gt_pose_list = obj.gt_pose_list(testImageNr, occlusion_limit=occlusion_limit)
    print(
        f'Object "{obj.__name__}" depth image {testImageNr} has {len(gt_pose_list)} gt poses with less than {occlusion_limit} occlusion')

    object_candidates = gt_pose_list  # + generate_object_candidates(gt_pose_list=gt_pose_list,n=(n:=100), t_sd=(t_sd:=0.01), r_sd=(r_sd:=pi/4.0))
    nroc = len(object_candidates)
    # print(f'Generated {n} noisy object candidates with noise normal dist. translational and rotational sd {t_sd} and {r_sd}')

    object_candidates_pc = []
    inliers_source = []
    cameraT = obj.camera_params.T
    cameraZ = cameraT[0:3, 2].reshape(3, 1)
    startTime = time()
    nrSourcePts = []
    for i, oc in enumerate(object_candidates):  # generating list inliers souce points for each object candidate
        oc_pc = copy(source)
        oc_pc.transform(oc)

        visibleIdxs = np.argwhere(np.asarray(oc_pc.normals).dot(cameraZ) < 0)
        nrSourcePts.append(visibleIdxs.shape[0])
        oc_pc = oc_pc.select_by_index(visibleIdxs.ravel())

        oc_pc.paint_uniform_color(np.array((0, 0, 1)))
        object_candidates_pc.append(oc_pc)
        evaluation = o3d.registration.evaluate_registration(source, target,
                                                            max_correspondence_distance=1.1 * voxel_size,
                                                            transformation=oc)
        inliers_source.append(np.asarray(evaluation.correspondence_set)[:, 1].ravel())
    stopTime = time()
    print("Time spent on prep {} seconds".format(stopTime - startTime))
    times = []


    def fitness_func(X):
        st = time()
        activeIdxs = np.argwhere(X == 1)
        inliers = np.hstack([inliers_source[i] for i in activeIdxs.ravel()])
        nrInliers = inliers.shape[0]
        nrUniqueInliers = np.unique(inliers).shape[0]

        result = -(nrUniqueInliers / nrInliers) - (nrUniqueInliers) / np.sum(
            np.asarray(nrSourcePts)[activeIdxs.ravel()])
        et = time()
        times.append(et - st)
        return result


    startTime = time()
    model = geneticalgorithm(function=fitness_func, dimension=len(object_candidates), variable_type='bool',
                             algorithm_parameters=algorithm_param)
    ax = model.run()
    stopTime = time()
    solution = model.output_dict

    print(f'GA took {stopTime - startTime} seconds')
    print(f'Time spend on fitness {sum(times)}')
    print(
        f'Found solution \n\tGT\t{list(np.asarray(solution["variable"], dtype=int))[0:len(gt_pose_list)]}\n\tNoise\t{list(np.asarray(solution["variable"], dtype=int))[len(gt_pose_list):-1]}')

    plt.axes(ax)
    plt.show(block=False)
    plt.pause(.001)
    solution_ocpc = [ocpc for i, ocpc in enumerate(object_candidates_pc) if solution['variable'][i]]
    # o3d.visualization.draw_geometries(solution_ocpc)

    solution_ocpc += [target]
    o3d.visualization.draw_geometries(solution_ocpc)
