#include <iostream>
#include <string>
#include "dataset/SileaneDatasetObject.hpp"
#include "dataset/SileaneDataset.hpp"
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl-1.10/pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/viz.hpp>
#include <chronometer.h>
#include <ga/ga.hpp>
#include <dataset/pose_noise.hpp>
#include <ga/ga_functions.hpp>
#include <ga/utility/logging.hpp>
#include <ga/utility/vizualization.hpp>


int main() {
    Chronometer chronometer;
    SileaneData sileaneData("/home/jens/masterData/Siléane-Dataset");
    SileaneObjectPtr sileaneObject = std::static_pointer_cast<SileaneDatasetObject>(sileaneData.objects[1]);
    int sample_n = 4;
    std::cout << "Using sample " << sileaneObject->filenames[sample_n] << " from dataset object folder "
              << sileaneObject->name << std::endl;

    // DatasetObject mesh ply data
    std::shared_ptr<cv::viz::Mesh> meshptr = sileaneObject->get_mesh();

//    // DatasetObject sample point cloud
    PointCloudT::Ptr pc = sileaneObject->get_pcd(sample_n);
//    pcl::visualization::PCLVisualizer vis;
//    vis.addPointCloud(pc, "data");
//    vis.spin();
    // DatasetObject mesh pc data
    PointCloudT::Ptr pcm = sileaneObject->get_mesh_point_cloud();
    NormalCloudT::Ptr ncm = sileaneObject->get_mesh_normal_cloud();

    // DatasetObject candidates (GT)
    std::vector<T4> object_candidates = sileaneObject->get_object_candidates(sample_n);
    int n_gt_poses = object_candidates.size();

    // Add noise DatasetObject candidates
    PoseNoise poseNoise(0.001, 3.14 / 12);
    poseNoise.append_noisy_transforms(object_candidates, object_candidates.size());

    std::cout << object_candidates.size() << " candidates used" << endl;
    // Camera pose for the chosen dataset object
    T4 camera_pose = sileaneObject->sileaneCameraParams.T;

    // Initializing the genetic Evaluator
    chronometer.tic();
    std::shared_ptr<GeneticEvaluatorOC> geneticEvaluatorOCPtr = std::make_shared<GeneticEvaluatorOC>(sileaneObject,
                                                                                                     sample_n, 0.001);
    //std::shared_ptr<GeneticEvaluatorOC> geneticEvaluatorOCPtr = std::make_shared<GeneticEvaluatorOC>(object_candidates,
//                                                                                                     pc, pcm, ncm,
//                                                                                                     meshptr,
//                                                                                                     camera_pose,
//                                                                                                     0.001);
    std::cout << "GeneticEvaluatorOC init elapsed time: " << chronometer.toc() << "s\n";

    // GA object initilization, configuration and solution
    GA ga(object_candidates.size());
    ga.geneticEvaluatorPtr = geneticEvaluatorOCPtr;
    ga.N_chromosomes = 100;
    ga.generation_max = 50;
    ga.mutation_rate = 0.01;
    ga.elite_count = (int) (0.1 * ga.N_genes);
    ga.parent_pool_count = (int) (0.3 * ga.N_genes);
    ga.crossover = crossover_uniform;
    ga.mutation = mutation_flip;

    chronometer.tic();
    GAResult result = ga.solve();
    std::cout << "GA solve elapsed time: " << chronometer.toc() << "s\n";

    result_write(result, "/home/jens/masterRepo/data/ga_results.json");

    result_vis(&ga, std::dynamic_pointer_cast<DatasetObject>(sileaneObject));

//    for (double mr = 0.01; mr<0.5; mr+=0.01){
//        chronometer.tic();
//        ga.mutation_rate = mr;
//        GAResult res = ga.solve();
//        std::cout << "GA solve elapsed time: " <<  chronometer.toc() << "s\n";
//
//        nlohmann::json j = res;
//        ofstream out_file("/home/jens/masterRepo/data/mutation_rate/ga_results_mr_"+std::to_string(int(mr*100))+".json");
//        if (!out_file.is_open())
//            std::cout<<"Json file not opened"<<std::endl;
//        out_file<<j.dump();
//        out_file.close();
//    }



    return 0;
}
