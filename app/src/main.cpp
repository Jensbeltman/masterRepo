#include <iostream>
#include <string>
#include "dataset/SileaneObject.hpp"
#include "dataset/SileaneDataset.hpp"
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl-1.10/pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/viz.hpp>
#include <chronometer.h>
#include <ga/ga.hpp>
#include <ga/utility/pose_noise.hpp>
#include <ga/ga_functions.hpp>


int main() {
    Chronometer chronometer;
    SileaneData sileaneData("/home/jens/masterData/Siléane-Dataset");
    SileaneObjectPtr sileaneObject = std::static_pointer_cast<SileaneObject>(sileaneData.objects[1]);
    int sample_n = 4;
    std::cout << "Using sample " << sileaneObject->filenames[sample_n] << " from dataset object folder "
              << sileaneObject->name << std::endl;

    // DatasetObject mesh ply data
    std::shared_ptr<cv::viz::Mesh> meshptr = sileaneObject->get_mesh();

    // DatasetObject sample point cloud
    PointCloudT::Ptr pc = sileaneObject->get_pcd(sample_n);

    // DatasetObject mesh pc data
    PointCloudT::Ptr pcm = sileaneObject->get_mesh_point_cloud();
    NormalCloudT::Ptr ncm = sileaneObject->get_mesh_normal_cloud();

    // DatasetObject candidates (GT)
    std::vector<T4> object_candidates = sileaneObject->get_gt_poses(sample_n);
    int n_gt_poses = object_candidates.size();

    // Add noise DatasetObject candidates
    PoseNoise poseNoise(0.001, 3.14 / 12);
    poseNoise.append_noisy_transforms(object_candidates, object_candidates.size());

    std::cout << object_candidates.size() << " candidates used" << endl;
    // Camera pose for the chosen dataset object
    T4 camera_pose = sileaneObject->camera_params.T;

    // Initializing the genetic Evaluator
    chronometer.tic();
    std::shared_ptr<GeneticEvaluatorOC> geneticEvaluatorOCPtr = std::make_shared<GeneticEvaluatorOC>(object_candidates,
                                                                                                     pc, pcm, ncm,
                                                                                                     meshptr,
                                                                                                     camera_pose,
                                                                                                     0.001);
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

    // Solution visualization
    pcl::visualization::PCLVisualizer vis_solution;
    vis_solution.addPointCloud(pc, "data");
    PointCloudT::Ptr mesh_pc = sileaneObject->get_mesh_point_cloud();

    pcl::ExtractIndices<PointT> extractIndices;

    nlohmann::json j = result;
    ofstream out_file("/home/jens/masterRepo/data/ga_results.json");
    if (!out_file.is_open())
        std::cout << "Json file not opened" << std::endl;
    out_file << j.dump();
    out_file.close();

    for (int i = 0; i < object_candidates.size(); i++) {
        if (result.best_chromosome[i]) {
            PointCloudT::Ptr ocpc(new PointCloudT);
            extractIndices.setInputCloud(pcm);
            extractIndices.setIndices(geneticEvaluatorOCPtr->oc_visible_pt_idxs[i]);
            extractIndices.filter(*ocpc);
            pcl::transformPointCloud(*ocpc, *ocpc, object_candidates[i]);
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(ocpc, 0, 255, 0);
            vis_solution.addPointCloud<pcl::PointXYZ>(ocpc, color, "oc_" + std::to_string(i));
            // Todo add camera

            //cout<<"Visible inliers pts for oc "<<i<<" "<<costFunction->oc_visible_pt_idxs[i]->size()<<" "<<costFunction->oc_visible_inlier_pt_idxs[i]->size()<<endl;

        }
    }

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

    vis_solution.spin();

    return 0;
}
