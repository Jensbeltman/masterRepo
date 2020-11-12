#include <iostream>
#include <string>
#include "dataset/scape/ScapeDatasetObject.hpp"
#include "dataset/scape/ScapeDataset.hpp"
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <pcl/filters/extract_indices.h>
#include <opencv4/opencv2/core.hpp>
#include <chronometer.h>
#include <fcl/fcl.h>
#include <ga/ga.hpp>
#include <ga/ga_functions.hpp>
#include <ga/utility/logging.hpp>
#include <ga/utility/visualization.hpp>

std::ostream &operator<<(std::ostream &os, const std::vector<double>& vectord){
    for(int i = 0; i<vectord.size()-1;i++)
        os<<vectord[i]<<", ";
    os<<vectord.back();
    return os;
}

int main(int argc, char** argv) {
    int sample_n = std::stoi(argv[1]);
    std::string object_name = argv[2];
    Chronometer chronometer;

    ScapeDataset scapeData("/home/jens/masterData/ScapeDataset/Scape/Full Dataset",
                           "/home/jens/masterData/ScapeDataset/Data from Scape Recognition",false);


    DatasetObjectPtr datasetObject = scapeData.get_object_by_name(object_name);
    ScapeDatasetObjectPtr scapeObject = std::dynamic_pointer_cast<ScapeDatasetObject>(datasetObject);

    // DatasetObject sample point cloud
    chronometer.tic();
    // Initializing the genetic Evaluator
    std::shared_ptr<GeneticEvaluatorOC> geneticEvaluatorOCPtr = std::make_shared<GeneticEvaluatorOC>(datasetObject, sample_n, 1);
    std::cout << "GeneticEvaluatorOC init elapsed time: " << chronometer.toc() << "s\n";

    std::cout << "Testing zone " << sample_n << " with related point cloud data filename "
              << scapeObject->pcd_filenames[scapeObject->scape_data_points[sample_n].pc_filename_idx] << " from dataset object folder "
              << scapeObject->name << "\n\n";
    PointCloudT::Ptr pc = scapeObject->get_pcd(sample_n);


    // DatasetObject candidates (GT)
    std::vector<T4> object_candidates = scapeObject->get_object_candidates(sample_n);
    std::cout << object_candidates.size() << " candidates used" << endl;

    // GA object initilization, configuration and solution
    GA ga(object_candidates.size());
    ga.geneticEvaluatorPtr = geneticEvaluatorOCPtr;
    ga.population_size = 100;
    ga.generation_max = 50;
    ga.mutation_rate = 0.01;
    ga.elite_pct = 0.1;
    ga.parent_pool_pct = 0.3;
    ga.crossover = crossover_uniform;
    ga.mutation = mutation_flip;

    chronometer.tic();
    GAResult result = ga.solve();
    std::cout << "GA solve elapsed time: " << chronometer.toc() << "s\n";
    std::cout << "GA Result:\n"<<result<<std::endl;
    std::cout << "Ocs scores "<<scapeObject->get_scores(sample_n)<<std::endl;
    result_write(result, "/home/jens/masterRepo/data/ga_results.json");

    CustomVisualizer vis;

    pcl::ExtractIndices<PointT> extractIndices;
    for (int i = 0; i < geneticEvaluatorOCPtr->object_candidates.size(); i++) {
        std::string id = "oc_" + std::to_string(i);
        PointCloudT::Ptr ocpc(new PointCloudT);
        extractIndices.setInputCloud(geneticEvaluatorOCPtr->pcm);
        extractIndices.setIndices(geneticEvaluatorOCPtr->oc_visible_pt_idxs[i]);
        extractIndices.filter(*ocpc);
        pcl::transformPointCloud(*ocpc, *ocpc, geneticEvaluatorOCPtr->object_candidates[i]);

        if (ga.result.best_chromosome[i]) {
            vis.addPointCloud(ocpc, id, "accepted", 0, 255, 0);
        } else {
            vis.addPointCloud(ocpc, id, "rejected", 255, 0, 0);
        }
    }

    vis.addPointCloud(geneticEvaluatorOCPtr->pc, "pc");
    PointCloudT::Ptr mesh_pc = geneticEvaluatorOCPtr->pcm;

    vis.spin();

    return 0;
}
