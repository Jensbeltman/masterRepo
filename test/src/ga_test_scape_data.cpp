#include <iostream>
#include <string>
#include "dataset/scape/ScapeDatasetObject.hpp"
#include "dataset/scape/ScapeDataset.hpp"
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


    ScapeDataset scapeData("/home/jens/masterData/ScapeDataset/Scape/Full Dataset",
                           "/home/jens/masterData/ScapeDataset/Data from Scape Recognition");

    ScapeDatasetObjectPtr scapeObject = std::static_pointer_cast<ScapeDatasetObject>(
            scapeData.get_object_by_name("Ears"));


    // DatasetObject sample point cloud
    int sample_n = 2;
    std::cout << "Testing zone " << sample_n << " with related point cloud data filename "
              << scapeObject->filenames[scapeObject->zones[sample_n].pc_filename_idx] << " from dataset object folder "
              << scapeObject->name << "\n\n";
    PointCloudT::Ptr pc = scapeObject->get_pcd(sample_n);


    // DatasetObject candidates (GT)
    std::vector<T4> object_candidates = scapeObject->get_object_candidates(sample_n);
    std::cout << object_candidates.size() << " candidates used" << endl;


    // Initializing the genetic Evaluator
    chronometer.tic();
    std::shared_ptr<GeneticEvaluatorOC> geneticEvaluatorOCPtr = std::make_shared<GeneticEvaluatorOC>(scapeObject,
                                                                                                     sample_n, 1);
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

    result_vis(&ga, std::dynamic_pointer_cast<DatasetObject>(scapeObject));

    return 0;
}
