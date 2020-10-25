#include <chronometer.h>
#include <dataset/sileane/SileaneDataset.hpp>
#include <dataset/scape/ScapeDataset.hpp>
#include <vector>
#include <utility>
#include <memory>
#include <opencv2/viz.hpp>
#include <fcl/fcl.h>
#include <dataset/vizualization.hpp>
#include <ga/collision/collision_checking.hpp>


int main(){
    Chronometer chronometer;

    ScapeDataset scapeData("/home/jens/masterData/ScapeDataset/Scape/Full Dataset",
                           "/home/jens/masterData/ScapeDataset/Data from Scape Recognition",false);
    DatasetObjectPtr datasetObject = scapeData.get_object_by_name("Ears");
    ScapeDatasetObjectPtr scapeObject = std::dynamic_pointer_cast<ScapeDatasetObject>(datasetObject);

    std::shared_ptr<cv::viz::Mesh> meshPtr = scapeObject->get_mesh();
    std::vector<T4> object_candidates = scapeObject->get_object_candidates(3);

    std::vector<std::pair<int,int>> collisions = get_collisions(object_candidates,meshPtr);
    std::cout<<collisions;

    pcl::visualization::PCLVisualizer::Ptr vis = vis_pc_and_oc(datasetObject,3);
    vis->spin();
    while (!vis->wasStopped ())
    {
        vis->spinOnce ();
    }

    return 0;
}