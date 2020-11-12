#include <dataset/scape/ScapeDataset.hpp>
#include <chronometer.h>
#include <opencv2/viz/vizcore.hpp>
#include <pcl/common/common.h>

int main() {
    Chronometer chronometer;
    chronometer.tic();
    ScapeDataset scapeData("/home/jens/masterData/ScapeDataset/Scape/Full_Dataset",
                           "/home/jens/masterData/ScapeDataset/Data from Scape Recognition");
    std::cout << "\nScape Dataset Init Load time: " << chronometer.toc() << "s\n";

    std::cout << "Following objects where loaded for the scape dataset" << "\n";
    for (auto &ob:scapeData.objects) {
        ScapeDatasetObjectPtr sob = std::dynamic_pointer_cast<ScapeDatasetObject>(ob);
        std::cout   <<"\t"<<ob->name
                    <<"\n\t\tnumber of zones: " << ob->size()
                    <<"\n\t\tnumber of pcd filenames: " << ob->pcd_filenames.size()
                    <<"\n\t\tnumber of recognition paths: " << sob->recognition_paths.size()
                    <<"\n\n";
    }


    chronometer.tic();
    for (auto object:scapeData.objects) {
        ScapeDatasetObjectPtr sobject = std::dynamic_pointer_cast<ScapeDatasetObject>(object);
        for (int i = 0; i < object->size(); i++) {
            ScapeDataPoint &scape_data_point = sobject->scape_data_points[i];
            PointCloudT::Ptr pc = object->get_pcd(i);
            if (pc->points.empty())
                std::cout << "Datasample " << i << " pointcloud was empty" << "\n";
            std::vector<T4> &ocs = scape_data_point.ocs;
            std::vector<double> &scores = scape_data_point.oc_scores;
            if (ocs.empty())
                std::cout << "Object " << sobject->name << " candidates were empty for zone "
                          << scape_data_point.zone_idx << " for pc file "
                          << scape_data_point.pcd_filename << "\n";
            if(ocs.size() != scores.size())
                std::cout << "Object " << sobject->name << " had missmatch between n of ocs and scores for zone "
                          << sobject->scape_data_points[i].zone_idx << "\n";

        }
    }
    std::cout << "Scape Dataset Traversal of pcs and ocs time: " << chronometer.toc() << "s\n\n";

    ScapeDatasetObjectPtr scapeObject = std::static_pointer_cast<ScapeDatasetObject>(scapeData.objects[1]);


    // DatasetObject sample point cloud
    int sample_n = 2;
    std::cout << "Testing zone " << sample_n << " with related point cloud data filename "
              << scapeObject->data_points[sample_n].pcd_filename << " from dataset object folder "
              << scapeObject->name << "\n\n";
    PointCloudT::Ptr pc = scapeObject->get_pcd(sample_n);


    // DatasetObject mesh ply data
    std::shared_ptr<cv::viz::Mesh> meshptr = scapeObject->get_mesh();
    std::cout << "Mesh loaded from " << scapeObject->mesh_path << "\n\n";

    // DatasetObject mesh pc data
    PointCloudT::Ptr pcm = scapeObject->get_mesh_point_cloud();
    std::cout << "Mesh point cloud loaded from " << scapeObject->mesh_path << " n of points: " << pcm->points.size()
              << "\n\n";
    NormalCloudT::Ptr ncm = scapeObject->get_mesh_normal_cloud();
    std::cout << "Mesh normal cloud loaded from " << scapeObject->mesh_path << " n of points: " << ncm->points.size()
              << "\n\n";

    // Camera pose for the chosen dataset object
    std::cout << "Camera pose\n" << scapeObject->camera_pose.matrix() << "\n\n";


    std::cout << "Point cloud loaded with " << pc->points.size() << " points" << "\n\n";

    // DatasetObject candidates (GT)
    std::vector<T4> object_candidates = scapeObject->get_object_candidates(sample_n);
    int n_oc = object_candidates.size();
    std::cout << "Found " << n_oc << " oc's First and last one is \n" << object_candidates[0].matrix() << "\n"
              << object_candidates.back().matrix() << "\n\n";


    return 0;
}