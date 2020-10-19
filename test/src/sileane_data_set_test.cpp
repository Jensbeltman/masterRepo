#include <dataset/SileaneDataset.hpp>
#include <chronometer.h>
#include <opencv2/viz/vizcore.hpp>
#include <pcl/common/common.h>

int main() {
    Chronometer chronometer;
    SileaneData sileaneData("/home/jens/masterData/Siléane-Dataset");

    std::cout << "Following objects where loaded for the silean dataset" << "\n";
    for (auto &ob:sileaneData.objects) {
        std::cout << ob->name << ", Number of filenames: " << ob->filenames.size() << "\n";
    }

    SileaneObjectPtr sileaneObject = std::static_pointer_cast<SileaneDatasetObject>(sileaneData.objects[1]);
    int sample_n = 2;
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
    std::vector<T4> object_candidates = sileaneObject->get_object_candidates(sample_n);
    int n_gt_poses = object_candidates.size();

    // Camera pose for the chosen dataset object
    T4 camera_pose = sileaneObject->sileaneCameraParams.T;

    return 0;
}