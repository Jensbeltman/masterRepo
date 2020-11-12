#include "dataset/sileane/SileaneDatasetObject.hpp"

SileaneDatasetObject::SileaneDatasetObject(std::filesystem::path path, std::string data_ext) : DatasetObject(path,
                                                                                                             data_ext) {
    type = "Sileane";
    pc_data_ext = ".pcd";
    mesh_data_ext = ".ply";
    get_filenames_with_ext_from_dir(data_ext, path.string() + "/depth", pcd_filenames);
    std::sort(pcd_filenames.begin(), pcd_filenames.end());
    mesh_path = (path / "mesh").replace_extension(mesh_data_ext);
    mesh_pcd_path = (path / "mesh").replace_extension(pc_data_ext);
    sileaneCameraParams = SileaneCameraParams(path / "camera_params.txt");
    camera_pose = sileaneCameraParams.T;
    has_depth_gt = std::filesystem::exists(path / "depth_gt");

    if (!std::filesystem::exists(path / "depth")) {
        std::cout << "No depth directory found object" << "\"name\"" << "might not work as intended" << std::endl;
    }

    data_points.resize(pcd_filenames.size());
    for(int i = 0;i<pcd_filenames.size();i++){
        auto &fn =pcd_filenames[i];
        auto &dp =data_points[i];
        load_object_candidates_and_gt(fn,dp);
        dp.pcd_filename = fn;
    }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr SileaneDatasetObject::get_pcd(int n) { return get_pcd(n, false); }

pcl::PointCloud<pcl::PointXYZ>::Ptr SileaneDatasetObject::get_pcd(int n, bool gt) {
    if (n < pcd_filenames.size()) {
        std::filesystem::path dir(path);
        if (gt) {
            if (has_depth_gt) {
                dir /= "depth_gt";
            } else {
                std::cout << "No gt data available" << std::endl;
            }
        } else {
            dir /= "depth";
        }
        return sileane_depth_to_pcd((dir / pcd_filenames[n]).replace_extension(".PNG"), sileaneCameraParams);
    } else {
        return nullptr;
    }
};

cv::Mat SileaneDatasetObject::get_color(int n, bool gt) {
    if (n < pcd_filenames.size()) {
        std::filesystem::path p = (path / "rgb" / pcd_filenames[n]).replace_extension(pc_data_ext);
        if (std::filesystem::exists(p)) {
            return cv::imread(p, cv::IMREAD_UNCHANGED);
        } else {
            std::cout << "Could not find color image at \"" << p << "\"" << std::endl;
        }
    }
    return cv::Mat();
};


void SileaneDatasetObject::load_object_candidates_and_gt(std::string pcd_filename, DataPoint &dp) {

    dp.ground_truth_filename = pcd_filename;
    std::ifstream i((path / "gt" / pcd_filename).replace_extension(".json"));

    nlohmann::json j;
    i >> j;

    for (auto gt : j) {
        T4 T;
        T(0, 0) = gt["R"][0][0];
        T(0, 1) = gt["R"][0][1];
        T(0, 2) = gt["R"][0][2];
        T(1, 0) = gt["R"][1][0];
        T(1, 1) = gt["R"][1][1];
        T(1, 2) = gt["R"][1][2];
        T(2, 0) = gt["R"][2][0];
        T(2, 1) = gt["R"][2][1];
        T(2, 2) = gt["R"][2][2];
        T(0, 3) = gt["t"][0];
        T(1, 3) = gt["t"][1];
        T(2, 3) = gt["t"][2];
        dp.gts.push_back(T);
    }

    TransformUtility transformUtility(mu_noise, sigma_noise);
    if (n_noisy_poses == -1)
    for(auto &gt:dp.gts)
        dp.ocs.emplace_back(transformUtility.get_noisy_transform(gt));
    else{
        transformUtility.append_noisy_transforms(dp.gts, dp.ocs, n_noisy_poses);
    }

}

pcl::PointCloud<pcl::PointXYZ>::Ptr
SileaneDatasetObject::sileane_depth_to_pcd(std::string path, SileaneCameraParams &camera_params) {
    cv::Mat depth = cv::imread(path, cv::IMREAD_UNCHANGED);
    int width = depth.cols;
    int height = depth.rows;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
    auto pc_itt = pc->begin();
    cv::MatIterator_<u_int16_t> depth_itt = depth.begin<u_int16_t>();
    double depth_range = camera_params.clip_end - camera_params.clip_start;
    pcl::PointXYZ p;

    for (int r = 0; r < height; r++) {
        for (int c = 0; c < width; c++) {
            u_int16_t d = depth.at<u_int16_t>(r, c);
            if (d < 65535) {
                pcl::PointXYZ p;
                p.z = camera_params.clip_start + depth_range * (d / (float) 65535);
                p.x = p.z * (c - camera_params.cu) / (double) camera_params.fu;
                p.y = p.z * (r - camera_params.cv) / (double) camera_params.fv;
                pc->push_back(p);
            }
        }
    }
    pcl::transformPointCloud(*pc, *pc, camera_params.location, camera_params.rotation);

    return pc;
}
