#include "dataset/DatasetObject.hpp"

 DatasetObject::DatasetObject(std::filesystem::path path, std::string data_ext): path (path), pc_data_ext(data_ext) {
    name = path.stem().string();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr DatasetObject::get_pcd(int n) { return nullptr; }
pcl::PointCloud<pcl::PointXYZ>::Ptr DatasetObject::get_pcd(DataPoint &dp) {return nullptr;}

std::shared_ptr<cv::viz::Mesh> DatasetObject::get_mesh() {
    std::shared_ptr<cv::viz::Mesh> meshptr = std::make_shared<cv::viz::Mesh>(cv::viz::Mesh::load(mesh_path));
    return meshptr;
}

pcl::shared_ptr<PointCloudT> DatasetObject::get_mesh_point_cloud() {
    pcl::shared_ptr<PointCloudT> pc = pcl::make_shared<PointCloudT>();
    pcl::io::loadPCDFile(mesh_pcd_path, *pc);
    return pc;
}

pcl::shared_ptr<NormalCloudT> DatasetObject::get_mesh_normal_cloud() {
    pcl::shared_ptr<NormalCloudT> nc = pcl::make_shared<NormalCloudT>();
    pcl::io::loadPCDFile(mesh_pcd_path, *nc);
    return nc;
}

int DatasetObject::size() const { return data_points.size(); }

bool DatasetObject::has_gt(int n){return !data_points[n].gts.empty();}

bool DatasetObject::has_scores(int n){return !data_points[n].oc_scores.empty();}

std::vector<T4> DatasetObject::get_gt(unsigned int n) { return data_points[n].gts; }

std::vector<double> DatasetObject::get_scores(unsigned int n) { return data_points[n].oc_scores; }

std::vector<T4> DatasetObject::get_object_candidates(unsigned int n) { return data_points[n].ocs; }

void DatasetObject::get_filenames_with_ext_from_dir(std::string ext, std::string dir, std::vector<std::string> &vofs) {
    for (auto &p : std::filesystem::directory_iterator(dir)) {
        if (p.path().extension() == ext)
            vofs.push_back(p.path().stem().string());
        }
    }

bool DatasetObject::operator<(const DatasetObject &rhs) const {
    return name < rhs.name;
}

bool DatasetObject::operator==(const DatasetObject &rhs) const {
    return name == rhs.name;
}

//int DatasetObject::datapoint_index(DataPoint &dp) {
//    auto found_it = std::find(data_points.begin(),data_points.end(),dp);
//    if(found_it!=data_points.end())
//        return std::distance(data_points.begin(),found_it);
//    else
//        return -1;
//}
//
