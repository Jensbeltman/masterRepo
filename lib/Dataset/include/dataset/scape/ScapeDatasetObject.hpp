#ifndef MASTER_SCAPEDATASETOBJECT_HPP
#define MASTER_SCAPEDATASETOBJECT_HPP

#include <dataset/DatasetObject.hpp>
#include <dataset/scape/ScapeZone.hpp>
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <fstream>
#include <sstream>
#include <array>
#include <algorithm>


class ScapeDatasetObject : public DatasetObject {
public:
    std::string name_singular;
    std::vector<std::filesystem::path> recognition_paths;
    std::vector<ScapeZone> zones;
    std::vector<std::pair<std::string,int>> gt_file_data;

    ScapeDatasetObject();

    ScapeDatasetObject(std::string _path, std::vector<std::filesystem::path> &recognition_paths, bool verbose = false);

    std::vector<T4> get_object_candidates(unsigned int n) override;

    int size() const override; // Accounts for there being multiple recognitions results / zones

    PointCloudT::Ptr get_pcd(int n) override;

    bool has_gt(int n) override;

    bool has_scores(int n) override;

    std::vector<T4> get_gt(unsigned int n) override;

    std::vector<double> get_scores(unsigned int n) override;

private:
    int get_corners(std::filesystem::path path);

    static void load_scores(std::filesystem::path path, std::vector<ScapeZone>::iterator begin,
                            std::vector<ScapeZone>::iterator end);

    static void load_object_candidates(std::filesystem::path path, std::vector<ScapeZone>::iterator begin,
                                       std::vector<ScapeZone>::iterator end);

    std::vector<T4> load_gt(std::string path);

    int get_n_zones(std::filesystem::path path);
};


#endif //MASTER_SCAPEDATASETOBJECT_HPP
