#ifndef MASTER_SCAPEDATASETOBJECT_HPP
#define MASTER_SCAPEDATASETOBJECT_HPP

#include <dataset/DatasetObject.hpp>
#include <dataset/scape/ScapeDataPoint.hpp>
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <fstream>
#include <sstream>
#include <array>
#include <algorithm>
#include <map>

class ScapeDatasetObject : public DatasetObject {
typedef std::shared_ptr<ScapeDatasetObject> ScapeDatasetObjectPtr;
public:
    std::string name_singular;
    std::vector<std::filesystem::path> recognition_paths;
    std::vector<ScapeDataPoint> scape_data_points;

    std::map<std::string,std::string> pcd_fn_to_gt_fn;
    std::vector<std::pair<std::string,int>> gt_file_data;

    std::map<std::string,std::vector<int>> pcd_fn_to_scape_dpis;


    ScapeDatasetObject();

    ScapeDatasetObject(std::string _path, std::vector<std::filesystem::path> &recognition_paths, bool verbose = false);


    std::vector<T4> get_object_candidates(unsigned int n) override;

    int size() const override; // Accounts for there being multiple recognitions results / zones

    PointCloudT::Ptr get_pcd(int n) override;

    PointCloudT::Ptr get_pcd(DataPoint &dp) override;

    void load_gt(std::string path, std::vector<T4> &gts);
private:
    int get_corners(std::filesystem::path path);

    static void load_scores(std::filesystem::path path, std::vector<ScapeDataPoint>::iterator begin,
                            std::vector<ScapeDataPoint>::iterator end);

    static void load_object_candidates(std::filesystem::path path, std::vector<ScapeDataPoint>::iterator begin,
                                       std::vector<ScapeDataPoint>::iterator end);



    int get_n_zones(std::filesystem::path path);
};

typedef std::shared_ptr<ScapeDatasetObject> ScapeDatasetObjectPtr;

#endif //MASTER_SCAPEDATASETOBJECT_HPP
