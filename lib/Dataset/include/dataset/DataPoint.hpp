#ifndef MASTER_DATAPOINT_HPP
#define MASTER_DATAPOINT_HPP
#include "dataset/typedefinitions.hpp"

class DataPoint {
public:
    std::string pcd_filename;
    std::string ground_truth_path;
    std::vector<T4> ocs;
    std::vector<T4> gts;
    std::vector<double> oc_scores;
};


#endif //MASTER_DATAPOINT_HPP
