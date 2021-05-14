//
// Created by jens on 1/22/21.
//

#ifndef MASTER_ALGORITHM_DATA_PROC_HPP
#define MASTER_ALGORITHM_DATA_PROC_HPP
#include "algorithm_interface.hpp"
#include "dataset/transform_utility.hpp"
#include "dataset/scape/ScapeDataset.hpp"
#include "dataset/sileane/SileaneDataset.hpp"
#include "matplot/matplot.h"
#include "datautil/ga_conversions.hpp"
#include "datautil/csv_doc.hpp"

class AlgorithmDataProc {
public:
    AlgorithmDataProc();
    AlgorithmDataProc(double t_thresh, double r_thresh);
    double t_thresh, r_thresh;

    std::vector<std::string> column_names = {"algName","objName","dpI","chromosome","tp","tn","fp","fn","precision","recall","accuracy","f1","time","cost","t_dist_avr","r_dist_avr","t_dist_std","r_dist_std"};
    std::map<std::string,int> column_name_indices;
    void set_column_names(rapidcsv::CSVDocPtr &csvDoc);
    void append_processed_data_to_doc(rapidcsv::CSVDocPtr &csvDoc,int row_i,std::string &alg_name,DatasetObjectPtr &objPtr,int dpI, HVResult &hvResult);
    void t_r_dist(chromosomeT &chromosome,std::vector<int> &correct_oc_i,std::vector<double> &t_dists, std::vector<double> &r_dists, double &t_dist_avr, double &r_dist_avr, double &t_dist_std, double &r_dist_std);
};


#endif //MASTER_ALGORITHM_DATA_PROC_HPP
