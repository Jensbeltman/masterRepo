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

//    matplot::figure_handle  bar_plot(std::string value_name);
/*
    size_t begin_index(std::string key,std::vector<std::string> &keys);
    size_t end_index(std::string key,std::vector<std::string> &keys);

    bool get_begin_end_index(size_t &bi, size_t &ei,std::string alg_key, std::string obj_key = "");

    template <typename T>
    std::vector<double> VecIToVecD(std::vector<T> &vec){
        return std::vector<double>(vec.begin(), vec.end());
    }

    double get_sum(std::vector<double> &vals, std::string alg_key, std::string obj_key = "");

    double get_avr(std::vector<double> &vals, std::string alg_key, std::string obj_key = "");

    int get_sum(std::vector<int> &vals, std::string alg_key, std::string obj_key = "");

    double get_avr(std::vector<int> &vals, std::string alg_key, std::string obj_key = "");*/
};


#endif //MASTER_ALGORITHM_DATA_PROC_HPP
