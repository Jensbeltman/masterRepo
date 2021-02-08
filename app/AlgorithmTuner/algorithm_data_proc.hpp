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
    AlgorithmDataProc(std::string derived_data_path,std::string static_data_path);
    AlgorithmDataProc(rawDataMapAlgObjVecT rawDataMapAlgObjVec,double t_thresh, double r_thresh);

    void update_data();

    rawDataMapAlgObjVecT rawDataMapAlgObjVec;

    // Derived Data
    rapidcsv::CSVDocPtr derivedCSVDocPtr = nullptr;
    std::vector<std::string> algName,objName;
    std::vector<int> dpIndex;
    std::vector<chromosomeT> chromosome;
    std::vector<int> tp,tn,fp,fn;
    std::vector<double> accuracy, precision ,recall, time;

    // Static Data
    rapidcsv::CSVDocPtr staticCSVDocPtr = nullptr;
    std::string DatasetType,DatasetPath;
    std::vector<std::string> uniqAlgNames,uniqObjNames;
    double t_thresh, r_thresh;

    // Utility data
    std::vector<std::vector<bool>> trueOCVec;
    std::vector<std::vector<int>> tpIVec,tnIVec,fpIVec,fnIVec;
    std::vector<std::vector<T4>> ocVec, gtVec;
    std::vector<int> nOCVec;


    TransformUtility tu;

    void save_data(std::string derived_data_filename, std::string static_data_filename);

    void getFPTN(std::vector<int> &tp, std::vector<int> &tn, std::vector<int> &fp, std::vector<int> &fn,chromosomeT chromosome, chromosomeT correct_ocs);

    void getFPTN(int &tp, int &fp, int &tn, int &fn, chromosomeT chromosome, chromosomeT correct_ocs);



    matplot::figure_handle  bar_plot(std::string value_name);

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

    double get_avr(std::vector<int> &vals, std::string alg_key, std::string obj_key = "");
};


#endif //MASTER_ALGORITHM_DATA_PROC_HPP
