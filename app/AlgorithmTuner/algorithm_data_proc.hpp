//
// Created by jens on 1/22/21.
//

#ifndef MASTER_ALGORITHM_DATA_PROC_HPP
#define MASTER_ALGORITHM_DATA_PROC_HPP
#include "algorithm_interface.hpp"
#include "dataset/transform_utility.hpp"
#include "matplot/matplot.h"
class AlgorithmDataProc {
public:
    AlgorithmDataProc();
    AlgorithmDataProc(rawDataMapAlgObjVecT rawDataMapAlgObjVec,double t_thresh, double r_thresh);
    void generate_data(double t_thresh, double r_thresh);
//    void update_data(double t_thresh, double r_thresh);

    rawDataMapAlgObjVecT rawDataMapAlgObjVec;

    std::vector<std::string> algNameVec,objNameVec;
    std::vector<std::string> algNameUniqueVec,objNameUniqueVec;
    std::vector<chromosomeT> chromosomeVec;
    std::vector<std::vector<T4>> ocVec, gtVec;
    std::vector<std::vector<bool>> trueOCVec;
    std::vector<int> tpVec,tnVec,fpVec,fnVec, nOCVec;
    std::vector<std::vector<int>> tpIVec,tnIVec,fpIVec,fnIVec;
    std::vector<double> accuracyVec, precisionVec ,recallVec, timeVec;


    TransformUtility tu;

    void save_data(std::string folder);

    void getFPTN(std::vector<int> &tp, std::vector<int> &tn, std::vector<int> &fp, std::vector<int> &fn,chromosomeT chromosome, chromosomeT correct_ocs);

    void getFPTN(int &tp, int &fp, int &tn, int &fn, chromosomeT chromosome, chromosomeT correct_ocs);


    matplot::figure_handle  bar_plot(std::vector<int> &valVec);
    matplot::figure_handle  bar_plot(std::vector<double> &valVec);

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
