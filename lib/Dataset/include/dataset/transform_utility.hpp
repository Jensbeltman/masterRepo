#ifndef MASTER_POSE_NOISE_H
#define MASTER_POSE_NOISE_H

#include <Eigen/Eigen>
#include <random>
#include <chrono>
#include <math.h>
#include <vector>
#include <dataset/typedefinitions.hpp>
#include <dataset/DataPoint.hpp>

namespace tu{

    class NoiseGen {
    public:
        std::mt19937_64 rng; // random generator
        std::uniform_real_distribution<double> uniform_dist;
        std::normal_distribution<double> normal_dist_r;
        std::normal_distribution<double> normal_dist_t;
        std::uniform_int_distribution<int> uniform_dist_int;

        explicit NoiseGen(double std_t = 0.1, double std_r = M_PI / 4);
        void append_noisy_transforms(std::vector<T4> &Ts, int n);
        void append_noisy_transforms(std::vector<T4> &Ts,std::vector<T4> &Tsn, int n);
        T4 get_noisy_transform(T4 &T);
        Eigen::Vector3d spherical_uniform_unitvector();
    };


    void non_maximum_supression(DataPoint &dp, double t_thresh, double r_thresh,std::vector<T4> symmetry_transforms = std::vector<T4>{});
    void find_correct_ocs(std::vector<T4> ocs, std::vector<T4> gts, double t_thresh, double r_thresh, std::vector<int> &correct_oc_indices, std::vector<T4> symmetry_transforms = std::vector<T4>{});
    void find_correct_ocs(std::vector<T4> ocs, std::vector<T4> gts, double t_thresh, double r_thresh, std::vector<int> &correct_oc_indices,std::vector<double> &t_dists,std::vector<double> &r_dists, std::vector<T4> symmetry_transforms = std::vector<T4>{});

    void getFPTN(std::vector<int> &tp, std::vector<int> &tn, std::vector<int> &fp, std::vector<int> &fn,std::vector<bool> chromosome,std::vector<int> correct_oc_indices);
    void getFPTN(int &tp, int &tn, int &fp, int &fn,std::vector<bool> chromosome,std::vector<int> correct_oc_indices);
    double SafeAcos (double x);

}


#endif //MASTER_POSE_NOISE_H
