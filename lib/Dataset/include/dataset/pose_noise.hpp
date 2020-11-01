#ifndef MASTER_POSE_NOISE_H
#define MASTER_POSE_NOISE_H

#include <Eigen/Eigen>
#include <random>
#include <chrono>
#include <math.h>
#include <vector>

class PoseNoise {
public:
    std::mt19937_64 rng; // random generator
    std::uniform_real_distribution<double> uniform_dist;
    std::normal_distribution<double> normal_dist_r;
    std::normal_distribution<double> normal_dist_t;
    std::uniform_int_distribution<int> uniform_dist_int;


    explicit PoseNoise(double std_t = 0.1, double std_r = M_PI / 4);

    void append_noisy_transforms(std::vector<Eigen::Transform<double, 3, Eigen::Affine>> &Ts, int n);

    Eigen::Vector3d spherical_uniform_unitvector();

    Eigen::Transform<double, 3, Eigen::Affine> get_noisy_transform(Eigen::Transform<double, 3, Eigen::Affine> &T);
};

#endif //MASTER_POSE_NOISE_H
