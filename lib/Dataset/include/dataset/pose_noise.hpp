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


    PoseNoise(double std_t = 0.1, double std_r = M_PI / 4) : normal_dist_t(std::normal_distribution<double>(0, std_t)),
                                                             normal_dist_r(std::normal_distribution<double>(0, std_r)) {
        // initialize the random number generator with time-dependent seed
        uint64_t timeSeed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
        std::seed_seq ss{uint32_t(timeSeed & 0xffffffff), uint32_t(timeSeed >> 32)};
        rng.seed(ss);

    };

    Eigen::Vector3d spherical_uniform_unitvector() {
        Eigen::Vector3d unitvector;
        double phi = uniform_dist(rng) * 2 * M_PI;
        double chostheta = 2 * (uniform_dist(rng) - 0.5);
        double theta = std::acos(chostheta);
        unitvector[0] = std::sin(theta) * std::cos(phi);
        unitvector[1] = std::sin(theta) * std::sin(phi);
        unitvector[2] = std::sin(theta);
        return unitvector;
    }

    Eigen::Transform<double, 3, Eigen::Affine> get_noisy_transform(Eigen::Transform<double, 3, Eigen::Affine> &T) {
        Eigen::Transform<double, 3, Eigen::Affine> Tn = T;
        Tn.translation() += spherical_uniform_unitvector() * normal_dist_t(rng);
        Tn.linear() *= Eigen::AngleAxis<double>(normal_dist_r(rng), spherical_uniform_unitvector()).matrix();
        return Tn;
    }

    void append_noisy_transforms(std::vector<Eigen::Transform<double, 3, Eigen::Affine>> &Ts, int n) {
        uniform_dist_int = std::uniform_int_distribution<int>(0, Ts.size());
        for (int i = 0; i < n; i++) {
            Ts.push_back(get_noisy_transform(Ts[uniform_dist_int(rng)]));
        }
    }
};

#endif //MASTER_POSE_NOISE_H
