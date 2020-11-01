#include "dataset/pose_noise.hpp"

PoseNoise::PoseNoise(double std_t, double std_r) : normal_dist_t(std::normal_distribution<double>(0, std_t)),
                                                   normal_dist_r(std::normal_distribution<double>(0, std_r)) {
    // initialize the random number generator with time-dependent seed
    uint64_t timeSeed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    std::seed_seq ss{uint32_t(timeSeed & 0xffffffff), uint32_t(timeSeed >> 32)};
    rng.seed(ss);

}

void PoseNoise::append_noisy_transforms(std::vector<Eigen::Transform<double, 3, Eigen::Affine>> &Ts, int n) {
    uniform_dist_int = std::uniform_int_distribution<int>(0, Ts.size());
    for (int i = 0; i < n; i++) {
        Ts.push_back(get_noisy_transform(Ts[uniform_dist_int(rng)]));
    }
}

Eigen::Vector3d PoseNoise::spherical_uniform_unitvector() {
    Eigen::Vector3d unitvector;
    double phi = uniform_dist(rng) * 2 * M_PI;
    double chostheta = 2 * (uniform_dist(rng) - 0.5);
    double theta = std::acos(chostheta);
    unitvector[0] = std::sin(theta) * std::cos(phi);
    unitvector[1] = std::sin(theta) * std::sin(phi);
    unitvector[2] = std::sin(theta);
    return unitvector;
}

Eigen::Transform<double, 3, Eigen::Affine>
PoseNoise::get_noisy_transform(Eigen::Transform<double, 3, Eigen::Affine> &T) {
    Eigen::Transform<double, 3, Eigen::Affine> Tn = T;
    Tn.translation() += spherical_uniform_unitvector() * normal_dist_t(rng);
    Tn.linear() *= Eigen::AngleAxis<double>(normal_dist_r(rng), spherical_uniform_unitvector()).matrix();
    return Tn;
}
