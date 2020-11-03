#include "dataset/transform_utility.hpp"

TransformUtility::TransformUtility(double std_t, double std_r) : normal_dist_t(
        std::normal_distribution<double>(0, std_t)),
                                                                 normal_dist_r(
                                                                         std::normal_distribution<double>(0, std_r)) {
    // initialize the random number generator with time-dependent seed
    uint64_t timeSeed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    std::seed_seq ss{uint32_t(timeSeed & 0xffffffff), uint32_t(timeSeed >> 32)};
    rng.seed(ss);

}

void TransformUtility::append_noisy_transforms(std::vector<Eigen::Transform<double, 3, Eigen::Affine>> &Ts, int n) {
    uniform_dist_int = std::uniform_int_distribution<int>(0, Ts.size());
    for (int i = 0; i < n; i++) {
        Ts.push_back(get_noisy_transform(Ts[uniform_dist_int(rng)]));
    }
}

Eigen::Vector3d TransformUtility::spherical_uniform_unitvector() {
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
TransformUtility::get_noisy_transform(Eigen::Transform<double, 3, Eigen::Affine> &T) {
    Eigen::Transform<double, 3, Eigen::Affine> Tn = T;
    Tn.translation() += spherical_uniform_unitvector() * normal_dist_t(rng);
    Tn.linear() *= Eigen::AngleAxis<double>(normal_dist_r(rng), spherical_uniform_unitvector()).matrix();
    return Tn;
}

std::vector<bool>
TransformUtility::get_true_positives(std::vector<T4> ocs, std::vector<T4> gts, double t_thresh, double r_thresh) {
    std::vector<bool> fp(ocs.size(), false);


    for (auto gt:gts) {
        int best_i = -1;
        double best_t_diff = std::numeric_limits<double>::max();
        double best_r_diff = std::numeric_limits<double>::max();

        for (int i = 0; i < ocs.size(); i++) {
            Eigen::Vector3d oc_t = ocs[i].matrix().block<3, 1>(0, 3);
            Eigen::Vector3d gt_t = gt.matrix().block<3, 1>(0, 3);
            Eigen::Matrix3d oc_r = ocs[i].matrix().block<3, 3>(0, 0);
            Eigen::Matrix3d gt_r = gt.matrix().block<3, 3>(0, 0);
            Eigen::Vector3d t = oc_t - gt_t;
            Eigen::Matrix3d R = gt_r.transpose() * oc_r;

            double t_diff = t.lpNorm<2>();
            double r_diff = Eigen::AngleAxis<double>(R).angle();
            if ((t_diff <= t_thresh) && (r_diff <= r_thresh)) {
                if (t_diff < best_t_diff && r_diff < best_r_diff) {
                    best_i = i;
                    best_t_diff = t_diff;
                    best_r_diff = r_diff;
                }
            }
        }
        if (best_i != -1) {
            fp[best_i] = true;
        }
    }

    return fp;
}
