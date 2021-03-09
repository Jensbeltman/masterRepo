#include "dataset/transform_utility.hpp"
#include <algorithm>    // std::find
#include <cmath>
#include <math.h>
#include <iomanip>


namespace tu {

    NoiseGen::NoiseGen(double std_t, double std_r) : normal_dist_t(
            std::normal_distribution<double>(0, std_t)),
                                                     normal_dist_r(
                                                             std::normal_distribution<double>(0,
                                                                                              std_r)) {
        // initialize the random number generator with time-dependent seed
        uint64_t timeSeed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
        std::seed_seq ss{uint32_t(timeSeed & 0xffffffff), uint32_t(timeSeed >> 32)};
        rng.seed(ss);
    }

    void NoiseGen::append_noisy_transforms(std::vector<Eigen::Transform<double, 3, Eigen::Affine>> &Ts, int n) {
        uniform_dist_int = std::uniform_int_distribution<int>(0, Ts.size());
        for (int i = 0; i < n; i++) {
            Ts.push_back(get_noisy_transform(Ts[uniform_dist_int(rng)]));
        }
    }

    void NoiseGen::append_noisy_transforms(std::vector<T4> &Ts, std::vector<T4> &Tsn, int n) {
        uniform_dist_int = std::uniform_int_distribution<int>(0, Ts.size());
        for (int i = 0; i < n; i++) {
            Tsn.emplace_back(get_noisy_transform(Ts[uniform_dist_int(rng)]));
        }
    }

    Eigen::Vector3d NoiseGen::spherical_uniform_unitvector() {
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
    NoiseGen::get_noisy_transform(Eigen::Transform<double, 3, Eigen::Affine> &T) {
        Eigen::Transform<double, 3, Eigen::Affine> Tn = T;
        Tn.translation() += spherical_uniform_unitvector() * normal_dist_t(rng);
        Tn.linear() *= Eigen::AngleAxis<double>(normal_dist_r(rng), spherical_uniform_unitvector()).matrix();
        return Tn;
    }

    void find_correct_ocs(std::vector<T4> ocs, std::vector<T4> gts, double t_thresh, double r_thresh,
                          std::vector<int> &correct_oc_indices, std::vector<T4> symmetry_transforms) {
        std::vector<double> t_dists;
        std::vector<double> r_dists;
        find_correct_ocs(ocs, gts, t_thresh, r_thresh, correct_oc_indices, t_dists, r_dists, symmetry_transforms);
    }

    void find_correct_ocs(std::vector<T4> ocs, std::vector<T4> gts, double t_thresh, double r_thresh,
                          std::vector<int> &correct_oc_indices, std::vector<double> &t_dists,
                          std::vector<double> &r_dists, std::vector<T4> symmetry_transforms) {
        for (auto gt:gts) {
            int best_i = -1;
            double best_t_diff = std::numeric_limits<double>::max();
            double best_r_diff = std::numeric_limits<double>::max();

            std::vector<T4> gt_with_symmetry = {gt};
            for (auto &st:symmetry_transforms) {
                gt_with_symmetry.emplace_back(gt * st);
            }
            for (auto &gtws:gt_with_symmetry) {
                gtws.matrix().block<3, 3>(0, 0).transposeInPlace();
                gtws.matrix().block<3, 1>(0, 3) = -gtws.matrix().block<3, 3>(0, 0) * gtws.matrix().block<3, 1>(0, 3);
            }

            for (int i = 0; i < ocs.size(); i++) {
                {
                    for (auto &gtws:gt_with_symmetry) {
                        //note that gtws have already been inverted
                        T4 diff = gtws * ocs[i];
                        Eigen::Vector3d t_diff_vec = diff.matrix().block<3, 1>(0, 3);
                        Eigen::Matrix3d r_diff_mat = diff.matrix().block<3, 3>(0, 0);

                        double t_diff = t_diff_vec.lpNorm<2>();
                        double r_diff = SafeAcos((r_diff_mat.trace() - 1) / 2) * (180.0 / M_PI);
                        if (std::isnan(t_diff) || std::isnan(r_diff))
                            std::cout << "Nan in find correct ocs" << std::endl;
                        if ((t_diff <= t_thresh) && (r_diff <= r_thresh)) {
                            if (t_diff < best_t_diff && r_diff < best_r_diff) {
                                if (std::find(correct_oc_indices.begin(), correct_oc_indices.end(), i) ==
                                    correct_oc_indices.end()) {
                                    best_i = i;
                                    best_t_diff = t_diff;
                                    best_r_diff = r_diff;
                                }
                            }
                        }
                    }
                }
            }
            if (best_i != -1) {
                correct_oc_indices.emplace_back(best_i);
                t_dists.emplace_back(best_t_diff);
                r_dists.emplace_back(best_r_diff);
            }
        }
    }

    void non_maximum_supression(DataPoint &dp, double t_thresh, double r_thresh, std::vector<T4> symmetry_transforms) {
        std::vector<bool> disabled(dp.ocs.size(), false);
        std::vector<bool> chosen(dp.ocs.size(), false);
        int best_i;
        double best_t_diff;
        double best_r_diff;

        Eigen::Vector3d t_diff_vec;
        Eigen::Matrix3d r_diff_mat;

        for (auto &gt:dp.gts) {
            std::vector<int> approved;
            best_i = -1;
            best_t_diff = std::numeric_limits<double>::max();
            best_r_diff = std::numeric_limits<double>::max();
            std::vector<T4> gt_with_symmetry = {gt};

            for (auto &st:symmetry_transforms) {
                gt_with_symmetry.emplace_back(gt * st);
            }
            for (auto &gtws:gt_with_symmetry) {
                gtws.matrix().block<3, 3>(0, 0).transposeInPlace();
                gtws.matrix().block<3, 1>(0, 3) = -gtws.matrix().block<3, 3>(0, 0) * gtws.matrix().block<3, 1>(0, 3);
            }

            for (int i = 0; i < dp.ocs.size(); i++) {
                if (!chosen[i] && !disabled[i]) {
                    auto &oc = dp.ocs[i];

                    for (auto &gtws:gt_with_symmetry) {
                        T4 diff = gtws * oc;
                        t_diff_vec = diff.matrix().block<3, 1>(0, 3);
                        r_diff_mat = diff.matrix().block<3, 3>(0, 0);

                        double t_diff = t_diff_vec.lpNorm<2>();
                        double r_diff = SafeAcos((r_diff_mat.trace() - 1) / 2) * (180.0 / M_PI);


                        if ((t_diff <= t_thresh) && (r_diff <= r_thresh)) {
                            approved.emplace_back(i);
                            if (t_diff < best_t_diff && r_diff < best_r_diff) {
                                best_i = i;
                                best_t_diff = t_diff;
                                best_r_diff = r_diff;
                            }
                        }
                    }
                }
            }

            if (best_i != -1) {
                for (int i:approved)
                    if (i != best_i) {
                        disabled[i] = true;
                    }

                chosen[best_i] = true;
            }
        }

        auto tmp_ocs = dp.ocs;
        auto tmp_oc_scores = dp.oc_scores;
        dp.ocs.clear();
        dp.oc_scores.clear();
        for(int i = 0; i < tmp_ocs.size(); i++){
            if(!disabled[i])
            {
                dp.ocs.emplace_back(tmp_ocs[i]);
                dp.oc_scores.emplace_back(tmp_oc_scores[i]);
            }
        }
    }


    void getFPTN(std::vector<int> &tp, std::vector<int> &tn, std::vector<int> &fp, std::vector<int> &fn,
                 std::vector<bool> chromosome, std::vector<int> correct_oc_indices) {

        std::vector<bool> correct_ocs(chromosome.size(), false);
        for (auto &i:correct_oc_indices)
            correct_ocs[i] = true;

        for (int i = 0; i < correct_ocs.size(); i++) {
            if (correct_ocs[i]) {
                if (chromosome[i]) {
                    tp.emplace_back(i);
                } else {
                    fn.emplace_back(i);
                }
            } else {
                if (chromosome[i]) {
                    fp.emplace_back(i);
                } else {
                    tn.emplace_back(i);
                }
            }
        }
    }

    void
    getFPTN(int &tp, int &tn, int &fp, int &fn, std::vector<bool> chromosome, std::vector<int> correct_oc_indices) {
        {
            std::vector<bool> correct_ocs(chromosome.size(), false);
            for (auto &i:correct_oc_indices)
                correct_ocs[i] = true;

            for (int i = 0; i < correct_ocs.size(); i++) {
                if (correct_ocs[i]) {
                    if (chromosome[i]) {
                        tp++;
                    } else {
                        fn++;
                    }
                } else {
                    if (chromosome[i]) {
                        fp++;
                    } else {
                        tn++;
                    }
                }
            }
        }
    }

    double SafeAcos(double x) {
        if (x < -1.0) x = -1.0;
        else if (x > 1.0) x = 1.0;
        return acos(x);
    }


}