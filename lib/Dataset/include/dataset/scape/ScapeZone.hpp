#ifndef MASTER_SCAPEZONE_HPP
#define MASTER_SCAPEZONE_HPP

#include <dataset/typedefinitions.hpp>
#include <fstream>
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <sstream>
#include <array>
#include <algorithm>

class ScapeZone {
public:
    ScapeZone() {};
    int zone_idx = 0;
    int pc_filename_idx = 0;
    std::vector<T4> ocs;
    std::vector<int> ocs_global_index;// The oc index in the recognition files
    std::vector<T4> gt;
    std::vector<double> scores;


    // ScapeZone plane information
    std::array<Eigen::Vector3d, 4> corners;
    std::array<Eigen::Vector3d, 2> projection_vectors;
    Eigen::Vector3d normal;
    std::array<Eigen::Vector2d, 4> projected_corners;
    std::array<Eigen::Vector2d, 4> projected_corner_vectors;
    std::array<Eigen::Vector2d, 4> projected_corner_vectors_p;


    bool push_back_oc_if_valid(T4 &object_candidate) {
        Eigen::Vector3d tvec = object_candidate.matrix().block<3, 1>(0, 3);
        Eigen::Vector2d ptvec = project_to_plane(tvec);
        std::array<bool, 4> res;
        for (int i = 0; i < 4; i++) {
            res[i] = std::signbit(projected_corner_vectors_p[i].dot(projected_corners[i] - ptvec));
        }

        if (std::all_of(res.begin(), res.end(), [](bool r) { return r; }) ||
            std::all_of(res.begin(), res.end(), [](bool r) { return !r; })) {
            ocs.push_back(object_candidate);
            return true;
        }
        return false;
    };

    void init_projection_plane() {
        projection_vectors[0] = (corners[1] - corners[0]);
        normal = projection_vectors[0].cross(corners[3] - corners[0]);
        projection_vectors[1] = normal.cross(projection_vectors[0]);
        projection_vectors[0].normalize();
        projection_vectors[1].normalize();

        for (int i = 0; i < 4; i++) {
            projected_corners[i] = project_to_plane(corners[i]);
        }
        for (int i = 0; i < 4; i++) {
            projected_corner_vectors[i] = projected_corners[(i + 1) % 4] - projected_corners[i];
        }
        for (int i = 0; i < 4; i++) {
            projected_corner_vectors_p[i] = perpendicular_clockwise(projected_corner_vectors[i]);
        }
    };

    Eigen::Vector2d project_to_plane(Eigen::Vector3d &vec) {
        return Eigen::Vector2d(projection_vectors[0].dot(vec), projection_vectors[1].dot(vec));
    };

    static Eigen::Vector2d perpendicular_clockwise(Eigen::Vector2d &vec) {
        return Eigen::Vector2d(vec[1], -vec[0]);
    };

    static Eigen::Vector2d perpendicular_counter_clockwise(Eigen::Vector2d &vec) {
        return Eigen::Vector2d(-vec[1], vec[0]);
    };
};
#endif //MASTER_SCAPEZONE_HPP
