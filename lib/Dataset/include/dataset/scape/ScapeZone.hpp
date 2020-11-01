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
    ScapeZone() = default;

    int zone_idx = 0;
    int pc_filename_idx = 0;
    int gt_file_data_idx = 0;
    std::vector<T4> ocs;
    std::vector<int> ocs_global_index;// The oc index in the recognition files
    std::vector<double> scores;
    bool has_gt=false;

    // ScapeZone plane information
    Eigen::Vector3d normal;
    std::array<Eigen::Vector3d, 4> corners;
    std::array<Eigen::Vector3d, 2> projection_vectors;
    std::array<Eigen::Vector2d, 4> projected_corners;
    std::array<Eigen::Vector2d, 4> projected_corner_vectors;
    std::array<Eigen::Vector2d, 4> projected_corner_vectors_p;



    bool push_back_oc_if_valid(T4 &object_candidate);

    void init_projection_plane();

    Eigen::Vector2d project_to_plane(Eigen::Vector3d &vec);

    static Eigen::Vector2d perpendicular_clockwise(Eigen::Vector2d &vec);

    static Eigen::Vector2d perpendicular_counter_clockwise(Eigen::Vector2d &vec);
};

#endif //MASTER_SCAPEZONE_HPP
