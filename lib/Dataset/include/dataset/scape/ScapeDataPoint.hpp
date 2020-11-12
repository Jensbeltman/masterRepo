#ifndef MASTER_SCAPEDATAPOINT_HPP
#define MASTER_SCAPEDATAPOINT_HPP

#include <dataset/typedefinitions.hpp>
#include <dataset/DataPoint.hpp>
#include <fstream>
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <sstream>
#include <array>
#include <algorithm>


class ScapeDataPoint: public DataPoint{
public:
    ScapeDataPoint() = default;

    int zone_idx = 0;
    std::vector<int> ocs_global_index;// The oc index in the recognition files

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

#endif //MASTER_SCAPEDATAPOINT_HPP
