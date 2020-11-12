#include "dataset/scape/ScapeDataPoint.hpp"

bool ScapeDataPoint::push_back_oc_if_valid(T4 &object_candidate) {
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
}

void ScapeDataPoint::init_projection_plane() {
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
}

Eigen::Vector2d ScapeDataPoint::project_to_plane(Eigen::Vector3d &vec) {
    return Eigen::Vector2d(projection_vectors[0].dot(vec), projection_vectors[1].dot(vec));
}

Eigen::Vector2d ScapeDataPoint::perpendicular_clockwise(Eigen::Vector2d &vec) {
    return Eigen::Vector2d(vec[1], -vec[0]);
}

Eigen::Vector2d ScapeDataPoint::perpendicular_counter_clockwise(Eigen::Vector2d &vec) {
    return Eigen::Vector2d(-vec[1], vec[0]);
}