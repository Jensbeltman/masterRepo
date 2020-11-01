#ifndef OPENGACUSTOM_SILEANECAMERAPARAMS_HPP
#define OPENGACUSTOM_SILEANECAMERAPARAMS_HPP

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <array>
#include <Eigen/Eigen>
#include "dataset/typedefinitions.hpp"

class SileaneCameraParams {
public:
    int width = 0;
    int height = 0;
    int fu = 0;
    int fv = 0;
    int cu = 0;
    int cv = 0;
    double clip_start = 0;
    double clip_end = 0;
    Eigen::Vector3d location;
    Eigen::Quaternion<double> rotation;
    T4 T;

    SileaneCameraParams() = default;

    SileaneCameraParams(const SileaneCameraParams &other) = default;

    explicit SileaneCameraParams(std::string path);
};


#endif //OPENGACUSTOM_SILEANECAMERAPARAMS_HPP
