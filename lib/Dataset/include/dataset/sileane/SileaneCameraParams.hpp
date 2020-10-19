#ifndef OPENGACUSTOM_SILEANECAMERAPARAMS_HPP
#define OPENGACUSTOM_SILEANECAMERAPARAMS_HPP

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <array>
#include <Eigen/Eigen>


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
    Eigen::Transform<double, 3, Eigen::Affine> T;

    SileaneCameraParams() = default;

    SileaneCameraParams(const SileaneCameraParams &other) = default;

    explicit SileaneCameraParams(std::string path) {
        std::ifstream file(path);
        std::string line;
        std::string word;
        while (getline(file, line)) {
            std::istringstream iss(line);
            std::string word;
            while (iss >> word) {
                if (word == "width") {
                    iss >> width;
                }
                if (word == "height") {
                    iss >> height;
                }
                if (word == "fu") {
                    iss >> fu;
                }
                if (word == "fv") {
                    iss >> fv;
                }
                if (word == "cu") {
                    iss >> cu;
                }
                if (word == "cv") {
                    iss >> cv;
                }
                if (word == "clip_start") {
                    iss >> clip_start;
                }
                if (word == "clip_end") {
                    iss >> clip_end;
                }
                if (word == "location") {
                    iss >> location(0);
                    iss >> location(1);
                    iss >> location(2);
                }
                if (word == "rotation") {
                    iss >> rotation.w();
                    iss >> rotation.x();
                    iss >> rotation.y();
                    iss >> rotation.z();
                }
            }
        }
        T.matrix().block<3, 3>(0, 0) = rotation.toRotationMatrix();
        T.matrix().block<3, 1>(0, 3) = location.matrix();
    };
};


#endif //OPENGACUSTOM_SILEANECAMERAPARAMS_HPP
