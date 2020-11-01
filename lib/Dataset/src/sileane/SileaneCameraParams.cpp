#include "dataset/sileane/SileaneCameraParams.hpp"

SileaneCameraParams::SileaneCameraParams(std::string path) {
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