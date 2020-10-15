//
// Created by jens on 10/12/20.
//

#ifndef MASTER_SCAPEOBJECT_HPP
#define MASTER_SCAPEOBJECT_HPP

#include <dataset/DatasetObject.hpp>


class ScapeObject : public DatasetObject {
    std::filesystem::path recognition_path;


    ScapeObject(std::string path,std::string recognition_path) : DatasetObject(path),recognition_path(recognition_path) {};


};


#endif //MASTER_SCAPEOBJECT_HPP
