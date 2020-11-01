#ifndef MASTER_DATASET_HPP
#define MASTER_DATASET_HPP

#include <dataset/DatasetObject.hpp>
#include <memory>

typedef std::shared_ptr<DatasetObject> DatasetObjectPtr;

class Dataset {
public:
    std::string name;
    std::filesystem::path path;
    std::vector<DatasetObjectPtr> objects;

    Dataset() = default;

    explicit Dataset(std::string _path);

    virtual DatasetObjectPtr get_object_by_name(std::string name);

};


#endif //MASTER_DATASET_HPP
