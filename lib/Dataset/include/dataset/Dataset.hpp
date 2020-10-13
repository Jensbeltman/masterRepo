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

    Dataset(std::string _path) : path(_path) {
        name = path.stem().string();
    };

    virtual DatasetObjectPtr get_object_by_name(std::string name) {
        for (DatasetObjectPtr &sileaneObject:objects)
            if (sileaneObject->name == name)
                return sileaneObject;
        return nullptr;
    };

};


#endif //MASTER_DATASET_HPP
