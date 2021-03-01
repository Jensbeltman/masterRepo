#include "dataset/Dataset.hpp"

Dataset::Dataset(std::string _path) : path(_path) {
        name = path.stem().string();
};

DatasetObjectPtr Dataset::get_object_by_name(std::string name) {
    for (DatasetObjectPtr &sileaneObject:objects)
        if (sileaneObject->name == name)
            return sileaneObject;
    return nullptr;
}

bool Dataset::operator<(const DatasetObject &rhs) const {
    return name < rhs.name;
};