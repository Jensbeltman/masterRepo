#ifndef OPENGACUSTOM_SILEANEDATASET_HPP
#define OPENGACUSTOM_SILEANEDATASET_HPP

#include <iostream>
#include <vector>
#include "SileaneObject.hpp"
#include <fstream>
#include <filesystem>
#include <dataset/Dataset.hpp>

typedef std::shared_ptr<SileaneObject> SileaneObjectPtr;

class SileaneData : public Dataset {
public:
    SileaneData(std::string path) : Dataset(path) {
        for (auto &p : std::filesystem::directory_iterator(path))
            if (std::filesystem::is_directory(p.path())) {
                objects.push_back(std::static_pointer_cast<DatasetObject>(std::make_shared<SileaneObject>(p.path())));
            }
    };
};


#endif //OPENGACUSTOM_SILEANEDATASET_HPP
