#ifndef OPENGACUSTOM_SILEANEDATASET_HPP
#define OPENGACUSTOM_SILEANEDATASET_HPP

#include <iostream>
#include <vector>
#include "SileaneDatasetObject.hpp"
#include <fstream>
#include <filesystem>
#include <dataset/Dataset.hpp>

typedef std::shared_ptr<SileaneDatasetObject> SileaneDatasetObjectPtr;

class SileaneData : public Dataset {
public:
    explicit SileaneData(std::string path);
};


#endif //OPENGACUSTOM_SILEANEDATASET_HPP
