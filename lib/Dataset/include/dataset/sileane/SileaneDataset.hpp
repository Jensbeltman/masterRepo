#ifndef OPENGACUSTOM_SILEANEDATASET_HPP
#define OPENGACUSTOM_SILEANEDATASET_HPP

#include <iostream>
#include <vector>
#include "SileaneDatasetObject.hpp"
#include <fstream>
#include <filesystem>
#include <dataset/Dataset.hpp>

typedef std::shared_ptr<SileaneDatasetObject> SileaneDatasetObjectPtr;

class SileaneDataset : public Dataset {
public:
    explicit SileaneDataset(std::string path);
};
typedef std::shared_ptr<SileaneDataset> SileaneDatasetPtr;


#endif //OPENGACUSTOM_SILEANEDATASET_HPP
