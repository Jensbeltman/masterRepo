#ifndef MASTER_SCAPEDATASET_HPP
#define MASTER_SCAPEDATASET_HPP

#include <dataset/Dataset.hpp>
#include <dataset/scape/ScapeDatasetObject.hpp>
#include <array>

typedef std::shared_ptr<ScapeDatasetObject> ScapeDatasetObjectPtr;

class ScapeDataset : public Dataset {
public:
    ScapeDataset(std::string path, std::string recognition_path, bool verbose = false);
};

#endif //MASTER_SCAPEDATASET_HPP
