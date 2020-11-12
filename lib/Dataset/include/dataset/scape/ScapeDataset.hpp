#ifndef MASTER_SCAPEDATASET_HPP
#define MASTER_SCAPEDATASET_HPP

#include <dataset/Dataset.hpp>
#include <dataset/scape/ScapeDatasetObject.hpp>
#include <array>

class ScapeDataset : public Dataset {
public:
    ScapeDataset(std::string path, std::string recognition_path, bool verbose = false);
};
typedef std::shared_ptr<ScapeDataset> ScapeDatasetPtr;

#endif //MASTER_SCAPEDATASET_HPP
