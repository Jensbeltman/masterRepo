#include "dataset/sileane/SileaneDataset.hpp"

SileaneData::SileaneData(std::string path) : Dataset(path) {
        for (auto &p : std::filesystem::directory_iterator(path))
        if (std::filesystem::is_directory(p.path())) {
            objects.push_back(
                    std::static_pointer_cast<DatasetObject>(std::make_shared<SileaneDatasetObject>(p.path())));
        }
};