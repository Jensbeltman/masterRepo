#include "dataset/sileane/SileaneDataset.hpp"

SileaneDataset::SileaneDataset(std::string path) : Dataset(path) {
    name = "Sileane";
        for (auto &p : std::filesystem::directory_iterator(path))
        if (std::filesystem::is_directory(p.path())) {
            objects.push_back(
                    std::static_pointer_cast<DatasetObject>(std::make_shared<SileaneDatasetObject>(p.path())));
        }
};