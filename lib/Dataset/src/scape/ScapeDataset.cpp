#include "dataset/scape/ScapeDataset.hpp"

ScapeDataset::ScapeDataset(std::string path, std::string recognition_path, bool verbose) {
    name = "Scape";
    for (auto &pdi : std::filesystem::directory_iterator(path)) {
        if (pdi.is_directory()) {
            std::array<std::string, 2> ignore{"gt", "models"};
            if (std::count(ignore.begin(), ignore.end(), pdi.path().stem()) == 0) {
                std::vector<std::filesystem::path> recognition_paths;
                for (auto &rpdi : std::filesystem::directory_iterator(recognition_path)) {
                    if (rpdi.is_directory()) {
                        std::string folder_name = pdi.path().stem().string();
                        std::string recognition_folder_name = rpdi.path().stem().string();
                        if (recognition_folder_name.find(folder_name.substr(0, folder_name.size() - 1)) !=
                            std::string::npos)// if folder name contains dataset object name
                        {
                            recognition_paths.push_back(rpdi.path());
                        }
                    }
                }
                objects.push_back(std::static_pointer_cast<DatasetObject>(
                        std::make_shared<ScapeDatasetObject>(pdi.path(), recognition_paths, verbose)));
            }
        }
    }
}