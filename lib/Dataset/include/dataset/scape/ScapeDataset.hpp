//
// Created by jens on 10/12/20.
//

#ifndef MASTER_SCAPEDATASET_HPP
#define MASTER_SCAPEDATASET_HPP

#include <dataset/Dataset.hpp>
#include <dataset/scape/ScapeDatasetObject.hpp>
#include <array>


typedef std::shared_ptr<ScapeDatasetObject> ScapeDatasetObjectPtr;

class ScapeDataset : public Dataset {
public:
    ScapeDataset(std::string path, std::string recognition_path,bool verbose = false) {
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
                            std::make_shared<ScapeDatasetObject>(pdi.path(), recognition_paths,verbose)));
                }
            }
        }
    }
};


#endif //MASTER_SCAPEDATASET_HPP