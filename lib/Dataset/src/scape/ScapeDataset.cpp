#include "dataset/scape/ScapeDataset.hpp"

ScapeDataset::ScapeDataset(std::string path, std::string recognition_path, bool verbose) {
    name = "Scape";
    for (auto &pdi : std::filesystem::directory_iterator(path)) {
        if (pdi.is_directory()) {
            std::array<std::string, 3> ignore{"gt", "models", "symmetry"};
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

                // Add symetry tranforms to object if it has any
                std::filesystem::path symmetry_path =
                        std::filesystem::path(path) / "symmetry" / (objects.back()->name + ".txt");
                if (std::filesystem::exists(symmetry_path)) {
                    std::ifstream oc_file(symmetry_path);
                    T4 oc;
                    std::string word;
                    while (oc_file >> word) {
                        oc(0, 0) = std::stod(word);
                        oc_file >> oc(0, 1) >> oc(0, 2) >> oc(0, 3) >> oc(1, 0) >> oc(1, 1) >> oc(1, 2)
                                >> oc(1, 3)
                                >> oc(2, 0) >> oc(2, 1) >> oc(2, 2) >> oc(2, 3) >> oc(3, 0) >> oc(3, 1) >> oc(3, 2)
                                >> oc(3, 3);
                        objects.back()->symmetry_transforms.emplace_back(oc);
                    }
                }
            }
        }
    }
}

ScapeDatasetObjectPtr ScapeDataset::get_scape_object_by_name(std::string name) {
    return std::static_pointer_cast<ScapeDatasetObject>(get_object_by_name(name));
}
