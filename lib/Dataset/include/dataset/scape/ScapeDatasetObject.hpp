#ifndef MASTER_SCAPEDATASETOBJECT_HPP
#define MASTER_SCAPEDATASETOBJECT_HPP

#include <dataset/DatasetObject.hpp>
#include <fstream>
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <sstream>
#include <array>
#include <algorithm>
#include <dataset/scape/ScapeZone.hpp>


class ScapeDatasetObject : public DatasetObject {
public:
    std::string name_singular;
    std::vector<std::filesystem::path> recognition_paths;
    std::vector<ScapeZone> zones;

    ScapeDatasetObject() {};

    ScapeDatasetObject(std::string _path, std::vector<std::filesystem::path> &recognition_paths, bool verbose = false) : DatasetObject(_path),
                                                                                                   recognition_paths(
                                                                                                           recognition_paths) {
        type = "Scape";
        pc_data_ext = ".pcd";
        mesh_data_ext = ".ply";
        name_singular = name.substr(0, name.size() - 1);
        mesh_path = ((path.parent_path() /= "models") /= (name_singular + mesh_data_ext));
        mesh_pcd_path = ((path.parent_path() /= "models") /= (name_singular + pc_data_ext));
        camera_pose = T4::Identity();
        // Rotate camera pose so that z points downwards (for visibility calc)//Todo get camera pose from scape if possible
        camera_pose(1,1)=-1;
        camera_pose(2,2)=-1;

        get_filenames_with_ext_from_dir(pc_data_ext, path, filenames);
        std::sort(filenames.begin(), filenames.end()); // To make indexing regonition data easyer
        bool recog_pcd_match;
        for (int i = 0; i < recognition_paths.size(); i++) {
            for (auto &de:std::filesystem::directory_iterator(recognition_paths[i])) {
                if (de.is_regular_file()) {
                    std::filesystem::path p = de.path();
                    std::string ext = p.replace_extension().extension();
                    if (ext == ".pc3d") {
                        std::string p_str = p.string();
                        std::string filename_without_zoneidx;
                        for (int j = 0; j < filenames.size(); j++) {
                            filename_without_zoneidx = filenames[j].substr(0, filenames[j].find_last_not_of('_') - 1);
                            recog_pcd_match = (p_str.find(filename_without_zoneidx) != p_str.npos);
                            if (recog_pcd_match)
                                break;
                        }
                        if (recog_pcd_match) {
                            {
                                int n_zones = get_corners(p_str + "_ZoneCorners.txt");

                                auto begin = zones.begin() += (zones.size() - n_zones);
                                auto end = zones.end();

                                get_object_candidates(p_str + ".txt", begin, end); // Adds object candidates
                                get_scores(p_str + "_scores.txt", begin, end);
                                for (auto itt = begin; itt != end; ++itt) {
                                    std::string search_string =
                                            filename_without_zoneidx + "_" + std::to_string(itt->zone_idx);
                                    auto itt_fn = std::find(filenames.begin(), filenames.end(), search_string);

                                    if (itt_fn == filenames.end())
                                        if(verbose)
                                            std::cout << "Filename not found" << std::endl;

                                    itt->pc_filename_idx = std::distance(filenames.begin(), itt_fn);
                                }
                            }
                        }
                        else{
                            if(verbose)
                                std::cout<<"No pcd data match for: "<<p_str<<"\n";
                        }
                    }
                }
            }
        }
    };

    std::vector<T4> get_object_candidates(unsigned int n) {
        return zones[n].ocs;
    };

    int size() const { return zones.size(); }; // Accounts for there being multiple recognitions results

    PointCloudT::Ptr get_pcd(int n) {
        n %= filenames.size();
        PointCloudT::Ptr pc = pcl::make_shared<PointCloudT>();
        pcl::io::loadPCDFile(path.string() + "/" + filenames[zones[n].pc_filename_idx] + pc_data_ext, *pc);
        return pc;
    };

    bool has_gt(int n){return !zones[n].gt.empty();}
    bool has_scores(int n){return !zones[n].scores.empty();}
    std::vector<T4> get_gt(unsigned int n) {return zones[n].gt; }
    std::vector<double> get_scores(unsigned int n) { return zones[n].scores;}

private:

    int get_corners(std::filesystem::path path) {
        std::ifstream oc_file(path);

        std::string line, val;
        int n_zones = 0;
        double x, y, z;
        while (std::getline(oc_file, line)) {
            if (!line.empty()) {
                size_t find_match = line.find("ZoneIndex:");
                if (find_match != line.npos) {
                    n_zones++;
                    zones.emplace_back();
                    zones.back().zone_idx = std::stod(line.substr(find_match + 10));

                    std::getline(oc_file, line);
                    line.erase(std::remove_if(line.begin(), line.end(),
                                              [](unsigned char x) {
                                                  return ((x == '(') || (x == ')') || (x == '\r'));
                                              }), line.end());
                    std::istringstream cords(line);
                    for (int i = 0; i < 4; i++) {
                        getline(cords, val, ',');
                        x = std::stod(val);
                        getline(cords, val, ',');
                        y = std::stod(val);
                        getline(cords, val, ' ');
                        z = std::stod(val);

                        zones.back().corners[i] = Eigen::Vector3d(x, y, z);
                    }
                    zones.back().init_projection_plane();
                }
            }
        }

        return n_zones;
    }

    void get_scores(std::filesystem::path path, std::vector<ScapeZone>::iterator begin, std::vector<ScapeZone>::iterator end) {
        std::ifstream oc_file(path);
        std::string word;
        int global_index=0;
        int zoneindex = -1;
        while (oc_file >> word) {
            if (word.npos != word.find("ZoneIndex")) {
                oc_file >> zoneindex;
            } else {
                double score = std::stod(word);
                for (auto itt = begin; itt != end; itt++) {
                    if ( std::find(itt->ocs_global_index.begin(),itt->ocs_global_index.end(),global_index) != itt->ocs_global_index.end()){
                        itt->scores.push_back(score);
                    }
                }
                global_index++;
            }
        }
    }



    void
    get_object_candidates(std::filesystem::path path, std::vector<ScapeZone>::iterator begin,
                          std::vector<ScapeZone>::iterator end) {
        std::ifstream oc_file(path);

        T4 oc;
        int global_index=0;
        std::string word;
        int zoneindex = -1;
        while (oc_file >> word) {
            if (word.npos != word.find("ZoneIndex")) {
                oc_file >> zoneindex;
            } else {
                oc(0, 0) = std::stod(word);
                oc_file >> oc(0, 1) >> oc(0, 2) >> oc(0, 3) >> oc(1, 0) >> oc(1, 1) >> oc(1, 2)
                        >> oc(1, 3)
                        >> oc(2, 0) >> oc(2, 1) >> oc(2, 2) >> oc(2, 3) >> oc(3, 0) >> oc(3, 1) >> oc(3, 2)
                        >> oc(3, 3);

                for (auto itt = begin; itt != end; itt++) {
                    if (itt->push_back_oc_if_valid(oc)){
                        itt->ocs_global_index.push_back(global_index);
                    }
                }
                global_index++;
            }
        }
    };

    int get_n_zones(std::filesystem::path path) {
        std::ifstream oc_file(path);
        std::string word;

        int n_zones = 0;
        while (oc_file >> word) {
            if (word.find("ZoneIndex") != word.npos) {
                zones.emplace_back();
                oc_file >> zones.back().zone_idx;
                n_zones++;
            }
        }
        return n_zones;
    }
};



#endif //MASTER_SCAPEDATASETOBJECT_HPP
