//
// Created by jens on 10/30/20.
//

#include <dataset/scape/ScapeDatasetObject.hpp>

ScapeDatasetObject::ScapeDatasetObject() {}

ScapeDatasetObject::ScapeDatasetObject(std::string _path, std::vector<std::filesystem::path> &recognition_paths,
                                       bool verbose) : DatasetObject(_path),
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
    camera_pose(1, 1) = -1;
    camera_pose(2, 2) = -1;

    get_filenames_with_ext_from_dir(pc_data_ext, path, pcd_filenames);
    std::sort(pcd_filenames.begin(), pcd_filenames.end()); // To make indexing regonition data easyer

    // Find gt files
    std::filesystem::path gt_path = _path;
    gt_path = gt_path.parent_path()/="gt";

    std::string di_s;
    for(auto &di : std::filesystem::directory_iterator(gt_path)){
        std::string di_s = di.path().string();
        if (di_s.find(name_singular)!=di_s.npos){
            for(int i = 0 ; i < pcd_filenames.size(); i++){
                std::string fn_stem = std::filesystem::path(pcd_filenames[i]).stem();
                if(di_s.find(fn_stem)!=di_s.npos){
                    pcd_fn_to_gt_fn[pcd_filenames[i]]=di_s;
                }
            }
        }
    }
    std::sort(gt_file_data.begin(),gt_file_data.end(),[](const std::pair<std::string,int> &a, const std::pair<std::string,int> &b) {return a.second < b.second;}); // Sort based on int


    bool recog_pcd_match;
    for (int i = 0; i < recognition_paths.size(); i++) {
        for (auto &de:std::filesystem::directory_iterator(recognition_paths[i])) {
            if (de.is_regular_file()) {
                std::filesystem::path p = de.path();
                std::string ext = p.replace_extension().extension();
                if (ext == ".pc3d") {
                    std::string p_str = p.string();
                    std::string filename_without_zoneidx;
                    for (int j = 0; j < pcd_filenames.size(); j++) {
                        filename_without_zoneidx = pcd_filenames[j].substr(0, pcd_filenames[j].find_last_not_of('_') - 1);
                        recog_pcd_match = (p_str.find(filename_without_zoneidx) != p_str.npos);
                        if (recog_pcd_match)
                            break;
                    }
                    if (recog_pcd_match) {
                        {
                            int n_zones = get_corners(p_str + "_ZoneCorners.txt");

                            auto begin = scape_data_points.begin() += (scape_data_points.size() - n_zones);
                            auto end = scape_data_points.end();

                            load_object_candidates(p_str + ".txt", begin, end); // Adds object candidates
                            load_scores(p_str + "_scores.txt", begin, end);
                            for (auto itt = begin; itt != end; ++itt) {
                                std::string search_string =
                                        filename_without_zoneidx + "_" + std::to_string(itt->zone_idx);
                                auto itt_fn = std::find(pcd_filenames.begin(), pcd_filenames.end(), search_string);

                                if (itt_fn == pcd_filenames.end())
                                    if (verbose)
                                        std::cout << "Filename not found" << std::endl;

                                // Relate recognition to pcd and gt data and vice verso
                                 // Maps from filename to zone ptr or ptrs in case of multiple recog of the same pcd
                                itt->pcd_filename =*itt_fn;
                                pcd_fn_to_scape_dpis[*itt_fn].emplace_back(std::distance(scape_data_points.begin(), itt));

                                if(pcd_fn_to_gt_fn.contains(*itt_fn)) {
                                    itt->ground_truth_path = pcd_fn_to_gt_fn[*itt_fn];
                                    load_gt(itt->ground_truth_path, itt->gts);
                                }
                            }
                        }
                    } else {
                        if (verbose)
                            std::cout << "No pcd data match for: " << p_str << "\n";
                    }
                }
            }
        }
    }

    std::sort(scape_data_points.begin(),scape_data_points.end(),[](ScapeDataPoint &a,ScapeDataPoint &b){return a.pcd_filename<b.pcd_filename;});
    for(auto& sdp:scape_data_points){
        data_points.emplace_back(static_cast<DataPoint>(sdp)); // Cast scape data point to regular so that they can be used my the general interface
    }

}

std::vector<T4> ScapeDatasetObject::get_object_candidates(unsigned int n) {
    return scape_data_points[n].ocs;
};

int ScapeDatasetObject::size() const { return scape_data_points.size(); }

PointCloudT::Ptr ScapeDatasetObject::get_pcd(DataPoint &dp) {
    PointCloudT::Ptr pc = pcl::make_shared<PointCloudT>();
    pcl::io::loadPCDFile(path.string() + "/" + dp.pcd_filename + pc_data_ext, *pc);
    return pc;
}

PointCloudT::Ptr ScapeDatasetObject::get_pcd(int n) {
    return get_pcd(data_points[n]);
}

int ScapeDatasetObject::get_corners(std::filesystem::path path) {
    std::ifstream oc_file(path);

    std::string line, val;
    int n_zones = 0;
    double x, y, z;
    while (std::getline(oc_file, line)) {
        if (!line.empty()) {
            size_t find_match = line.find("ZoneIndex:");
            if (find_match != line.npos) {
                n_zones++;
                scape_data_points.emplace_back();
                scape_data_points.back().zone_idx = std::stod(line.substr(find_match + 10));

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

                    scape_data_points.back().corners[i] = Eigen::Vector3d(x, y, z);
                }
                scape_data_points.back().init_projection_plane();
            }
        }
    }

    return n_zones;
}

void ScapeDatasetObject::load_scores(std::filesystem::path path, std::vector<ScapeDataPoint>::iterator begin,
                                     std::vector<ScapeDataPoint>::iterator end) {
    std::ifstream oc_file(path);
    std::string word;
    int global_index = 0;
    int zoneindex = -1;
    while (oc_file >> word) {
        if (word.npos != word.find("ZoneIndex")) {
            oc_file >> zoneindex;
        } else {
            double score = std::stod(word);
            for (auto itt = begin; itt != end; itt++) {
                if (std::find(itt->ocs_global_index.begin(), itt->ocs_global_index.end(), global_index) !=
                    itt->ocs_global_index.end()) {
                    itt->oc_scores.push_back(score);
                }
            }
            global_index++;
        }
    }
}


void ScapeDatasetObject::load_object_candidates(std::filesystem::path path, std::vector<ScapeDataPoint>::iterator begin,
                                                std::vector<ScapeDataPoint>::iterator end) {
    std::ifstream oc_file(path);

    T4 oc;
    int global_index = 0;
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
                if (itt->push_back_oc_if_valid(oc)) {
                    itt->ocs_global_index.push_back(global_index);
                }
            }
            global_index++;
        }
    }
};

int ScapeDatasetObject::get_n_zones(std::filesystem::path path) {
    std::ifstream zone_file(path);
    std::string word;

    int n_zones = 0;
    while (zone_file >> word) {
        if (word.find("ZoneIndex") != word.npos) {
            scape_data_points.emplace_back();
            zone_file >> scape_data_points.back().zone_idx;
            n_zones++;
        }
    }
    return n_zones;
}

std::ifstream& operator >> (std::ifstream& in, T4 &t4)
{
    in >> t4(0, 0) >> t4(0, 1) >> t4(0, 2) >> t4(0, 3) >> t4(1, 0) >> t4(1, 1) >> t4(1, 2)
            >> t4(1, 3)
            >> t4(2, 0) >> t4(2, 1) >> t4(2, 2) >> t4(2, 3) >> t4(3, 0) >> t4(3, 1) >> t4(3, 2)
            >> t4(3, 3);

    return in;
}

void ScapeDatasetObject::load_gt(std::string path, std::vector<T4> &gts) {
    std::ifstream gt_file(path);
    std::string word;
    T4 gt;
    while (gt_file >> gt) {
        gts.emplace_back(gt);
    }

}

