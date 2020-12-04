#include "ga/visualization/point_cloud_group_visualizer.hpp"
#include <ga/genetic_evaluator/GeneticEvaluatorObjectCandidates.hpp>
#include <ga/typedefinitions.hpp>

#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>
#include <thread>
#include <chrono>
#include <vtkRenderWindow.h>
#include <vtkCamera.h>

using namespace std::chrono_literals;


void PointCloudGroupVisualizer::init() {
    registerKeyboardCallback(&PointCloudGroupVisualizer::keyboardCallback, *this);
    setShowFPS(false);
};


PointCloudGroupVisualizer::PointCloudGroupVisualizer(const std::string &name, const bool create_interactor) : PCLVisualizer(name,
                                                                                                                            create_interactor) {
    init();
}

PointCloudGroupVisualizer::PointCloudGroupVisualizer(int &argc, char **argv, const std::string &name,
                                                     pcl::visualization::PCLVisualizerInteractorStyle *style,
                                                     const bool create_interactor) : PCLVisualizer(argc, argv, name, style,
                                                                                 create_interactor) {
    init();
}

PointCloudGroupVisualizer::PointCloudGroupVisualizer(vtkSmartPointer<vtkRenderer> ren, vtkSmartPointer<vtkRenderWindow> wind,
                                                     const std::string &name, const bool create_interactor) : PCLVisualizer(ren, wind,
                                                                                                          name,
                                                                                                          create_interactor) {
    init();
}

PointCloudGroupVisualizer::PointCloudGroupVisualizer(int &argc, char **argv, vtkSmartPointer<vtkRenderer> ren,
                                                     vtkSmartPointer<vtkRenderWindow> wind, const std::string &name,
                                                     pcl::visualization::PCLVisualizerInteractorStyle *style,
                                                     const bool create_interactor) : PCLVisualizer(argc, argv, ren, wind, name, style,
                                                                                 create_interactor) {
    init();
}

void PointCloudGroupVisualizer::updateSelector() {

    group_ids_itt = group_ids.begin();

    if (!group_ids.empty()) {

        std::array<double, 3> rgb = group_color[group_ids_itt->first];
        pcl::visualization::Camera camera;
        getCameraParameters(camera);
        if (contains(text_id)) {
            update_text();
        } else {
            addText(group_ids_itt->first, static_cast<int>(camera.window_size[0] / 2),
                    static_cast<int>(camera.window_size[1] - font_size * 1.5), font_size, rgb[0], rgb[1],
                    rgb[2],
                    text_id);
            change_pt_size(group_ids_itt->first, 1);
        }
    }
}

void PointCloudGroupVisualizer::custom_spin() {
    updateSelector();
    spin();
}

void PointCloudGroupVisualizer::update_text() {
    std::string s = group_ids_itt->first;
    std::array<double, 3> rgb = group_color[s];
    pcl::visualization::Camera camera;
    getCameraParameters(camera);
    updateText(group_ids_itt->first, camera.window_size[0] / 2, camera.window_size[1] - font_size * 1.5,
               font_size,
               rgb[0], rgb[1], rgb[2], text_id);
}

void PointCloudGroupVisualizer::toggle_opacity(std::string id) {
    double current_opacity;
    getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, current_opacity, id);
    current_opacity = (current_opacity < 1.0) ? 1.0 : 0.0;
    setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, current_opacity, id);
}

void PointCloudGroupVisualizer::toggle_group_opacity(std::string group_id) {

    if (!group_ids[group_id].empty()) {
        double current_opacity;
        getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, current_opacity,
                                         group_ids[group_id][0]);
        current_opacity = (current_opacity < 1.0) ? 1.0 : 0.0;
        for (auto &id:group_ids[group_id]) {
            setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, current_opacity,
                                             id);
        }
    }
}

void PointCloudGroupVisualizer::change_pt_size(std::string group_id, int change) {
    if (!group_ids[group_id].empty()) {
        double current_point_size;
        getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                         current_point_size,
                                         group_ids[group_id][0]);
        for (auto &id:group_ids[group_id]) {
            setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                             current_point_size + change, id);
        }
    }
}

void PointCloudGroupVisualizer::keyboardCallback(const pcl::visualization::KeyboardEvent &event, void *viewer_void) {
    if (!group_ids.empty()) {
        if (!event.isShiftPressed()) {
            if (event.keyDown()) {
                std::string key = event.getKeySym();
                if (key == "Right" && (group_ids_itt != --group_ids.end())) {
                    group_ids_itt++;
                    ids_itt = group_ids_itt->second.begin();
                } else if (key == "Left" && group_ids_itt != group_ids.begin()) {
                    group_ids_itt--;
                    ids_itt = group_ids_itt->second.begin();
                } else if (key == "Up") {
                    toggle_group_opacity(group_ids_itt->first);
                }
                update_text();
            }
        } else {
            if (event.keyDown()) {
                if (group_ids_itt->second.size() > 1) {
                    std::string key = event.getKeySym();
                    if (key == "Right" && (ids_itt != --(group_ids_itt->second.end()))) {
                        setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, *ids_itt);
                        ids_itt++;
                    } else if (key == "Left" && ids_itt != group_ids[group_ids_itt->first].begin()) {
                        setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, *ids_itt);
                        ids_itt--;
                    } else if (key == "Up") {
                        toggle_opacity(*ids_itt);
                    }

                    double current_point_size;
                    getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, current_point_size,
                                                     *ids_itt);
                    if (current_point_size == 1)
                        setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, *ids_itt);
                }
            }
        }
    }
}

void
PointCloudGroupVisualizer::addIdPointCloud(PointCloudT::Ptr &pc, std::string id, std::string group_id, int r, int g, int b) {
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(pc, r, g, b);
    addPointCloud(pc, color_handler, id);


    if (group_id.empty()) { group_id = id; }

    group_ids[group_id].emplace_back(id);
    group_color[group_id] = std::array<double, 3>{r / 255.0, g / 255.0, b / 255.0};
}

void PointCloudGroupVisualizer::addIdPointCloud(PointCloudT::Ptr &pc,
                                                const pcl::visualization::PointCloudColorHandler<PointT> &color_handler,
                                                std::string id, std::string group_id) {
    addPointCloud(pc, color_handler, id);

    if (group_id.empty()) { group_id = id; }

    group_ids[group_id].emplace_back(id);
    group_color[group_id] = std::array<double, 3>{1.0, 1.0, 1.0};
}

void PointCloudGroupVisualizer::clear() {
    removeAllPointClouds();
    group_ids.clear();
    group_color.clear();
}

void PointCloudGroupVisualizer::clear(std::string group) {

    if (group_ids.find(group) != group_ids.end()) {
        for (auto &id:group_ids[group]) {
            removePointCloud(id);
        }
        group_ids.erase(group);
        group_color.erase(group);
    }

}

