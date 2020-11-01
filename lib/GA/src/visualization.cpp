#include "ga/utility/visualization.hpp"
#include <ga/genetic_evaluator/GeneticEvaluatorObjectCandidates.hpp>
#include <ga/typedefinitions.hpp>

#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>


CustomVisualizer::CustomVisualizer() {
    pclVisualizer.registerKeyboardCallback(&CustomVisualizer::keyboardCallback, *this);
}

void CustomVisualizer::spin() {
    ids_itt = group_ids.begin();
    ids_itt_max = group_ids.end();
    ids_itt_max--;

    // Setup selection text
    pclVisualizer.spinOnce(); // Spin once to make such that the camera is setup
    std::array<double, 3> rgb = group_color[ids_itt->first];
    pcl::visualization::Camera camera;
    pclVisualizer.getCameraParameters(camera);
    if (pclVisualizer.contains(text_id)) {
        update_text();
    } else {

        pclVisualizer.addText(ids_itt->first, static_cast<int>(camera.window_size[0] / 2),
                              static_cast<int>(camera.window_size[1] - font_size * 1.5), font_size, rgb[0], rgb[1],
                              rgb[2],
                              text_id);
        change_pt_size(ids_itt->first, 1);
    }
    pclVisualizer.spin();

}

void CustomVisualizer::update_text() {
    std::array<double, 3> rgb = group_color[ids_itt->first];
    pcl::visualization::Camera camera;
    pclVisualizer.getCameraParameters(camera);
    pclVisualizer.updateText(ids_itt->first, camera.window_size[0] / 2, camera.window_size[1] - font_size * 1.5,
                             font_size,
                             rgb[0], rgb[1], rgb[2], text_id);
}

void CustomVisualizer::toggle_opacity(std::string group_id) {

    if (!group_ids[group_id].empty()) {
        double current_opacity;
        pclVisualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, current_opacity,
                                                       group_ids[group_id][0]);
        current_opacity = (current_opacity < 1.0) ? 1.0 : 0.0;
        for (auto &id:group_ids[group_id]) {
            pclVisualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, current_opacity,
                                                           id);
        }
    }
}

void CustomVisualizer::change_pt_size(std::string group_id, int change) {
    if (!group_ids[group_id].empty()) {
        double current_point_size;
        pclVisualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                       current_point_size,
                                                       group_ids[group_id][0]);
        for (auto &id:group_ids[group_id]) {
            pclVisualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                           current_point_size + change, id);
        }
    }
}

void CustomVisualizer::keyboardCallback(const pcl::visualization::KeyboardEvent &event, void *viewer_void) {
    if (event.keyDown()) {
        std::string key = event.getKeySym();
        if (key == "Right" && (ids_itt != ids_itt_max)) {
            ids_itt++;
            update_text();
        } else if (key == "Left" && ids_itt != group_ids.begin()) {
            ids_itt--;
            update_text();
        } else if (key == "Up") {
            toggle_opacity(ids_itt->first);
        }
    }
}

void
CustomVisualizer::addPointCloud(PointCloudT::Ptr &pc, std::string id, std::string group_id, int r, int g, int b) {
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(pc, r, g, b);
    pclVisualizer.addPointCloud(pc, color, id);


    if (group_id.empty()) { group_id = id; }

    group_ids[group_id].emplace_back(id);
    group_color[group_id] = std::array<double, 3>{r / 255.0, g / 255.0, b / 255.0};
}

void CustomVisualizer::clear() {
    pclVisualizer.removeAllPointClouds();
    group_ids.clear();
    group_color.clear();
}

void CustomVisualizer::clear(std::string group) {

    if (group_ids.find(group) != group_ids.end()) {
        for (auto &id:group_ids[group]) {
            pclVisualizer.removePointCloud(id);
        }
        group_ids.erase(group);
        group_color.erase(group);
    }

}
