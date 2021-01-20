#include "ga/visualization/point_cloud_group_visualizer.hpp"
#include <ga/genetic_evaluator/GeneticEvaluatorObjectCandidates.hpp>
#include <ga/typedefinitions.hpp>

#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>
#include <thread>
#include <chrono>
#include <vtkRenderWindow.h>
#include <vtkCamera.h>
#include <vtkTextActor.h>

using namespace std::chrono_literals;


void PointCloudGroupVisualizer::init() {
    registerKeyboardCallback(&PointCloudGroupVisualizer::keyboardCallback, *this);
    setShowFPS(false);
    pcv_root = std::shared_ptr<PCVGroup>(new PCVGroup{"Visualization Root", nullptr});
    current_group = pcv_root;

    // Text setup
    std::vector<std::pair<std::string, vtkSmartPointer<vtkTextActor>>> id_actors;
    id_actors.push_back(std::make_pair(current_group_text_id, vtkSmartPointer<vtkTextActor>::New()));
    id_actors.push_back(std::make_pair(selected_group_text_id, vtkSmartPointer<vtkTextActor>::New()));
    id_actors.push_back(std::make_pair(selected_node_text_id, vtkSmartPointer<vtkTextActor>::New()));

    pcl::visualization::ShapeActorMapPtr shape_actor_map = getShapeActorMap();
    for (auto &ia:id_actors) {
        vtkSmartPointer<vtkTextProperty> tprop = ia.second->GetTextProperty();
        tprop->SetFontSize(font_size);
        tprop->SetFontFamilyToArial();
        tprop->SetJustificationToCentered();
        tprop->BoldOff();
        tprop->SetColor(1, 1, 1);
        (*shape_actor_map)[ia.first] = ia.second;
    }

    // Add text to renderers
    auto rens = getRendererCollection();
    rens->InitTraversal();
    vtkRenderer *renderer = nullptr;
    while ((renderer = rens->GetNextItem())) {
        for (auto &ia:id_actors) {
            renderer->AddActor(ia.second);
        }
    }


};


PointCloudGroupVisualizer::PointCloudGroupVisualizer(const std::string &name, const bool create_interactor)
        : PCLVisualizer(name,
                        create_interactor) {
    init();
}

PointCloudGroupVisualizer::PointCloudGroupVisualizer(int &argc, char **argv, const std::string &name,
                                                     pcl::visualization::PCLVisualizerInteractorStyle *style,
                                                     const bool create_interactor) : PCLVisualizer(argc, argv, name,
                                                                                                   style,
                                                                                                   create_interactor) {
    init();
}

PointCloudGroupVisualizer::PointCloudGroupVisualizer(vtkSmartPointer<vtkRenderer> ren,
                                                     vtkSmartPointer<vtkRenderWindow> wind,
                                                     const std::string &name, const bool create_interactor)
        : PCLVisualizer(ren, wind,
                        name,
                        create_interactor) {
    init();
}

PointCloudGroupVisualizer::PointCloudGroupVisualizer(int &argc, char **argv, vtkSmartPointer<vtkRenderer> ren,
                                                     vtkSmartPointer<vtkRenderWindow> wind, const std::string &name,
                                                     pcl::visualization::PCLVisualizerInteractorStyle *style,
                                                     const bool create_interactor) : PCLVisualizer(argc, argv, ren,
                                                                                                   wind, name, style,
                                                                                                   create_interactor) {
    init();
}

void PointCloudGroupVisualizer::updateSelector() {
    spinOnce();
    update_text();
    if (!(current_group->nodes.empty())) {
        double current_point_size;
        getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, current_point_size,
                                         current_group->selected_node->get()->id);
        if (current_point_size != 3)
            setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3,
                                             current_group->selected_node->get()->id);
    }
}

void PointCloudGroupVisualizer::custom_spin() {
    update_text();
    spin();
}


void PointCloudGroupVisualizer::update_text() {

    pcl::visualization::Camera camera;
    getCameraParameters(camera);
    std::array<double, 3> rgb;


    if (current_group != nullptr) {
        current_group_text = current_group->id;
        current_group_text_color = current_group->get_rgb();
    }

    if (!current_group->groups.empty()) {
        PCVGroupPtr selected_group = *(current_group->selected_group);
        bool isBegin = is_first(current_group->selected_group, current_group->groups);
        bool isEnd = is_last(current_group->selected_group, current_group->groups);

        if (isBegin && isEnd)
            selected_group_text = selected_group->id;
        if (!isBegin && !isEnd)
            selected_group_text = lsym + selected_group->id + rsym;
        if (isBegin && !isEnd)
            selected_group_text = selected_group->id + rsym;
        if (!isBegin && isEnd)
            selected_group_text = lsym + selected_group->id;

        selected_group_text_color = selected_group->get_rgb();
    } else {
        selected_group_text = "None";
        selected_group_text_color[0] = 0.5;
        selected_group_text_color[1] = 0.5;
        selected_group_text_color[2] = 0.5;
    }

    if (!current_group->nodes.empty()) {
        PCVNodePtr selected_node = *(current_group->selected_node);
        bool isBegin = is_first(current_group->selected_node, current_group->nodes);
        bool isEnd = is_last(current_group->selected_node, current_group->nodes);

        if (isBegin && isEnd)
            selected_node_text = selected_node->id;
        if (!isBegin && !isEnd)
            selected_node_text = lsym + selected_node->id + rsym;
        if (isBegin && !isEnd)
            selected_node_text = selected_node->id + rsym;
        if (!isBegin && isEnd)
            selected_node_text = lsym + selected_node->id;

        selected_node_text_color = selected_node->rgb;
    } else {
        selected_node_text = "None";
        selected_node_text_color[0] = 0.5;
        selected_node_text_color[1] = 0.5;
        selected_node_text_color[2] = 0.5;
    }


    updateText(current_group_text, camera.window_size[0] / 2, camera.window_size[1] - font_size * 1.5,
               current_group_text_color[0], current_group_text_color[1], current_group_text_color[2],
               current_group_text_id);
    updateText(selected_group_text, camera.window_size[0] / 2, camera.window_size[1] - font_size * 1.5 * 2,
               selected_group_text_color[0], selected_group_text_color[1], selected_group_text_color[2],
               selected_group_text_id);

    updateText(selected_node_text, camera.window_size[0] / 2, camera.window_size[1] - font_size * 1.5 * 3,
               selected_node_text_color[0], selected_node_text_color[1], selected_node_text_color[2],
               selected_node_text_id);

//    pcl::visualization::ShapeActorMapPtr shape_actor_map_ = getShapeActorMap();
//    pcl::visualization::ShapeActorMap::iterator am_it = getShapeActorMap()->find(current_group_text_id);
//    if (am_it != shape_actor_map_->end()) {
//        vtkTextActor *actor = vtkTextActor::SafeDownCast(am_it->second);
//        if (actor) {
//            vtkTextActor *actor = dynamic_cast<vtkTextActor *>((*getShapeActorMap())[current_group_text_id].Get());
//            double tsize[2];
//            actor->GetSize(getRendererCollection()->GetFirstRenderer(),tsize);
//            std::cout<<tsize[0]<<", "<<tsize[1]<<std::endl;
//        }
//    }
}

void PointCloudGroupVisualizer::toggle_opacity(PCVNodePtr node) {
    double current_opacity;
    getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, current_opacity, node->id);
    current_opacity = (current_opacity < 1.0) ? 1.0 : 0.0;
    setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, current_opacity, node->id);
}

void PointCloudGroupVisualizer::toggle_group_opacity(PCVGroupPtr topPCVGroup) {

    if (!topPCVGroup->nodes.empty()) {
        double current_opacity;
        getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, current_opacity,
                                         topPCVGroup->nodes[0]->id);
        current_opacity = (current_opacity < 1.0) ? 1.0 : 0.0;
        for (auto &node:topPCVGroup->nodes) {
            setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, current_opacity,
                                             node->id);
        }
    }

    for (auto group:topPCVGroup->groups)
        toggle_group_opacity(group);
}

void PointCloudGroupVisualizer::change_pt_size(PCVGroupPtr topPCVGroup, int change) {
    if (!topPCVGroup->nodes.empty()) {
        double current_point_size;
        getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                         current_point_size,
                                         topPCVGroup->nodes[0]->id);
        for (auto &node:current_group->nodes) {
            setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                             current_point_size + change, node->id);
        }
    }

    for (auto group:topPCVGroup->groups)
        change_pt_size(group);
}

void PointCloudGroupVisualizer::keyboardCallback(const pcl::visualization::KeyboardEvent &event, void *viewer_void) {
    if (!event.isShiftPressed()) {
        if (event.keyDown()) {
            std::string key = event.getKeySym();

            if (!current_group->groups.empty()) {
                if (key == "Right" && !is_last(current_group->selected_group, current_group->groups)) {
                    ++current_group->selected_group;
                } else if (key == "Left" && !is_first(current_group->selected_group, current_group->groups)) {
                    --current_group->selected_group;
                } else if (key == "v") {
                    toggle_group_opacity(*(current_group->selected_group));
                }
            }


            if (key == "Up") {
                if (current_group->parent_group != nullptr) {
                    if (!current_group->nodes.empty()) {
                        setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,
                                                         current_group->selected_node->get()->id);
                    }
                    current_group = current_group->parent_group;
                }
            } else if (key == "Down") {
                if (!(current_group->groups.empty())) {
                    if (!current_group->nodes.empty()) {
                        setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,
                                                         current_group->selected_node->get()->id);
                    }
                    current_group = *current_group->selected_group;
                }
            }
            updateSelector();
        }
    } else {
        if (event.keyDown()) {
            std::string key = event.getKeySym();

            if (!current_group->nodes.empty()) {
                if (key == "Right" && !is_last(current_group->selected_node, current_group->nodes)) {
                    setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,
                                                     current_group->selected_node->get()->id);

                    ++current_group->selected_node;
                } else if (key == "Left" && !is_first(current_group->selected_node, current_group->nodes)) {
                    setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,
                                                     current_group->selected_node->get()->id);
                    --current_group->selected_node;
                } else if (key == "V") {
                    toggle_opacity(*(current_group->selected_node));
                }
            }

            updateSelector();
        }
    }
}


// Group id should be on typical folder format with / sperating groups. This function return the last group in a path and creat the groups if they dont exist.
PCVGroupPtr PointCloudGroupVisualizer::getInsertionGroup(std::string group_id) {
    if (group_id[0] != '/') { group_id = "/" + group_id; }

    PCVGroupPtr found_group = nullptr;//
    PCVGroupPtr last_group = pcv_root;
    size_t pos = 0;
    std::string token;
    int found_groups = 0;
    const std::string delimiter = "/";

    while (group_id.size() > 0) {
        pos = group_id.find(delimiter);
        token = group_id.substr(0, pos);

        if (pos != group_id.npos)
            group_id.erase(0, pos + delimiter.length());
        else
            group_id.erase(0, pos);

        if (token != "") {
            found_group = ::find_pcv_group_id(last_group, token);
            if (found_group != nullptr) {
                last_group = found_group;
            } else {
                last_group->push_back_group(std::shared_ptr<PCVGroup>(new PCVGroup{token, last_group}));
                last_group = last_group->groups.back();
            }
        }
    }

    return last_group;
}

void
PointCloudGroupVisualizer::addIdPointCloud(PointCloudT::Ptr &pc, std::string id, std::string group_id, int r, int g,
                                           int b) {
    if (!contains(id)) {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(pc, r, g, b);
        addPointCloud(pc, color_handler, id);

        if (group_id.empty()) { group_id = "/" + id; }
        PCVGroupPtr insertionGroup = getInsertionGroup(group_id);

        insertionGroup->push_back_node(
                std::shared_ptr<PCVNode>(new PCVNode{id, std::array<double, 3>{r / 255.0, g / 255.0, b / 255.0}}));

    } else {
        std::cout << "ID " + id + " already exist in visualizer, point cloud not added" << std::endl;
    }


}

void PointCloudGroupVisualizer::addIdPointCloud(PointCloudT::Ptr &pc,
                                                const pcl::visualization::PointCloudColorHandler<PointT> &color_handler,
                                                std::string id, std::string group_id) {
    if (!contains(id)) {
        addPointCloud(pc, color_handler, id);

        if (group_id.empty()) { group_id = "/" + id; }
        PCVGroupPtr insertionGroup = getInsertionGroup(group_id);

        insertionGroup->push_back_node(std::shared_ptr<PCVNode>(new PCVNode{id, std::array<double, 3>{1, 1, 1}}));
    } else {
        std::cout << "ID " + id + " already exist in visualizer, point cloud not added" << std::endl;
    }
}

void PointCloudGroupVisualizer::clear() {
    removeAllPointClouds();
    pcv_root->groups.clear();
}

void PointCloudGroupVisualizer::clear(std::string group) {

    auto found_group = ::find_pcv_group_id(pcv_root, group);
    if (found_group != nullptr || found_group != pcv_root) {
        for (auto &node:found_group->nodes) {
            removePointCloud(node->id);
        }
        for (auto &group:found_group->groups) {
            clear(group->id);
        }
    }

    auto &containing_vector = found_group->parent_group->groups;
    containing_vector.erase(std::find(containing_vector.begin(), containing_vector.end(), found_group));
}

PCVGroupPtr PointCloudGroupVisualizer::find_pcv_group_id(std::string id) {
    return ::find_pcv_group_id(pcv_root, id);
}

PCVNodePtr PointCloudGroupVisualizer::find_pcv_node_id(std::string id, bool recursive) {
    return ::find_pcv_node_id(pcv_root, id, recursive);
}

void PointCloudGroupVisualizer::remove_pcv_group(std::string group_id) {
    clear(group_id);
}

void PointCloudGroupVisualizer::remove_pcv_node(std::string id) {
    if (contains(id)) { // if PCL visualizer contains id
        removePointCloud(id);
        auto group_with_id = find_pcv_group_containing_id(id);
        group_with_id->nodes.erase(std::find_if(group_with_id->nodes.begin(),group_with_id->nodes.end(),[&id](PCVNodePtr np){return np->id==id;}));
    }
}

PCVGroupPtr PointCloudGroupVisualizer::find_pcv_group_containing_id(std::string id) {
    return ::find_pcv_group_containing_id(pcv_root, id);
}


PCVGroupPtr find_pcv_group_id(PCVGroupPtr topPCVGroup, std::string id) {
    if (topPCVGroup->id == id)
        return topPCVGroup;

    for (auto &group:topPCVGroup->groups)
        if (find_pcv_group_id(group, id) != nullptr)
            return group;

    return nullptr;
}

PCVNodePtr find_pcv_node_id(PCVGroupPtr &topPCVGroup, std::string id, bool recursive) {
    for (auto &node:topPCVGroup->nodes)
        if (node->id == id)
            return node;

    if (!recursive)
        return nullptr;

    PCVNodePtr rres = std::make_shared<PCVNode>();
    for (auto &group:topPCVGroup->groups) {
        rres = find_pcv_node_id(group, id);
        if (rres != nullptr)
            return rres;
    }

    return nullptr;
}

PCVGroupPtr find_pcv_group_containing_id(PCVGroupPtr topPCVGroup, std::string id) {
    for (auto &node:topPCVGroup->nodes)
        if (node->id == id)
            return topPCVGroup;

    PCVNodePtr rres = std::make_shared<PCVNode>();
    for (auto &group:topPCVGroup->groups) {
        rres = find_pcv_node_id(group, id);
        if (rres != nullptr)
            return group;
    }
    return nullptr;
}
