#ifndef MASTER_POINT_CLOUD_GROUP_VISUALIZER_HPP
#define MASTER_POINT_CLOUD_GROUP_VISUALIZER_HPP

#include <ga/ga.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <map>


// Visualizaiton point
struct PCVNode {
    std::string id;
    std::array<double, 3> rgb;
};
typedef std::shared_ptr<PCVNode> PCVNodePtr;

// Visualization point cloud cloud
struct PCVGroup {
    std::string id;
    std::shared_ptr<PCVGroup> parent_group = nullptr;
    std::vector<PCVNodePtr> nodes;
    std::vector<std::shared_ptr<PCVGroup>> groups;
    std::vector<PCVNodePtr>::iterator selected_node = nodes.begin();
    std::vector<std::shared_ptr<PCVGroup>>::iterator selected_group = groups.begin();

    // Color og first node, defaults to white if none exist.
    void reset_group_itt(){selected_group=groups.begin();};
    void reset_node_itt(){selected_node=nodes.begin();};
    void push_back_group(std::shared_ptr<PCVGroup> group){groups.emplace_back(group);reset_group_itt();};
    void push_back_node(PCVNodePtr node){nodes.emplace_back(node);reset_node_itt();};
    std::array<double, 3> get_rgb() {
        if (nodes.empty())
            return std::array<double, 3>{1.0, 1.0, 1.0};
        else
            return nodes.front()->rgb;
    }
};

typedef std::shared_ptr<PCVGroup> PCVGroupPtr;


//find pcv group by id
PCVGroupPtr find_pcv_group_id(PCVGroupPtr topPCVGroup, std::string id);

//find pcv node by id
PCVNodePtr find_pcv_node_id(PCVGroupPtr &topPCVGroup, std::string id, bool recursive = true);

PCVGroupPtr find_pcv_group_containing_id(PCVGroupPtr topPCVGroup, std::string id);


class PointCloudGroupVisualizer : public pcl::visualization::PCLVisualizer {
public:
    PCVGroupPtr pcv_root;
    PCVGroupPtr current_group;

    std::vector<std::string>::iterator ids_itt;

    int font_size = 16;

    PointCloudGroupVisualizer(const std::string &name = "Custom Vizualizer", const bool create_interactor = true);

    PointCloudGroupVisualizer(int &argc, char **argv, const std::string &name = "",
                              pcl::visualization::PCLVisualizerInteractorStyle *style = pcl::visualization::PCLVisualizerInteractorStyle::New(),
                              const bool create_interactor = true);

    PointCloudGroupVisualizer(vtkSmartPointer<vtkRenderer> ren, vtkSmartPointer<vtkRenderWindow> wind,
                              const std::string &name = "", const bool create_interactor = true);

    PointCloudGroupVisualizer(int &argc, char **argv, vtkSmartPointer<vtkRenderer> ren,
                              vtkSmartPointer<vtkRenderWindow> wind,
                              const std::string &name = "",
                              pcl::visualization::PCLVisualizerInteractorStyle *style = pcl::visualization::PCLVisualizerInteractorStyle::New(),
                              const bool create_interactor = true);

    PCVGroupPtr getInsertionGroup(std::string group_id);

    void addIdPointCloud(PointCloudT::Ptr &pc, std::string id, std::string group_id = "", int r = 255, int g = 255,
                         int b = 255);

    void addIdPointCloud(PointCloudT::Ptr &pc, const pcl::visualization::PointCloudColorHandler<PointT> &color_handler,
                         std::string id, std::string group_id = "");

    void updateSelector();

    void custom_spin();

    void keyboardCallback(const pcl::visualization::KeyboardEvent &event, void *viewer_void);

    void toggle_opacity(PCVNodePtr node);

    void toggle_group_opacity(PCVGroupPtr group);

    void change_pt_size(PCVGroupPtr group, int change = 1);

    void clear();

    void clear(std::string group);

    void update_text();

    //find pcv group by id
    PCVGroupPtr find_pcv_group_id(std::string group_id);

    //find pcv group by id
    PCVGroupPtr find_pcv_group_containing_id(std::string id);

    //find pcv node by id
    PCVNodePtr find_pcv_node_id(std::string id, bool recursive = true);

    //remove pcv group
    void remove_pcv_group(std::string group_id);

    //remove pcv node
    void remove_pcv_node(std::string id);


private:
    void init();

    const std::string current_group_text_id = "current_group_text_id";
    const std::string selected_group_text_id = "selected_group_text_id";
    const std::string selected_node_text_id = "selected_node_text_id";

    std::string current_group_text = "current_group_text";
    std::string selected_group_text = "selected_group_text";
    std::string selected_node_text = "selected_node_text";

    const std::string lsym = "< ";
    const std::string rsym = " >";

    std::array<double,3> current_group_text_color =   std::array<double,3>{1,1,1};
    std::array<double,3> selected_group_text_color =   std::array<double,3>{1,1,1};
    std::array<double,3> selected_node_text_color =   std::array<double,3>{1,1,1};

    //Itterator Utilities
    template <typename Iter, typename Cont>
    bool is_last(Iter  iter, const Cont& cont)
    {
        return (iter != cont.end()) && (next(iter) == cont.end());
    }
    template <typename Iter, typename Cont>
    bool is_first(Iter  iter, const Cont& cont)
    {
        return (iter == cont.begin());
    }
    template <typename Iter>
    Iter  next(Iter iter)
    {
        return ++iter;
    }
    template <typename Iter>

    Iter  prev(Iter iter)
    {
        return --iter;
    }

};


#endif //MASTER_POINT_CLOUD_GROUP_VISUALIZER_HPP
