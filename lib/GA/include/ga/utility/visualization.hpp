#ifndef MASTER_VISUALIZATION_HPP
#define MASTER_VISUALIZATION_HPP

#include <ga/ga.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <map>


class CustomVisualizer {
public:
    pcl::visualization::PCLVisualizer pclVisualizer;
    std::map<std::string, std::vector<std::string>> group_ids;
    std::map<std::string, std::array<double, 3>> group_color;
    std::map<std::string, std::vector<std::string>>::iterator ids_itt;
    std::map<std::string, std::vector<std::string>>::iterator ids_itt_max;

    int font_size = 16;

    CustomVisualizer();

    void addPointCloud(PointCloudT::Ptr &pc, std::string id, std::string group_id = "", int r = 255, int g = 255,
                       int b = 255);

    void spin();

    void keyboardCallback(const pcl::visualization::KeyboardEvent &event, void *viewer_void);

    void toggle_opacity(std::string group_id);

    void change_pt_size(std::string group_id, int change = 1);

    void clear();

    void clear(std::string group);


private:
    std::string text_id = "selected_group_text";

    void update_text();
};


#endif //MASTER_VISUALIZATION_HPP
