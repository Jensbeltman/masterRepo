#ifndef MASTER_VISUALIZATION_HPP
#define MASTER_VISUALIZATION_HPP

#include <ga/ga.hpp>
#include <pcl/visualization/pcl_visualizer.h>

class GAResultVis{
public:
    GA* ga;
    pcl::visualization::PCLVisualizer vis;

    const std::string pc_id = "pc";
    std::vector<std::string> accepted_ids;
    std::vector<std::string> rejected_ids;
    GAResultVis(GA *ga);

    void vis_result();
    void keyboardCallback(const pcl::visualization::KeyboardEvent &event, void* viewer_void);

private:
    void  flip_opacity(std::vector<std::string> &ids);
    void  flip_opacity(std::string ids);
};


#endif //MASTER_VISUALIZATION_HPP
