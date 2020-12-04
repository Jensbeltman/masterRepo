#ifndef MASTER_VISUALIZATION_HPP
#define MASTER_VISUALIZATION_HPP

#include <ga/ga.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <map>


class CustomVisualizer : public pcl::visualization::PCLVisualizer {
public:
    std::map<std::string, std::vector<std::string>> group_ids;
    std::map<std::string, std::array<double, 3>> group_color;
    std::map<std::string, std::vector<std::string>>::iterator group_ids_itt;
    std::vector<std::string>::iterator ids_itt;

    int font_size = 16;

    CustomVisualizer(const std::string &name = "Custom Vizualizer", const bool create_interactor = true);

    CustomVisualizer(int &argc, char **argv, const std::string &name = "",
                     pcl::visualization::PCLVisualizerInteractorStyle *style = pcl::visualization::PCLVisualizerInteractorStyle::New(),
                     const bool create_interactor = true);

    CustomVisualizer(vtkSmartPointer<vtkRenderer> ren, vtkSmartPointer<vtkRenderWindow> wind,
                     const std::string &name = "", const bool create_interactor = true);

    CustomVisualizer(int &argc, char **argv, vtkSmartPointer<vtkRenderer> ren, vtkSmartPointer<vtkRenderWindow> wind,
                     const std::string &name = "",
                     pcl::visualization::PCLVisualizerInteractorStyle *style = pcl::visualization::PCLVisualizerInteractorStyle::New(),
                     const bool create_interactor = true);

    void addIdPointCloud(PointCloudT::Ptr &pc, std::string id, std::string group_id = "", int r = 255, int g = 255,
                         int b = 255);

    void addIdPointCloud(PointCloudT::Ptr &pc, const pcl::visualization::PointCloudColorHandler<PointT> &color_handler,
                         std::string id, std::string group_id = "");

    void updateSelector();

    void custom_spin();

    void keyboardCallback(const pcl::visualization::KeyboardEvent &event, void *viewer_void);

    void toggle_opacity(std::string id);

    void toggle_group_opacity(std::string group_id);

    void change_pt_size(std::string group_id, int change = 1);

    void clear();

    void clear(std::string group);

    void update_text();


private:
    void init();

    std::string text_id = "selected_group_text";


};


#endif //MASTER_VISUALIZATION_HPP
