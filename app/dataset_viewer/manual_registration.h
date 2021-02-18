#ifndef MASTER_MANUAL_REGISTRATION_VIEWER_H
#define MASTER_MANUAL_REGISTRATION_VIEWER_H

#include <ui_manual_registration.h>


// Qt
#include <QMainWindow>
#include <QMutex>
#include <QTimer>
#include <vtkCellPicker.h>
#include <vtkPointPicker.h>

// Boost
#include <boost/thread/thread.hpp>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/common/common.h>
#include <pcl/common/angles.h>
#include <pcl/common/time.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>

// Visualizer dataobject and renders
#include "hypothesis_verification/visualization/point_cloud_group_visualizer.hpp"
#include "dataset/scape/ScapeDatasetObject.hpp"
#include "hypothesis_verification/evaluator/point_cloud_renderer.hpp"

//VTK
#include "vtkGenericOpenGLRenderWindow.h"


struct ManualRegistrationSettings {
    bool refine=true;
    bool robust=true;
    double point_picker_tolerance = 0.01;
};

// Useful macros

namespace Ui {
    class MainWindow;
}

class ManualRegistration : public QMainWindow {
Q_OBJECT
public:
    typedef pcl::PointXYZ PointT;
    typedef pcl::PointCloud<PointT> PointCloudT;


    ManualRegistration(QMainWindow *parent = nullptr);

    ~ManualRegistration() {save_settings();}

    ManualRegistrationSettings settings;

    void load_settings();

    void save_settings();

    void SourcePointPickCallback(const pcl::visualization::PointPickingEvent &event, void *);

    void DstPointPickCallback(const pcl::visualization::PointPickingEvent &event, void *);

    void srcKeyboardCallback(const pcl::visualization::KeyboardEvent &event, void *viewer_void);
    void dstKeyboardCallback(const pcl::visualization::KeyboardEvent &event, void *viewer_void);


    void setSrcCloud(PointCloudT::Ptr cloud_src, std::string mesh_ply_path);

    void setResolution(double res);

    void setDstCloud(PointCloudT::Ptr cloud_dst);

    void setGTs(std::vector<T4> & gts, std::string path);

    void removeGTsFromFile(std::vector<int> gt_indexes);

    void setOCs(std::vector<T4> & ocs);

    void setup();

protected:
    vtkSmartPointer<vtkRenderWindow> render_window_src_;
    vtkSmartPointer<vtkRenderWindow> render_window_dst_;
    vtkSmartPointer<vtkRenderer> renderer_src_;
    vtkSmartPointer<vtkRenderer> renderer_dst_;
    pcl::shared_ptr<PointCloudGroupVisualizer> vis_src_;
    pcl::shared_ptr<PointCloudGroupVisualizer> vis_dst_;
    std::string dst_filename;

    std::vector<T4> orig_gts_;
    std::vector<T4> new_gts_;
    std::vector<int> gts_to_be_removed;
    bool new_gt_verified = true;
    std::string ground_truth_path;
    std::string mesh_ply_path;

    PointCloudRendererPtr pointCloudRenderer;

    std::map<std::string,T4> oc_id_to_t4;


    vtkSmartPointer<vtkPointPicker> pointPicker_src;
    vtkSmartPointer<vtkPointPicker> pointPicker_dst;

    std::vector<T4> ocs_;
    PointCloudT::Ptr cloud_src_;
    PointCloudT::Ptr cloud_dst_;

    double res_;
    Ui::MainWindow *ui_;

    QTimer *vis_timer_;
    bool cloud_src_present_;
    bool cloud_src_modified_;
    bool cloud_dst_present_;

    bool cloud_dst_modified_;
    bool src_point_selected_;

    bool dst_point_selected_;
    PointT src_point_;
    PointT dst_point_;

    PointCloudT::Ptr src_pc_;
    PointCloudT::Ptr dst_pc_;
    std::vector<std::string> src_pc_id_;
    std::vector<std::string> dst_pc_id_;

    Eigen::Matrix4f transform_;
private:
    void update_res();

public slots:

    void calculatePressed();

    void clearPressed();

    void verifyPressed();

    void update_pick_tolerance(double pt);

    void save_anotation();

    void removeGT();

    void undoGTRemoval();

private slots:

    void timeoutSlot();

};

#endif //MASTER_MANUAL_REGISTRATION_VIEWER_H