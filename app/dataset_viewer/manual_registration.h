#ifndef MASTER_MANUAL_REGISTRATION_VIEWER_H
#define MASTER_MANUAL_REGISTRATION_VIEWER_H

#include <ui_manual_registration.h>

// Qt
#include <QMainWindow>
#include <QMutex>
#include <QTimer>

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

// Custom visualizer



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

    ~ManualRegistration() {}

    void setSrcCloud(PointCloudT::Ptr cloud_src) {
        cloud_src_ = cloud_src;
        cloud_src_present_ = true;
    }

    void setResolution(double res) {
        res_ = res;
    }

    void setDstCloud(PointCloudT::Ptr cloud_dst) {
        cloud_dst_ = cloud_dst;
        cloud_dst_present_ = true;
    }

    void SourcePointPickCallback(const pcl::visualization::PointPickingEvent &event, void *);

    void DstPointPickCallback(const pcl::visualization::PointPickingEvent &event, void *);

protected:
    std::shared_ptr<pcl::visualization::PCLVisualizer> vis_src_;
    std::shared_ptr<pcl::visualization::PCLVisualizer> vis_dst_;

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

    Eigen::Matrix4f transform_;

public slots:

    void calculatePressed();

    void clearPressed();

private slots:

    void timeoutSlot();

};

#endif //MASTER_MANUAL_REGISTRATION_VIEWER_H