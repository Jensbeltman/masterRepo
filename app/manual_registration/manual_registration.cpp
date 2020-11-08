#include "manual_registration.h"

// Qt
#include <QApplication>
#include <QEvent>
#include <QMessageBox>
#include <QMutexLocker>
#include <QObject>

// VTK
#include <vtkRenderWindow.h>
#include <vtkRendererCollection.h>
#include <vtkCamera.h>

// PCL
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/icp.h>
#include <pcl/io/ply_io.h>

// filesystem
#include <filesystem>

using namespace pcl;
using namespace std;

//////////////////////////////////////////////////////////////////////////////////////////////////////////
ManualRegistration::ManualRegistration() {
    // Initialize bogus
    res_ = 0.001;
    cloud_src_present_ = false;
    cloud_dst_present_ = false;
    src_point_selected_ = false;
    dst_point_selected_ = false;

    //Create a timer
    vis_timer_ = new QTimer(this);
    vis_timer_->start(5);//5ms

    connect(vis_timer_, SIGNAL (timeout()), this, SLOT (timeoutSlot()));

    ui_ = new Ui::MainWindow;
    ui_->setupUi(this);

    this->setWindowTitle("PCL Manual Registration");

    // Set up the source window
    vis_src_.reset(new pcl::visualization::PCLVisualizer("", false));
    ui_->qvtk_widget_src->SetRenderWindow(vis_src_->getRenderWindow());
    vis_src_->setupInteractor(ui_->qvtk_widget_src->GetInteractor(), ui_->qvtk_widget_src->GetRenderWindow());
    vis_src_->getInteractorStyle()->setKeyboardModifier(pcl::visualization::INTERACTOR_KB_MOD_SHIFT);
    ui_->qvtk_widget_src->update();

    vis_src_->registerPointPickingCallback(&ManualRegistration::SourcePointPickCallback, *this);

    // Set up the destination window
    vis_dst_.reset(new pcl::visualization::PCLVisualizer("", false));
    ui_->qvtk_widget_dst->SetRenderWindow(vis_dst_->getRenderWindow());
    vis_dst_->setupInteractor(ui_->qvtk_widget_dst->GetInteractor(), ui_->qvtk_widget_dst->GetRenderWindow());
    vis_dst_->getInteractorStyle()->setKeyboardModifier(pcl::visualization::INTERACTOR_KB_MOD_SHIFT);
    ui_->qvtk_widget_dst->update();

    vis_dst_->registerPointPickingCallback(&ManualRegistration::DstPointPickCallback, *this);


    // Connect all buttons
    //  connect (ui_->confirmSrcPointButton, SIGNAL(clicked()), this, SLOT(confirmSrcPointPressed()));
    //  connect (ui_->confirmDstPointButton, SIGNAL(clicked()), this, SLOT(confirmDstPointPressed()));
    connect(ui_->calculateButton, SIGNAL(clicked()), this, SLOT(calculatePressed()));
    connect(ui_->clearButton, SIGNAL(clicked()), this, SLOT(clearPressed()));
    //  connect (ui_->orthoButton, SIGNAL(stateChanged(int)), this, SLOT(orthoChanged(int)));
    //  connect (ui_->applyTransformButton, SIGNAL(clicked()), this, SLOT(applyTransformPressed()));
    //  connect (ui_->refineButton, SIGNAL(clicked()), this, SLOT(refinePressed()));
    //  connect (ui_->undoButton, SIGNAL(clicked()), this, SLOT(undoPressed()));
    //  connect (ui_->safeButton, SIGNAL(clicked()), this, SLOT(safePressed()));

    cloud_src_modified_ = true; // first iteration is always a new pointcloud
    cloud_dst_modified_ = true;

    src_pc_.reset(new Cloud);
    dst_pc_.reset(new Cloud);
}

void ManualRegistration::SourcePointPickCallback(const pcl::visualization::PointPickingEvent &event, void *) {
    // Check to see if we got a valid point. Early exit.
    int idx = event.getPointIndex();
    if (idx == -1)
        return;

    // Get the point that was picked
    event.getPoint(src_point_.x, src_point_.y, src_point_.z);
    PCL_INFO ("Src Window: Clicked point %d with X:%f Y:%f Z:%f\n", idx, src_point_.x, src_point_.y, src_point_.z);
    src_point_selected_ = true;

    if (src_point_selected_) {
        src_pc_->points.push_back(src_point_);
        PCL_INFO ("Selected %d source points\n", src_pc_->points.size());
        src_point_selected_ = false;
        src_pc_->width = src_pc_->points.size();

        std::ostringstream oss;
        oss << src_pc_->size();
        vis_src_->addSphere<PointT>(src_point_, 5 * res_, 0, 1, 0, "sphere_src_" + oss.str());

/*        if(vis_src_->getCloudActorMap()->find("src_pc") == vis_src_->getCloudActorMap()->end()) {
            // First time
            vis_src_->addPointCloud(src_pc_,
                    pcl::visualization::PointCloudColorHandlerCustom<PointT>(src_pc_, 255, 0, 0),
                    "src_pc");
            vis_src_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 25, "src_pc");
        } else {
            vis_src_->updatePointCloud(src_pc_,
                    pcl::visualization::PointCloudColorHandlerCustom<PointT>(src_pc_, 255, 0, 0),
                    "src_pc");
        }*/
    } else {
        PCL_INFO ("Please select a point in the source window first\n");
    }
}

void ManualRegistration::DstPointPickCallback(const pcl::visualization::PointPickingEvent &event, void *) {
    // Check to see if we got a valid point. Early exit.
    int idx = event.getPointIndex();
    if (idx == -1)
        return;

    // Get the point that was picked
    event.getPoint(dst_point_.x, dst_point_.y, dst_point_.z);
    PCL_INFO ("Dst Window: Clicked point %d with X:%f Y:%f Z:%f\n", idx, dst_point_.x, dst_point_.y, dst_point_.z);
    dst_point_selected_ = true;

    if (dst_point_selected_) {
        dst_pc_->points.push_back(dst_point_);
        PCL_INFO ("Selected %d destination points\n", dst_pc_->points.size());
        dst_point_selected_ = false;
        dst_pc_->width = dst_pc_->points.size();

        std::ostringstream oss;
        oss << dst_pc_->size();
        vis_dst_->addSphere<PointT>(dst_point_, 5 * res_, 1, 0, 0, "sphere_dst_" + oss.str());

/*        if(vis_dst_->getCloudActorMap()->find("dst_pc") == vis_dst_->getCloudActorMap()->end()) {
            // First time
            vis_dst_->addPointCloud(dst_pc_,
                    pcl::visualization::PointCloudColorHandlerCustom<PointT>(dst_pc_, 255, 0, 0),
                    "dst_pc");
            vis_dst_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 25, "dst_pc");
        } else {
            vis_dst_->updatePointCloud(dst_pc_,
                    pcl::visualization::PointCloudColorHandlerCustom<PointT>(dst_pc_, 255, 0, 0),
                    "dst_pc");
        }*/
    } else {
        PCL_INFO ("Please select a point in the destination window first\n");
    }
}

void ManualRegistration::calculatePressed() {
    if (dst_pc_->points.size() != src_pc_->points.size()) {
        PCL_INFO ("You haven't selected an equal amount of points, please do so!\n");
        QMessageBox::warning(this,
                             QString("Warning"),
                             QString("You haven't selected an equal amount of points, please do so!"));
        return;
    }

    const double voxel_size = 5 * res_;
    const double inlier_threshold_ransac = 2 * voxel_size;
    const double inlier_threshold_icp = 2 * voxel_size;

    CloudPtr cloud_src_ds(new Cloud);
    CloudPtr cloud_dst_ds(new Cloud);
    if (ui_->robustBox->isChecked() || ui_->refineBox->isChecked()) {
        PCL_INFO("Downsampling point clouds with a voxel size of %f...\n", voxel_size);
        pcl::VoxelGrid<PointT> grid;
        grid.setLeafSize(voxel_size, voxel_size, voxel_size);
        grid.setInputCloud(cloud_src_);
        grid.filter(*cloud_src_ds);
        grid.setInputCloud(cloud_dst_);
        grid.filter(*cloud_dst_ds);
    }

    if (ui_->robustBox->isChecked()) {
/*              PCL_INFO("Generating correspondences for the downsampled models...\n");
              pcl::search::KdTree<PointT> ssrc, sdst;
              ssrc.setInputCloud(cloud_src_ds);
              sdst.setInputCloud(cloud_dst_ds);

              pcl::Correspondences corr_ds(src_pc_->size());
              for(size_t i = 0; i < src_pc_->size(); ++i) {
                  std::vector<int> idx(1);
                  std::vector<float> distsq(1);

                  ssrc.nearestKSearch(src_pc_->points[corr_ds[i].index_query], 1, idx, distsq);
                  corr_ds[i].index_query = idx[0];

                  sdst.nearestKSearch(dst_pc_->points[corr_ds[i].index_match], 1, idx, distsq);
                  corr_ds[i].index_match = idx[0];
              }*/
        pcl::Correspondences corr(src_pc_->size());
        for (size_t i = 0; i < src_pc_->size(); ++i)
            corr[i].index_query = corr[i].index_match = i;

        PCL_INFO("Computing pose using RANSAC with an inlier threshold of %f...\n", inlier_threshold_ransac);
        pcl::registration::CorrespondenceRejectorSampleConsensus<PointT> sac;
        //      sac.setInputSource(cloud_src_ds);
        //      sac.setInputTarget(cloud_dst_ds);
        sac.setInputSource(src_pc_);
        sac.setInputTarget(dst_pc_);
        sac.setInlierThreshold(inlier_threshold_ransac);

        pcl::Correspondences inliers;
        //      sac.getRemainingCorrespondences(corr_ds, inliers);
        sac.getRemainingCorrespondences(corr, inliers);

        // Abort if RANSAC fails
        if (sac.getBestTransformation().isIdentity()) {
            PCL_ERROR("RANSAC failed!\n");
            QMessageBox::warning(this,
                                 QString("Error"),
                                 QString("RANSAC failed!"));
            return;
        }

        transform_ = sac.getBestTransformation();
    } else {
        PCL_INFO("Computing pose using clicked correspondences...\n");
        pcl::registration::TransformationEstimationSVD<PointT, PointT> tfe;
        tfe.estimateRigidTransformation(*src_pc_, *dst_pc_, transform_);
    }

    if (ui_->refineBox->isChecked()) {

        pcl::IterativeClosestPoint<PointT, PointT> icp;
        Cloud tmp;

        PCL_INFO("Refining pose using ICP with an inlier threshold of %f...\n", inlier_threshold_icp);
        icp.setInputSource(cloud_src_ds);
        icp.setInputTarget(cloud_dst_ds);
        icp.setMaximumIterations(100);
        icp.setMaxCorrespondenceDistance(inlier_threshold_icp);
        icp.align(tmp, transform_);

        if (!icp.hasConverged()) {
            PCL_ERROR("ICP failed!\n");
            QMessageBox::warning(this,
                                 QString("Error"),
                                 QString("ICP failed!"));
            return;
        }

        PCL_INFO("Rerunning fine ICP with an inlier threshold of %f...\n", voxel_size);
        icp.setMaximumIterations(100);
        icp.setMaxCorrespondenceDistance(0.1 * inlier_threshold_icp);
        icp.align(tmp, icp.getFinalTransformation());

        if (!icp.hasConverged()) {
            PCL_ERROR("Fine ICP failed!\n");
            QMessageBox::warning(this,
                                 QString("Error"),
                                 QString("Fine ICP failed!"));
            return;
        }

        PCL_INFO("Rerunning ultra-fine ICP at full resolution with an inlier threshold of %f...\n", res_);
        icp.setInputSource(cloud_src_);
        icp.setInputTarget(cloud_dst_);
        icp.setMaximumIterations(25);
        icp.setMaxCorrespondenceDistance(res_);
        icp.align(tmp, icp.getFinalTransformation());

        if (!icp.hasConverged()) {
            PCL_ERROR("Ultra-fine ICP failed!\n");
            QMessageBox::warning(this,
                                 QString("Error"),
                                 QString("Ultra-fine ICP failed!"));
            return;
        }

        transform_ = icp.getFinalTransformation();
    }

    PCL_INFO("All done! The final refinement was done with an inlier threshold of %f, "
             "and you can expect the resulting pose to be accurate within this bound.\n", res_);

    std::cout << "Transform: " << std::endl << transform_ << std::endl;

    std::cout
            << "The transform can be used to place the source (leftmost) point cloud into the target, and thus places observations (points, poses) relative to the source camera in the target camera (rightmost). If you need the other way around, use the inverse:"
            << std::endl << transform_.inverse() << std::endl;

    CloudPtr cloud_src_aligned(new Cloud);
    pcl::transformPointCloud<PointT>(*cloud_src_, *cloud_src_aligned, transform_);

    pcl::visualization::PCLVisualizer vpose("Pose visualization. Red: scene. Green: aligned object");
    vpose.addPointCloud<PointT>(cloud_dst_,
                                pcl::visualization::PointCloudColorHandlerCustom<PointT>(cloud_dst_, 255, 0, 0),
                                "scene");
    vpose.addPointCloud<PointT>(cloud_src_aligned,
                                pcl::visualization::PointCloudColorHandlerCustom<PointT>(cloud_src_aligned, 0, 255, 0),
                                "aligned_object");
    vpose.spin();
}

void ManualRegistration::clearPressed() {
    dst_point_selected_ = false;
    src_point_selected_ = false;
    src_pc_->points.clear();
    dst_pc_->points.clear();
    src_pc_->height = 1;
    src_pc_->width = 0;
    dst_pc_->height = 1;
    dst_pc_->width = 0;
//    vis_src_->removePointCloud("src_pc");
//    vis_dst_->removePointCloud("dst_pc");
    vis_src_->removeAllShapes();
    vis_dst_->removeAllShapes();
}

void ManualRegistration::timeoutSlot() {
    if (cloud_src_present_ && cloud_src_modified_) {
        if (!vis_src_->updatePointCloud(cloud_src_, "cloud_src_")) {
            vis_src_->addPointCloud(cloud_src_,
                                    pcl::visualization::PointCloudColorHandlerGenericField<PointT>(cloud_src_, "z"),
                                    "cloud_src_");
            vis_src_->resetCameraViewpoint("cloud_src_");
        }
        cloud_src_modified_ = false;
    }
    if (cloud_dst_present_ && cloud_dst_modified_) {
        if (!vis_dst_->updatePointCloud(cloud_dst_, "cloud_dst_")) {
            vis_dst_->addPointCloud(cloud_dst_,
                                    pcl::visualization::PointCloudColorHandlerGenericField<PointT>(cloud_dst_, "z"),
                                    "cloud_dst_");
            vis_dst_->resetCameraViewpoint("cloud_dst_");
        }
        cloud_dst_modified_ = false;
    }
    ui_->qvtk_widget_src->update();
    ui_->qvtk_widget_dst->update();
}

int main(int argc, char **argv) {
    QApplication app(argc, argv);

    ManualRegistration::CloudPtr cloud_src(new ManualRegistration::Cloud);
    ManualRegistration::CloudPtr cloud_dst(new ManualRegistration::Cloud);

    if (argc < 3) {
        PCL_ERROR ("Usage:\n\t%s <source_cloud.pcd> <target_cloud.pcd>\n", argv[0]);
        return 0;
    }

    // TODO do this with PCL console
    string ext = std::filesystem::path(argv[1]).extension().string();
    if (ext == ".pcd") {
        if (pcl::io::loadPCDFile<ManualRegistration::PointT>(argv[1], *cloud_src) == -1) {
            PCL_ERROR ("Couldn't read PCD file %s \n", argv[1]);
            return (-1);
        }
    } else if (ext == ".ply") {
        pcl::PLYReader plyReader;
        if (!plyReader.read(argv[1], *cloud_src)) {
            PCL_ERROR ("Couldn't read PLY file %s \n", argv[1]);
            return -1;
        }
    } else {
        PCL_ERROR ("Unknown extension %s for file %s \n", ext.c_str(), argv[1]);
    }

    if (pcl::io::loadPCDFile<ManualRegistration::PointT>(argv[2], *cloud_dst) == -1) {
        PCL_ERROR ("Couldn't read file %s \n", argv[2]);
        return (-1);
    }

    // Remove NaNs
    std::vector<int> dummy;
    pcl::removeNaNFromPointCloud(*cloud_src, *cloud_src, dummy);
    pcl::removeNaNFromPointCloud(*cloud_dst, *cloud_dst, dummy);

    pcl::search::KdTree<ManualRegistration::PointT> s;
    const int k = 5;
    std::vector<std::vector<int> > idx;
    std::vector<std::vector<float> > distsq;

    s.setInputCloud(cloud_src);
    s.nearestKSearch(*cloud_src, std::vector<int>(), 5, idx, distsq);
    double res_src = 0.0f;
    for (size_t i = 0; i < cloud_src->size(); ++i) {
        double resi = 0.0f;
        for (int j = 1; j < k; ++j)
            resi += sqrtf(distsq[i][j]);
        resi /= double(k - 1);
        res_src += resi;
    }
    res_src /= double(cloud_src->size());

    s.setInputCloud(cloud_dst);
    s.nearestKSearch(*cloud_dst, std::vector<int>(), 5, idx, distsq);
    double res_dst = 0.0f;
    for (size_t i = 0; i < cloud_dst->size(); ++i) {
        double resi = 0.0f;
        for (int j = 1; j < k; ++j)
            resi += sqrtf(distsq[i][j]);
        resi /= double(k - 1);
        res_dst += resi;
    }
    res_dst /= double(cloud_dst->size());

    ManualRegistration man_reg;

    man_reg.setSrcCloud(cloud_src);
    man_reg.setResolution(std::max<double>(res_src, res_dst));
    man_reg.setDstCloud(cloud_dst);

    man_reg.show();

    return (app.exec());
}
