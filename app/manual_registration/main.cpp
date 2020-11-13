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

int main(int argc, char **argv) {
    QApplication app(argc, argv);

    ManualRegistration::PointCloudT::Ptr cloud_src(new ManualRegistration::PointCloudT);
    ManualRegistration::PointCloudT::Ptr cloud_dst(new ManualRegistration::PointCloudT);

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
