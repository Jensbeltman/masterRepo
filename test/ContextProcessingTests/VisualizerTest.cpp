#include <iostream>
#include "ga/visualization/point_cloud_group_visualizer.hpp"
#include "ga/point_cloud_processing/point_cloud_renderer.hpp"
#include "vtkPLYReader.h"
#include <vtkActor.h>
#include "dataset/scape/ScapeDataset.hpp"
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkWindowToImageFilter.h>
#include <vtkMatrix4x4.h>
#include <pcl/common/transforms.h>


int main(){
    Chronometer chronometer;

    // Load dataset and create vtk actor
    ScapeDataset scapeData("/home/jens/masterData/ScapeDataset/Scape/Full_Dataset",
                           "/home/jens/masterData/ScapeDataset/Data from Scape Recognition");

    ScapeDatasetObjectPtr scapeObjPtr = scapeData.get_scape_object_by_name("Conrods");

    PointCloudRenderer pcRen;
    PointCloudGroupVisualizer vis;


    auto &dp = scapeObjPtr->data_points[0];

    pcRen.addActorsPLY(scapeObjPtr->mesh_path,dp.ocs);
    pcRen.fitCameraAndResolution();

    std::vector<PointCloudT::Ptr> pcs;
    for(int i = 0;i<dp.ocs.size();i++) {
        pcs.push_back(pcl::make_shared<PointCloudT>());
    }

    pcRen.renderPointClouds(pcs);

    for(int i = 0;i<pcs.size();i++) {
        vis.addIdPointCloud(pcs[i], "Point Cloud Number " + std::to_string(i), "Point Cloud Number " + std::to_string(i), 0, 255, 0);
    }

    vis.addCoordinateSystem(100);
    vis.resetCamera();

    vis.custom_spin();

    return 0;
}