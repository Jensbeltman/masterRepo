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

    auto reader = vtkSmartPointer<vtkPLYReader>::New();
    reader->SetFileName(scapeObjPtr->mesh_path.c_str());
    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(reader->GetOutputPort());
    for (int i = 0;i<dp.ocs.size();i++) {
        auto &t = dp.ocs[i];
        auto actor = vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(mapper);

        vtkMatrix4x4 *mat = vtkMatrix4x4::New();
        for (int r = 0; r < t.matrix().rows(); r++) {
            for (int c = 0; c < t.matrix().cols(); c++) {
                mat->SetElement(r, c, t.matrix()(r, c));
            }
        }

        actor->SetUserMatrix(mat);

        vis.getRendererCollection()->GetFirstRenderer()->AddActor(actor);
    }


    double time;
    double time_total=0;
    int count=0;

    pcRen.addActorsPLY(scapeObjPtr->mesh_path,dp.ocs);
    pcRen.fitCameraAndResolution();

    std::vector<PointCloudT::Ptr> pcs;
    for(int i = 0;i<dp.ocs.size();i++) {
        auto pc = pcl::make_shared<PointCloudT>();
        pcs.emplace_back(pc);
    }


    chronometer.tic();
    pcRen.renderPointClouds(pcs);
    for(auto& pc:pcs){}
    time = chronometer.toc();
    cout<<"Render all ocs took: "<<time<<" seconds"<<std::endl;

    for(int i = 0;i<pcs.size();i++) {
        auto tmp_pc = pcl::make_shared<PointCloudT>();
        pcl::transformPointCloud(*pcs[i],*tmp_pc,dp.ocs[i]);
        vis.addIdPointCloud(tmp_pc, "pc_" + std::to_string(i), "pc_" + std::to_string(i), 0, 255, 0);
    }

    vis.addCoordinateSystem(100);
    vis.resetCamera();



    vis.custom_spin();

    return 0;
}