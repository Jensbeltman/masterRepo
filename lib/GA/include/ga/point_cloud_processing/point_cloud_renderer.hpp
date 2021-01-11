#ifndef MASTER_POINT_CLOUD_RENDERER_HPP
#define MASTER_POINT_CLOUD_RENDERER_HPP

#include "vtkRenderer.h"
#include "vtkRenderWindow.h"
#include "vtkPolyDataMapper.h"
#include "vtkActor.h"
#include "vtkPLYReader.h"
#include "ga/typedefinitions.hpp"
#include <string>


class PointCloudRenderer {
public:
    vtkSmartPointer<vtkRenderer> renderer;
    vtkSmartPointer<vtkRenderWindow> renWin;
    std::vector<vtkSmartPointer<vtkActor> > actors;
    std::vector<T4> actor_transforms;

    float *depth = nullptr;


    PointCloudRenderer(int xres = 1920, int yres = 1080);
    ~PointCloudRenderer();

    void renderPointCloud(PointCloudT::Ptr &pc);

    void renderPointClouds(std::vector<PointCloudT::Ptr> &pcs,bool apply_inverse_transform = false);

    void addActorsPLY(std::string path, std::vector<T4> ts);

    void fitCameraAndResolution();

    void setRes(int x, int y);

    void getRes(int &x, int &y);


private:
    void init();

    void check_pcs(std::vector<PointCloudT::Ptr> &pcs);

    void getWorldCoordMatrix(Eigen::Matrix4f &mat);

    void convertDepthToPointCloud(PointCloudT::Ptr &pc, T4* object_transform = nullptr);

    void updateDepth();

    void updateColor();

    int xres;
    int yres;
    Eigen::Matrix4f mat;
    float dwidth;
    float dheight;


    // Nice to have
    void renderPointsCloudsNoOcclusion(std::vector<PointCloudT::Ptr> &pcs, double occlusion_dist_sq);

    void renderPointCloudsAllAtOnce(std::vector<PointCloudT::Ptr> &pcs);


};


#endif //MASTER_POINT_CLOUD_RENDERER_HPP
