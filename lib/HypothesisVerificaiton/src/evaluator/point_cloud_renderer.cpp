//
// Created by jens on 11/25/20.
//

#include "hypothesis_verification/evaluator/point_cloud_renderer.hpp"
#include "chronometer.h"
#include "vtkCamera.h"
#include "vtkMatrix4x4.h"
#include "vtkHomogeneousTransform.h"
#include "vtkProperty.h"

void PointCloudRenderer::init() {
    setRes(xres, yres);
    renWin->AddRenderer(renderer);
    renWin->SetOffScreenRendering(1);
    renWin->SetSize(xres, yres); // Set resolution
    renWin->SetMultiSamples(0); // Antialising so that color maps works
    renderer->PreserveColorBufferOn();
    renWin->Render();
};

PointCloudRenderer::PointCloudRenderer(int xres, int yres) : xres(xres), yres(yres),
                                                             renderer(vtkSmartPointer<vtkRenderer>::New()),
                                                             renWin(vtkSmartPointer<vtkRenderWindow>::New()) {
    init();
}


void PointCloudRenderer::renderPointCloud(PointCloudT::Ptr &pc,T4 inverse_transform) {
    renWin->Render();
    updateDepth();

    pc->points.reserve(xres * yres);

    getWorldCoordMatrix(mat);

    if (!inverse_transform.isApprox(T4::Identity()))
        convertDepthToPointCloud(pc,&(inverse_transform));
    else
        convertDepthToPointCloud(pc);

}

void PointCloudRenderer::check_pcs(std::vector<PointCloudT::Ptr> &pcs) {
    if (pcs.size() != actors.size())
        if (pcs.size() < actors.size()) {
            int diff = actors.size() - pcs.size();
            for (int i = 0; i < diff; i++)
                pcs.emplace_back(pcl::make_shared<PointCloudT>());
        }
        else {
            cout << "Received more point clouds than there are actors in function \"renderPointClouds\" only "
                 << pcs.size() - actors.size() << "pcs will be removed";
            pcs.resize(actors.size());
        }

}

void PointCloudRenderer::renderPointClouds(std::vector<PointCloudT::Ptr> &pcs,bool apply_inverse_transform) {
    check_pcs(pcs);

    for (auto &a:actors)
        a->SetVisibility(0);

    getWorldCoordMatrix(mat);


    for (int i = 0; i < actors.size(); i++) {
        auto &a = actors[i];
        auto &pc = pcs[i];
        a->SetVisibility(1);

        pc->points.reserve(xres * yres);
        renWin->Render();
        updateDepth();
        if (apply_inverse_transform)
            convertDepthToPointCloud(pc,&(actor_transforms[i]));
        else
            convertDepthToPointCloud(pc);

        a->SetVisibility(0);
    }

    for (auto &a:actors)
        a->SetVisibility(1);
}


void PointCloudRenderer::renderPointsCloudsNoOcclusion(std::vector<PointCloudT::Ptr> &pcs, double occlusion_dist_sq) {
    check_pcs(pcs);
    getWorldCoordMatrix(mat);

    auto pc_itt = pcs.begin();

    check_pcs(pcs);

    for (auto &a:actors)
        a->SetVisibility(0);

    getWorldCoordMatrix(mat);

    std::vector<bool> actor_rendered(actors.size(), false);
    std::vector<double[3]> actor_pos(actors.size());

    for (int i = 0; i < actors.size(); i++)
        actors[i]->GetPosition(actor_pos[i]);


    bool actors_left = true;
    double position[3];
    double *a_p, *b_p, *diff_p;
    bool in_occlusion = false;

    while (actors_left) {
        std::vector<int> chosen_actors_ids;
        for (int i = 0; i < actors.size(); i++) {
            auto &actor = actors[i];

            if (!actor_rendered[i]) {
                a_p = actor_pos[i];

                in_occlusion = false;
                for (int j = 0; j < chosen_actors_ids.size(); j++) {
                    b_p = actor_pos[chosen_actors_ids[j]];
                    diff_p[0] = a_p[0] - b_p[0];
                    diff_p[1] = a_p[1] - b_p[1];
                    diff_p[2] = a_p[2] - b_p[2];
                    if ((diff_p[0] * diff_p[0] + diff_p[1] * diff_p[1] + diff_p[2] * diff_p[2]) > occlusion_dist_sq) {
                        in_occlusion = true;
                        break;
                    }
                }
                if (!in_occlusion)
                    chosen_actors_ids.push_back(i);
            }
        }

        for (auto &ai:chosen_actors_ids)
            actors[ai]->SetVisibility(1);

        (*pc_itt)->points.reserve(xres * yres);
        renWin->Render();
        updateDepth();
        convertDepthToPointCloud((*pc_itt));

        for (auto &ai:chosen_actors_ids)
            actors[ai]->SetVisibility(0);

        if (chosen_actors_ids.empty()) {
            actors_left = false;
        }
    }
}


void PointCloudRenderer::renderPointCloudsAllAtOnce(std::vector<PointCloudT::Ptr> &pcs) {
    check_pcs(pcs);
    getWorldCoordMatrix(mat);
}

void PointCloudRenderer::getWorldCoordMatrix(Eigen::Matrix4f &mat) {
    // Transform pc to give camera coordinates instead of world coordinates!
    vtkCamera *camera = renderer->GetActiveCamera();
    vtkSmartPointer<vtkMatrix4x4> composite_projection_transform = camera->GetCompositeProjectionTransformMatrix(
            renderer->GetTiledAspectRatio(), 0, 1);
    composite_projection_transform->Invert();
    vtkSmartPointer<vtkMatrix4x4> view_transform = camera->GetModelViewTransformMatrix();

    double position[3];
    camera->GetPosition(position);
    double *quat_data = camera->GetOrientationWXYZ();

    Eigen::Quaternion<double> quaternion(quat_data);
    T4 user_transform_inverse = T4::Identity();
    user_transform_inverse.rotate(quaternion);
    user_transform_inverse.translate(Eigen::Vector3d(position));


    Eigen::Matrix4f mat1, mat2, mat3;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            mat1(i, j) = static_cast<float> (composite_projection_transform->Element[i][j]);
            mat2(i, j) = static_cast<float> (view_transform->Element[i][j]);
        }
    }
    mat = user_transform_inverse.matrix().cast<float>()*mat2 * mat1;
}

void PointCloudRenderer::convertDepthToPointCloud(PointCloudT::Ptr &pc, T4* object_transform) {// TODO add option for transforming point cloud based on custom tf(ocs)
    unsigned int ptr = 0;
    Eigen::Vector4f world_coords;
    vtkCamera *camera = renderer->GetActiveCamera();
    double pos[3];

    Eigen::Matrix4f mat_with_t;
    if(!(object_transform == nullptr))
        mat_with_t = object_transform->inverse().matrix().cast<float>()*mat;
    else
        mat_with_t = mat;

    float w3;
    for (int y = 0; y < yres; ++y) {
        for (int x = 0; x < xres; ++x, ++ptr) {
            if (depth[ptr] < 1.0) {
                world_coords(0) = dwidth * float(x) - 1.0f;
                world_coords(1) = dheight * float(y) - 1.0f;
                world_coords(2) = depth[ptr];
                world_coords(3) = 1.0f;
                world_coords = mat_with_t * world_coords;

                w3 = 1.0f / world_coords[3];
                // vtk view coordinate system is different than the standard camera coordinates (z forward, y down, x right), thus, the fliping in y and z
                world_coords[0] *= w3;
                world_coords[1] *= w3;
                world_coords[2] *= w3;

//                world_coords[0] += pos[0];
//                world_coords[1] += pos[1];
//                world_coords[2] += pos[2];




                (*pc).emplace_back(world_coords[0], world_coords[1], world_coords[2]);
   /*             world_coords[0] *= -w3;
                world_coords[1] *= w3;
                world_coords[2] *= w3;

                world_coords[0] += pos[0];
                world_coords[1] += pos[1];
                world_coords[2] -= pos[2];
                world_coords[2] *= -1;



                (*pc).emplace_back(world_coords[0], world_coords[1], world_coords[2]);*/
            }
        }
    }
}
void PointCloudRenderer::addActorPLY(std::string path, T4 t) {
    addActorsPLY(path,std::vector<T4>{t});
}

void PointCloudRenderer::addActorsPLY(std::string path, std::vector<T4> ts) {
    actor_transforms = ts;

    auto reader = vtkSmartPointer<vtkPLYReader>::New();
    reader->SetFileName(path.c_str());

    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(reader->GetOutputPort());

    for (int i = 0;i<ts.size();i++) {
        auto &t = ts[i];
        auto actor = vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(mapper);
//        actor->GetProperty()->SetColor(i/(double)ts.size(),0,0);
//        actor->GetProperty()->SetInterpolationToFlat();
//        actor->GetProperty()->LightingOff();

        actor->SetUserMatrix(t4ToVTK(t));

        renderer->AddActor(actor);
        actors.emplace_back(actor);
    }
}

void PointCloudRenderer::setRes(int x, int y) {
    if(!(depth == nullptr))
        delete[] depth;
    xres = x;
    yres = y;
    renWin->SetSize(xres, yres);
    dwidth = 2.0f / float(xres);
    dheight = 2.0f / float(yres);
    depth = new float[xres * yres];
}

void PointCloudRenderer::getRes(int &x, int &y) {
    x = xres;
    y = yres;
}

void PointCloudRenderer::updateDepth() { renWin->GetZbufferData(0, 0, xres - 1, yres - 1, &(depth[0])); }

void PointCloudRenderer::fitCameraAndResolution() {
//    auto camera = renderer->GetActiveCamera();
//    camera->SetRoll(0.3);
//    camera->UpdateViewport(renderer);
    renderer->ResetCamera();


}

PointCloudRenderer::~PointCloudRenderer() {
}

vtkMatrix4x4 *PointCloudRenderer::t4ToVTK(T4 &t4) {
    vtkMatrix4x4 *mat = vtkMatrix4x4::New();
    for (int r = 0; r < t4.matrix().rows(); r++) {
        for (int c = 0; c < t4.matrix().cols(); c++) {
            mat->SetElement(r, c, t4.matrix()(r, c));
        }
    }
    return mat;
}

bool PointCloudRenderer::updateActor(int i, T4 &t) {
    if(actors.size()>i){
        actors[i]->SetUserMatrix(t4ToVTK(t));
        return true;
    }else{
        return false;
    }
}







