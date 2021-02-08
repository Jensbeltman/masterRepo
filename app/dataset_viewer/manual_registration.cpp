#include "manual_registration.h"

// Qt
#include <QApplication>
#include <QEvent>
#include <QMessageBox>
#include <QMutexLocker>
#include <QObject>
#include <QSettings>

// VTK
#include <vtkRenderWindow.h>
#include <vtkRendererCollection.h>
#include <vtkCamera.h>
#include <vtkPointPicker.h>
#include <vtkPLYReader.h>

// PCL
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/icp.h>
#include <pcl/io/ply_io.h>

// filesystem
#include <filesystem>
#include <fstream>

using namespace pcl;
using namespace std;

//////////////////////////////////////////////////////////////////////////////////////////////////////////
ManualRegistration::ManualRegistration(QMainWindow *parent): QMainWindow(parent) {
    // Initialize bogus
    load_settings();
    res_ = 1;
    cloud_src_present_ = false;
    cloud_dst_present_ = false;
    src_point_selected_ = false;
    dst_point_selected_ = false;

    // Construction Visualizers
    render_window_src_ = vtkSmartPointer<vtkRenderWindow>::New();
    render_window_dst_ = vtkSmartPointer<vtkRenderWindow>::New();
    renderer_src_ = vtkSmartPointer<vtkRenderer>::New();
    renderer_dst_ = vtkSmartPointer<vtkRenderer>::New();
    render_window_src_->AddRenderer(renderer_src_);
    render_window_dst_->AddRenderer(renderer_dst_);

    vis_src_ = pcl::make_shared<PointCloudGroupVisualizer>(renderer_src_, render_window_src_, "vis_src", false);
    vis_dst_ = pcl::make_shared<PointCloudGroupVisualizer>(renderer_dst_, render_window_dst_, "vis_dst", false);


    //Create a timer
    vis_timer_ = new QTimer(this);
    vis_timer_->start(5);//5ms

    connect(vis_timer_, SIGNAL (timeout()), this, SLOT (timeoutSlot()));

    ui_ = new Ui::MainWindow;
    ui_->setupUi(this);

    this->setWindowTitle("Manual Registration");

    // Set up the source window
    ui_->qvtk_widget_src->SetRenderWindow(render_window_src_);

    vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor_src = ui_->qvtk_widget_src->GetInteractor();
    pointPicker_src = vtkSmartPointer<vtkPointPicker>::New();
    pointPicker_src->SetTolerance(0.001);
    renderWindowInteractor_src->SetPicker(pointPicker_src);
    renderWindowInteractor_src->SetRenderWindow(render_window_src_);

    vis_src_->setupInteractor(renderWindowInteractor_src, ui_->qvtk_widget_src->GetRenderWindow());
    vis_src_->getInteractorStyle()->setKeyboardModifier(pcl::visualization::INTERACTOR_KB_MOD_SHIFT);
    vis_src_->setShowFPS(false);
    ui_->qvtk_widget_src->update();


    vis_src_->registerPointPickingCallback(&ManualRegistration::SourcePointPickCallback, *this);

    // Set up the destination window
    ui_->qvtk_widget_dst->SetRenderWindow(render_window_dst_);

    vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor_dst = ui_->qvtk_widget_dst->GetInteractor();
    pointPicker_dst = vtkSmartPointer<vtkPointPicker>::New();
    pointPicker_dst->SetTolerance(0.001);
    renderWindowInteractor_dst->SetPicker(pointPicker_dst);
    renderWindowInteractor_dst->SetRenderWindow(render_window_dst_);

    vis_dst_->setupInteractor(renderWindowInteractor_dst, ui_->qvtk_widget_dst->GetRenderWindow());
    vis_dst_->getInteractorStyle()->setKeyboardModifier(pcl::visualization::INTERACTOR_KB_MOD_SHIFT);
    vis_dst_->setShowFPS(false);
    ui_->qvtk_widget_dst->update();

    vis_dst_->registerPointPickingCallback(&ManualRegistration::DstPointPickCallback, *this);


    // Connect all buttons
    connect(ui_->calculateButton, SIGNAL(clicked()), this, SLOT(calculatePressed()));
    connect(ui_->clearButton, SIGNAL(clicked()), this, SLOT(clearPressed()));
    connect(ui_->verifyButton, SIGNAL(clicked()), this, SLOT(verifyPressed()));
    connect(ui_->pickTolerancedoubleSpinBox, qOverload<double>(&QDoubleSpinBox::valueChanged),this, &ManualRegistration::update_pick_tolerance);
    connect(ui_->actionSave,SIGNAL(triggered()),this,  SLOT(save_anotation()));
    connect(ui_->removeGTButton,SIGNAL(clicked()),this,SLOT(removeGT()));
    connect(ui_->undoRemoveGTButton,SIGNAL(clicked()),this,SLOT(undoGTRemoval()));

    cloud_src_modified_ = true; // first iteration is always a new pointcloud
    cloud_dst_modified_ = true;

    src_pc_.reset(new PointCloudT);
    dst_pc_.reset(new PointCloudT);

    cloud_src_ =  pcl::make_shared<PointCloudT>();
    cloud_dst_ =  pcl::make_shared<PointCloudT>();

    setWindowState(Qt::WindowMaximized);
}

void ManualRegistration::calculatePressed() {
    if(!new_gt_verified){
        PCL_INFO ("Last anotation not verified, please verify before calculating a new one!\n");
        QMessageBox::warning(this,
                             QString("Warning"),
                             QString("Last anotation not verified, please verify before calculating a new one!"));
        return;
    }



    if (dst_pc_->points.size() != src_pc_->points.size()) {
        PCL_INFO ("You haven't selected an equal amount of points, please do so!\n");
        QMessageBox::warning(this,
                             QString("Warning"),
                             QString("You haven't selected an equal amount of points, please do so!"));
        return;
    }
    const double voxel_size = 3 * res_;
    const double inlier_threshold_ransac = 2 * voxel_size;
    const double inlier_threshold_icp = 2 * voxel_size;

    PointCloudT::Ptr cloud_src_ds = pcl::make_shared<PointCloudT>();
    PointCloudT::Ptr cloud_src_partial = pcl::make_shared<PointCloudT>();
    PointCloudT::Ptr cloud_src_partial_ds = pcl::make_shared<PointCloudT>();
    PointCloudT::Ptr cloud_dst_ds = pcl::make_shared<PointCloudT>();
    if (ui_->robustBox->isChecked() || ui_->refineBox->isChecked()) {
        PCL_INFO("Downsampling point clouds with a voxel size of %f...\n", voxel_size);
        pcl::VoxelGrid<PointT> grid;
        grid.setLeafSize(voxel_size, voxel_size, voxel_size);
        grid.setInputCloud(cloud_src_);
        grid.filter(*cloud_src_ds);
        grid.setInputCloud(cloud_dst_);
        grid.filter(*cloud_dst_ds);
    }

    if(ui_->useOCBox->isChecked()){
        if((!vis_dst_->current_group->nodes.empty()) && vis_dst_->current_group->selected_node->get() != nullptr){
            if(vis_dst_->current_group->id == "ocs"){
                transform_ =  oc_id_to_t4[vis_dst_->current_group->selected_node->get()->id].matrix().cast<float>();
            }
        }
    }
    else{
        if (ui_->robustBox->isChecked()) {
            pcl::Correspondences corr(src_pc_->size());
            for (size_t i = 0; i < src_pc_->size(); ++i)
                corr[i].index_query = corr[i].index_match = i;

            PCL_INFO("Computing pose using RANSAC with an inlier threshold of %f...\n", inlier_threshold_ransac);
            pcl::registration::CorrespondenceRejectorSampleConsensus<PointT> sac;
            sac.setInputSource(src_pc_);
            sac.setInputTarget(dst_pc_);
            sac.setInlierThreshold(inlier_threshold_ransac);

            pcl::Correspondences inliers;
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
            sac.getRefineModel();
        } else {
            PCL_INFO("Computing pose using clicked correspondences...\n");
            pcl::registration::TransformationEstimationSVD<PointT, PointT> tfe;
            tfe.estimateRigidTransformation(*src_pc_, *dst_pc_, transform_);
        }
    }

    T4 updateT(transform_.cast<double>());
    pointCloudRenderer->updateActor(0,updateT);
    pointCloudRenderer->fitCameraAndResolution();
    pointCloudRenderer->renderPointCloud(cloud_src_partial,updateT);
    pcl::VoxelGrid<PointT> grid;
    grid.setLeafSize(res_, res_, res_);
    grid.setInputCloud(cloud_src_partial);
    grid.filter(*cloud_src_partial);
    grid.setLeafSize(voxel_size, voxel_size, voxel_size);
    grid.setInputCloud(cloud_src_partial);
    grid.filter(*cloud_src_partial_ds);


    if (ui_->refineBox->isChecked()) {
        pcl::IterativeClosestPoint<PointT, PointT> icp;
        PointCloudT::Ptr tmp = pcl::make_shared<PointCloudT>();

        PCL_INFO("Refining pose using ICP with an inlier threshold of %f...\n", inlier_threshold_icp);
        icp.setInputSource(cloud_src_partial_ds);
        icp.setInputTarget(cloud_dst_ds);
        icp.setMaximumIterations(1000);
        icp.setMaxCorrespondenceDistance(inlier_threshold_icp);
        icp.align(*tmp, transform_);

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
        icp.align(*tmp, icp.getFinalTransformation());

        if (!icp.hasConverged()) {
            PCL_ERROR("Fine ICP failed!\n");
            QMessageBox::warning(this,
                                 QString("Error"),
                                 QString("Fine ICP failed!"));
        }

        PCL_INFO("Rerunning ultra-fine ICP at full resolution with an inlier threshold of %f...\n", res_);
        icp.setInputSource(cloud_src_partial);
        icp.setInputTarget(cloud_dst_);
        icp.setMaximumIterations(25);
        icp.setMaxCorrespondenceDistance(res_);
        icp.align(*tmp, icp.getFinalTransformation());

        if (!icp.hasConverged()) {
            PCL_ERROR("Ultra-fine ICP failed!\n");
            QMessageBox::warning(this,
                                 QString("Error"),
                                 QString("Ultra-fine ICP failed!"));
            return;
        }
    }

    PointCloudT::Ptr tmp= pcl::make_shared<PointCloudT>();
    transformPointCloud(*cloud_src_partial, *tmp,transform_);
    new_gts_.emplace_back(transform_.cast<double>());
    new_gt_verified = false;
    vis_dst_->addIdPointCloud(tmp,"new_gt_"+std::to_string(new_gts_.size()),"new_gts",0,255,0);
    vis_src_->update_text();


    PCL_INFO("All done! The final refinement was done with an inlier threshold of %f, "
             "and you can expect the resulting pose to be accurate within this bound.\n", res_);



    std::cout << "Transform: " << std::endl << transform_ << std::endl;

    std::cout
            << "The transform can be used to place the source (leftmost) point cloud into the target, and thus places observations (points, poses) relative to the source camera in the target camera (rightmost). If you need the other way around, use the inverse:"
            << std::endl << transform_.inverse() << std::endl;

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

    std::vector<std::string> props_to_remove_src;
    for(auto &prop:*(vis_src_->getShapeActorMap()))
        if(prop.second->IsA("vtkLODActor"))
            if(prop.first.find("sphere")!= prop.first.npos)
                props_to_remove_src.emplace_back(prop.first);
    std::for_each(props_to_remove_src.begin(), props_to_remove_src.end(), [this](string& propstr) { vis_src_->removeShape(propstr); });

    std::vector<std::string> props_to_remove_dst;
    for(auto &prop:*(vis_dst_->getShapeActorMap()))
        if(prop.second->IsA("vtkLODActor"))
            if(prop.first.find("sphere")!= prop.first.npos)
                props_to_remove_dst.emplace_back(prop.first);
    std::for_each(props_to_remove_dst.begin(), props_to_remove_dst.end(), [this](string& propstr) { vis_dst_->removeShape(propstr); });

    if(!new_gt_verified){
        PCVGroupPtr new_gts_g = find_pcv_group_id(vis_dst_->pcv_root,"new_gts");
        vis_dst_->remove_pcv_node(new_gts_g->nodes.back()->id);
        vis_dst_->update_text();
    }
    new_gt_verified=true;
}

void ManualRegistration::verifyPressed() {
 if(!new_gt_verified){
     new_gt_verified=true;
 }
}

void ManualRegistration::timeoutSlot() {
    ui_->qvtk_widget_src->update();
    ui_->qvtk_widget_dst->update();
}

void ManualRegistration::setSrcCloud(PointCloudT::Ptr cloud_src, std::string new_mesh_ply_path) {
    mesh_ply_path = new_mesh_ply_path;
    pointCloudRenderer = std::make_shared<PointCloudRenderer>();
    auto identity = T4::Identity();
    pointCloudRenderer->addActorPLY(mesh_ply_path,identity);
    pcl::Indices idcs;
    removeNaNFromPointCloud(*cloud_src,*cloud_src_,idcs);

    auto reader = vtkSmartPointer<vtkPLYReader>::New();
    reader->SetFileName(mesh_ply_path.c_str());
    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(reader->GetOutputPort());
    auto ply_actor = vtkSmartPointer<vtkActor>::New();
    ply_actor->SetMapper(mapper);
    vis_src_->getRendererCollection()->GetFirstRenderer()->AddActor(ply_actor);

    vis_src_->addIdPointCloud(cloud_src_,
                              pcl::visualization::PointCloudColorHandlerGenericField<PointT>(cloud_src_, "z"),
                              "cloud_src_");
    vis_src_->resetCameraViewpoint("ply_src_");
}

void ManualRegistration::setResolution(double res) {
    res_ = res;
}

void ManualRegistration::setDstCloud(PointCloudT::Ptr cloud_dst) {
    pcl::Indices idcs;
    removeNaNFromPointCloud(*cloud_dst,*cloud_dst_,idcs);
    vis_dst_->addIdPointCloud(cloud_dst_,
                              pcl::visualization::PointCloudColorHandlerGenericField<PointT>(cloud_dst_, "z"),
                              "cloud_dst_");
    vis_dst_->resetCameraViewpoint("cloud_dst_");
    update_res();
}

void ManualRegistration::setGTs(vector<T4> &gts, std::string path) {
    orig_gts_ = gts;
    ground_truth_path = path;

    if(vis_dst_->find_pcv_group_id("gts")!=nullptr)
        vis_dst_->remove_pcv_group("gts");


    for(int i = 0;i<gts.size();i++){
        PointCloudT::Ptr gtpc(new PointCloudT);
        pcl::transformPointCloud(*cloud_src_,*gtpc,gts[i]);
        std::string id = "gt_"+std::to_string(i);
        vis_dst_->addIdPointCloud(gtpc, id, "gts", 0, 180, 0);
    }
}

void ManualRegistration::removeGT() {
    if( (vis_dst_->current_group->id == "gts") && !vis_dst_->current_group->nodes.empty()){
        string id = vis_dst_->current_group->selected_node->get()->id;
        gts_to_be_removed.emplace_back(std::stoi(id.substr(3)));
        vis_dst_->remove_pcv_node(id);
    }
}

void ManualRegistration::undoGTRemoval() {
    setGTs(orig_gts_,ground_truth_path);
    gts_to_be_removed.clear();
}


void ManualRegistration::removeGTsFromFile(std::vector<int> gt_indexes) {
    std::ifstream infile(ground_truth_path,ios::in);
    std::ofstream tmpfile("./tmpfile",ios::out);

    char data[256];
    int i = 0;
    while(infile.getline(data,256)){
        if(std::find(gt_indexes.begin(),gt_indexes.end(),i/4)==gt_indexes.end()){
            tmpfile<<data<<"\n";
        }
        i++;
    }
    tmpfile.close();
    infile.close();
    std::remove(ground_truth_path.c_str());
    std::rename("./tmpfile",ground_truth_path.c_str());
}

void ManualRegistration::setOCs(vector<T4> &ocs) {
    ocs_ = ocs;

    for(int i = 0;i<ocs.size();i++){
        PointCloudT::Ptr ocpc(new PointCloudT);
        pcl::transformPointCloud(*cloud_src_,*ocpc,ocs[i]);
        std::string id = "oc_"+std::to_string(i);
        vis_dst_->addIdPointCloud(ocpc, id, "ocs", 255, 128, 0);
        oc_id_to_t4[id] = ocs[i];
    }
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
        std::string id = "sphere_src_" + oss.str();
        vis_src_->addSphere<PointT>(src_point_, 3, 0, 1, 0, id);
        vis_src_->getShapeActorMap()->at(id)->SetPickable(0);
        vtkProp * prop = vis_src_->getShapeActorMap()->at(id);
        vtkLODActor * actor = dynamic_cast<vtkLODActor*>(prop);
        actor->GetProperty()->SetInterpolationToGouraud();

        ui_->qvtk_widget_src->update();

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
        std::string id = "sphere_dst_" + oss.str();
        vis_dst_->addSphere<PointT>(dst_point_, 3, 0, 1, 0, id);
        vis_dst_->getShapeActorMap()->at(id)->SetPickable(0);
        vtkProp * prop = vis_dst_->getShapeActorMap()->at(id);
        vtkLODActor * actor = dynamic_cast<vtkLODActor*>(prop);
        actor->GetProperty()->SetInterpolationToGouraud();
    } else {
        PCL_INFO ("Please select a point in the destination window first\n");
    }
}

void ManualRegistration::update_res() {
    pcl::search::KdTree<ManualRegistration::PointT> s;
    const int k = 5;
    std::vector<std::vector<int> > idx;
    std::vector<std::vector<float> > distsq;

    s.setInputCloud(cloud_dst_);
    s.nearestKSearch(*cloud_dst_, std::vector<int>(), 5, idx, distsq);
    double res_dst = 0.0f;
    for(size_t i = 0; i < cloud_dst_->size(); ++i) {
        double resi = 0.0f;
        for(int j = 1; j < k; ++j)
            resi += sqrtf(distsq[i][j]);
        resi /= double(k - 1);
        res_dst += resi;
    }
    res_dst /= double(cloud_dst_->size());

    res_ = res_dst;
}

void ManualRegistration::setup() {
    vis_dst_->resetCamera();
    vis_src_->resetCamera();
    vis_dst_->update_text();
    vis_src_->update_text();
}

void ManualRegistration::load_settings() {
    QSettings qsettings(this);;
    settings.refine = qsettings.value("refine_bool", true).toBool();
    settings.robust = qsettings.value("robust_bool", false).toBool();
    settings.point_picker_tolerance = qsettings.value("point_picker_tolerance_double", 0.05).toDouble();
}

void ManualRegistration::save_settings() {
    QSettings qsettings(this);
    qsettings.setValue("refine_bool", settings.refine);
    qsettings.setValue("robust_bool", settings.robust);
    qsettings.setValue("point_picker_tolerance_double", settings.point_picker_tolerance);
}

void ManualRegistration::update_pick_tolerance(double pt) {
    pointPicker_dst->SetTolerance(pt);
    pointPicker_src->SetTolerance(pt);
    std::string s = std::to_string(static_cast<int>((*render_window_src_->GetScreenSize())*pt))+"/"+std::to_string(static_cast<int>((*render_window_dst_->GetScreenSize())*pt))+" [px]";
    ui_->toleranceLabel->setText(QString::fromStdString(s));
}

void ManualRegistration::save_anotation() {
    if(!gts_to_be_removed.empty())
        removeGTsFromFile(gts_to_be_removed);

    if(!new_gts_.empty()){
        std::ofstream outfile;
        if (std::filesystem::exists(ground_truth_path)) {
            outfile.open(ground_truth_path, std::ios_base::app); // append instead of overwrite
        }else {
            outfile.open(ground_truth_path, ios_base::out);
        }

        if(!outfile.is_open()){
            std::cout<<"File was not saved !! Check if path is invalid "<< ground_truth_path << std::endl;
        }else{
            for (auto &ngt:new_gts_) {
                outfile << ngt.matrix()<<"\n";
            }
            outfile.close();
        }
    }else{
        std::cout<<"No New Anotations found, nothing changed"<<std::endl;
    }

    this->close();
}






