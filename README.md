# Master Thesis Software Repository

This is the software repository for the my master thesis named: "Improving the False-Positive Rate of a Pose Estimation Pipeline In a Bin Picking Scenario using Hypothesis Verification". This repository does not contain data for testing.

**Libraries**:

- **GA**, an implementation of a Genetic Algorithm for binary single objective problem.
- **Dataset**, a lib that so far supports the Sileane Dataset and a non-public dataset given my Scape. 



## Build

**Pre-requisits**

- QT 5.12
  - Lower version might also workout
- VTK 7.1: https://github.com/Kitware/VTK/tree/v7.1.0
  - ~~Moved to 9.0 as 7.1 had errors related to it.~~
  - Moved back to 7.1 as PCL did not support and pull request that did gave unsolvable errors.
- OpenCV 4.4* and OpenCV contribution modules: https://github.com/opencv 
  - Build with VTK
- PCL 1.10: https://github.com/PointCloudLibrary/pcl/pull/4262
  - Build With QT,VTK 9.0
  - This pull request i to be merged to master soon as far as i understand.
- FCL: https://github.com/flexible-collision-library/fcl
- JSON lib: https://github.com/nlohmann/json
- Matplot++: https://github.com/alandefreitas/matplotplusplus
- Rapidcsv: https://github.com/d99kris/rapidcsv

### Build Notes

- When linking opencv to multiple executables use opencv_... targets instead of OPENCV_LIBS

  - ```
    Modules targets can be found in OpenCVConfig.cmake usually located at /usr/local/lib/cmake/opencv4 as of OpenCV 4.4: opencv_calib3d, opencv_core, opencv_dnn, opencv_features2d, opencv_flann, opencv_gapi, opencv_highgui, opencv_imgcodecs, opencv_imgproc, opencv_ml, opencv_objdetect, opencv_photo, opencv_stitching, opencv_video, opencv_videoio, opencv_alphamat, opencv_aruco, opencv_bgsegm, opencv_bioinspired, opencv_ccalib, opencv_datasets, opencv_dnn_objdetect, opencv_dnn_superres, opencv_dpm, opencv_face, opencv_fuzzy, opencv_hdf, opencv_hfs, opencv_img_hash, opencv_intensity_transform, opencv_line_descriptor, opencv_mcc, opencv_optflow, opencv_phase_unwrapping, opencv_plot, opencv_quality, opencv_rapid, opencv_reg, opencv_rgbd, opencv_saliency, opencv_shape, opencv_stereo, opencv_structured_light, opencv_superres, opencv_surface_matching, opencv_text, opencv_tracking, opencv_videostab, opencv_viz, opencv_xfeatures2d, opencv_ximgproc, opencv_xobjdetect, opencv_xphoto
    ```



