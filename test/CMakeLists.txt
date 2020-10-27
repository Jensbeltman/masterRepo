cmake_minimum_required(VERSION 3.12)
project(test
        VERSION 1.0
        LANGUAGES CXX)

add_executable(sileane_test src/sileane_data_set_test.cpp)
target_link_libraries(sileane_test PUBLIC Chronometer Dataset)

add_executable(scape_test src/scape_data_set_test.cpp)
target_link_libraries(scape_test PUBLIC Chronometer Dataset)

add_executable(ga_scape_test src/ga_test_scape_data.cpp)
target_link_libraries(ga_scape_test PRIVATE GA)

add_executable(fcl_test src/fcl_test.cpp)
target_link_libraries(fcl_test PUBLIC GA)

add_executable(baseline src/baseline.cpp)
target_link_libraries(baseline PUBLIC GA)

#add_executable(test src/test.cpp)
#target_compile_features(test PUBLIC cxx_std_17)
#opencv_calib3d;opencv_core;opencv_dnn;opencv_features2d;opencv_flann;opencv_gapi;opencv_highgui;opencv_imgcodecs;opencv_imgproc;opencv_ml;opencv_objdetect;opencv_photo;opencv_stitching;opencv_video;opencv_videoio;opencv_alphamat;opencv_aruco;opencv_bgsegm;opencv_bioinspired;opencv_ccalib;opencv_datasets;opencv_dnn_objdetect;opencv_dnn_superres;opencv_dpm;opencv_face;opencv_fuzzy;opencv_hdf;opencv_hfs;opencv_img_hash;opencv_intensity_transform;opencv_line_descriptor;opencv_mcc;opencv_optflow;opencv_phase_unwrapping;opencv_plot;opencv_quality;opencv_rapid;opencv_reg;opencv_rgbd;opencv_saliency;opencv_shape;opencv_stereo;opencv_structured_light;opencv_superres;opencv_surface_matching;opencv_text;opencv_tracking;opencv_videostab;opencv_viz;opencv_xfeatures2d;opencv_ximgproc;opencv_xobjdetect;opencv_xphoto