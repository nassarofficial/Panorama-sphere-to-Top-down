# Panoramic Spheres to Top-Down View Image
Convert panoramic spheres to top-down view image. This is a C++ implementation of the MATLAB implementation of "3D Geometry for Panorama" by Jianxiong Xiao.

### To Compile and Build:
g++ -std=c++0x -I/usr/local/include/opencv -I/usr/local/include/opencv2 -L/usr/local/lib/ -g -o Panorama_Unwarp  main.cpp -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_ml -lopencv_video -lopencv_features2d -lopencv_calib3d -lopencv_objdetect -lopencv_contrib -lopencv_legacy -lopencv_stitching -lboost_system -lboost_filesystem

### Citation:
J. Xiao, K. A. Ehinger, A. Oliva and A. Torralba.
Recognizing Scene Viewpoint using Panoramic Place Representation.
Proceedings of 25th IEEE Conference on Computer Vision and Pattern Recognition, 2012.
http://sun360.mit.edu

