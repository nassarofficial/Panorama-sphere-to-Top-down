// Panorama to Top-down view
// Author: Ahmed Nassar
// g++ -std=c++0x -I/usr/local/include/opencv -I/usr/local/include/opencv2 -L/usr/local/lib/ -g -o grab_and_unwarp  main.cpp -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_ml -lopencv_video -lopencv_features2d -lopencv_calib3d -lopencv_objdetect -lopencv_contrib -lopencv_legacy -lopencv_stitching

#define _USE_MATH_DEFINES
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#include <string>
#include <iomanip>
#include <algorithm>
#include <math.h>
#include <vector>
#include <fstream>
#include "opencv2/imgproc/imgproc.hpp"
// For file system

using namespace std;
using namespace cv;
using namespace std;

int new_imgH = 640; // Horizontal Resolution = Width
double new_imgShort = new_imgH/(4*3);
double fov = 317.14; // Horiontal Angle of View // Max is 317.14 and min is 2.8
double x = M_PI; // Horizontal Angle
double y = -1*(M_PI)/2;
vector<string> m_file_list;
vector<Mat> panoramic_imgs;
string extension = "png";

static void meshgrid(const cv::Mat &xgv, const cv::Mat &ygv,
                     cv::Mat1i &X, cv::Mat1i &Y)
{
    cv::repeat(xgv.reshape(1,1), ygv.total(), 1, X);
    cv::repeat(ygv.reshape(1,1).t(), 1, xgv.total(), Y);
}

// helper function (maybe that goes somehow easier)
static void meshgridTest(const cv::Range &xgv, const cv::Range &ygv,
                         cv::Mat1i &X, cv::Mat1i &Y)
{
    std::vector<int> t_x, t_y;
    for (int i = xgv.start; i <= xgv.end; ++i) t_x.push_back(i);
    for (int i = ygv.start; i <= ygv.end; ++i) t_y.push_back(i);
    meshgrid(cv::Mat(t_x), cv::Mat(t_y), X, Y);
}

// Function that iterates directory for files or directories


// Function that was converter from matlab to unwrap the images
Mat warpImageFast(Mat imgwarp, Mat XXdense, Mat YYdense)
{
    Mat result;
    double minY1, maxY1;
    minMaxIdx(YYdense, &minY1, &maxY1);
    double minX, minY, maxX, maxY;
    minMaxIdx(XXdense, &minX, &maxX);
    minMaxIdx(YYdense, &minY, &maxY);

    minX = std::max(1,int(floor(minX))-1);
    minY = std::max(1,int(floor(minY))-1); 
    maxX = std::min(imgwarp.cols,int(ceil(maxX))+1);
    maxY = std::min(imgwarp.rows,int(ceil(maxY))+1);
 
    for (int i = 0; i < XXdense.cols;++i)
    {
        for (int j = 0; j < YYdense.rows;++j)
        {
            XXdense.at<float>(i,j)=XXdense.at<float>(i,j)-minX+1;
            YYdense.at<float>(i,j)=YYdense.at<float>(i,j)-minY+1;
            
        }
    }

    Rect regionOfInterest(minX-1, minY, maxX, maxY-minY);
    imgwarp = imgwarp(regionOfInterest);

    result.create(maxX,maxY, imgwarp.type());
    cv::remap(imgwarp, result, XXdense, YYdense, CV_INTER_CUBIC);

    cout << "Saved." << endl;

    // namedWindow( "Display window", WINDOW_AUTOSIZE );
    // imshow( "Display window", result );
    // waitKey(0);

    return result;
}


Mat imgLookAt(Mat in_pano, double CENTERx, double CENTERy, int new_imgH, double new_imgShort, double fov)
{
    // Getting Image dimensions
    int sphereW = in_pano.cols;
    int sphereH = in_pano.rows;
    
    double r;
    vector<double> TX, TY, R, ANGy,ANGx, X, Y, Z, RZY, Px, Py;
    vector<int> INDy, INDx, INDn, INDxx;
    
    const int mySizes[3]={new_imgH,new_imgH,3};
    //cv::Mat warped_im = Mat::zeros(3,mySizes,CV_32FC1);
    cv::Mat warped_im;

    //cout << warped_im.at<double>(0,0,0) << endl;
    Mat Pxm, Pym;
    Pxm = cv::Mat::zeros(new_imgH, new_imgH, CV_32FC1);
    Pym = cv::Mat::zeros(new_imgH, new_imgH, CV_32FC1);

    cv::Mat1i TXwarp, TYwarp;
    meshgridTest(cv::Range(1,new_imgH), cv::Range(1,new_imgH), TXwarp, TYwarp);
    
    for (int i = 0; i < new_imgH;++i)
    {
        for (int j = 0; j < new_imgH;++j)
        {
            TX.push_back(TXwarp.at<int>(j,i)-0.5-(new_imgH/2));
            TY.push_back(TYwarp.at<int>(j,i)-0.5-(new_imgH/2));
        }
    }
    
    r = (new_imgH/2)/tan((fov/2));
    
    for (int i = 0; i < TY.size();++i)
    {
        R.push_back(sqrt(pow(TY[i],2)+pow(r,2)));
        ANGy.push_back(atan(-TY[i]/r)+CENTERy);
        X.push_back(sin(ANGy[i])*R[i]);
        Y.push_back(-cos(ANGy[i])*R[i]);
    }
    
    
    Z = TX;
    
    //INDn
    for (int i = 0; i<ANGy.size();++i)
    {
        if (abs(ANGy[i]) > M_PI/2)
        {
            INDn.push_back(i);
        }
    }
    
    for (int i = 0; i < Z.size();++i)
    {
        ANGx.push_back(atan(Z[i]/-Y[i]));
        RZY.push_back(sqrt(pow(Z[i],2)+pow(Y[i],2)));
        ANGy[i] = atan(X[i]/RZY[i]);
    }
    
    
    for (int i = 0; i < INDn.size();++i)
    {
        ANGx[INDn[i]] = ANGx[INDn[i]] + M_PI;
    }
    
    for (int i = 0; i < ANGx.size();++i)
    {
        ANGx[i] = ANGx[i] + CENTERx;
        
    }
    
    for (int i = 0; i < ANGy.size();++i)
    {
        if (ANGy[i] < -M_PI/2)
        {
            INDy.push_back(i);
        }
    }
    
    for (int i = 0; i < INDy.size();++i)
    {
        ANGy[INDy[i]] = -M_PI - ANGy[INDy[i]];
        ANGx[INDy[i]] = ANGx[INDy[i]] + M_PI;
    }
    
    for (int i = 0; i < ANGx.size();++i)
    {
        if (ANGx[i] <= -M_PI)
        {
            INDx.push_back(i);
        }
    }
    for (int i = 0; i < INDx.size();++i)
    {
        ANGx[INDx[i]] = ANGx[INDx[i]] + 2 * M_PI;
    }
    
    // Check why its repeated 3 times
    for (int i = 0; i < ANGx.size();++i)
    {
        if (ANGx[i] > M_PI)
        {
            INDx.push_back(i);
        }
    }
    for (int i = 0; i < INDx.size();++i)
    {
        ANGx[INDx[i]] = ANGx[INDx[i]] - 2 * M_PI;
    }
    
    
    for (int i = 0; i < ANGx.size();++i)
    {
        Px.push_back((ANGx[i]+M_PI)/(2*M_PI) * sphereW + 0.5);
        Py.push_back(((-ANGy[i])+M_PI/2)/M_PI * sphereH + 0.5);
    }
    
    
    for (int i=0; i < Px.size();++i)
    {
        if (Px[i] < 1)
        {
            INDxx.push_back(i);
        }
    }
    
    for (int i = 0; i < INDxx.size();++i)
    {
        Px[INDxx[i]] = Px[INDxx[i]] + sphereW;
    }


    int count = 0;

    for (int i = 0; i < new_imgH;++i)
    {
        for (int j = 0; j < new_imgH;++j)
        {
            Pxm.at<float>(j,i) = float(Px[count]);
            Pym.at<float>(j,i) = float(Py[count]);
            count = count + 1;
        }
    }

    warped_im = warpImageFast(in_pano, Pxm, Pym);

    return warped_im;
}

size_t write_data(char *ptr, size_t size, size_t nmemb, void *userdata) {
    std::ostringstream *stream = (std::ostringstream*)userdata;
    size_t count = size * nmemb;
    stream->write(ptr, count);
    return count;
}

//function to retrieve the image as Cv::Mat data type
int main(int argc, char *argv[])
{
    Mat panorama,panoimage;
    string panopath,coords;
    
    std::string current_exec_name = argv[0]; // Name of the current exec program
    std::string first_arge;
    std::vector<std::string> all_args;
    panopath = argv[1]; // Pano ID

    // Path load pano
    panorama = imread(panopath, CV_LOAD_IMAGE_COLOR);

    // Unwarp and get the top down
    panoimage = imgLookAt(panorama,x,y,new_imgH,new_imgShort,fov);

    imwrite("unwarped_output.jpg", panoimage);

    return 0;
}
