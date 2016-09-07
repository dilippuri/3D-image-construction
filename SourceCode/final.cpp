/**
*       @file opencv_reproject_image_pcl.cpp
*       @brief Reproject an image to Point Cloud using OpenCV and PCL.
*       @author Martin Peris (http://www.martinperis.com)
*       @date 06/01/2012
*/

/*
        opencv_reproject_image_pcl.cpp - Reproject an image to Point Cloud
        using OpenCV and PCL. The program receives from command line an 
        rgb-image (left image of stereo rig) a disparity-image (obtained with 
        some stereo matching algorithm) and the matrix Q (Generated at calibration 
        stage). It displays the 3D reconstruction of the scene using PCL.
        Copyright (c) 2012 Martin Peris (http://www.martinperis.com).
        All right reserved.
        
        This application is free software; you can redistribute it and/or
        modify it under the terms of the GNU Lesser General Public
        License as published by the Free Software Foundation; either
        version 2.1 of the License, or (at your option) any later version.
        
        This application is distributed in the hope that it will be useful,
        but WITHOUT ANY WARRANTY; without even the implied warranty of
        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
        Lesser General Public License for more details.
        
        You should have received a copy of the GNU Lesser General Public
        License along with this application; if not, write to the Free Software
        Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <cv.h>
#include <highgui.h>
#include <iostream>
#include <string>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/contrib/contrib.hpp"
#include <stdio.h>

//Kids, using this kind of #define is quite dirty, don't do it at home!!
#define CUSTOM_REPROJECT
/*** To understand the CUSTOM_REPROJECT code, please read Chapter 12 of the book
  Learning OpenCV: Computer Vision with the OpenCV Library. (Page 435) 
  I am using it because cv::reprojectImageTo3D is not giving me the expected
  results for some reason.
  
  If you want to use this program with cv::reprojectImageTo3D please comment
  the previous #define CUSTOM_REPROJECT and recompile.
    
***/


//This function creates a PCL visualizer, sets the point cloud to view and returns a pointer
boost::shared_ptr<pcl::visualization::PCLVisualizer> createVisualizer (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "reconstruction");
  //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "reconstruction");
  viewer->addCoordinateSystem ( 1.0 );
  viewer->initCameraParameters ();
  return (viewer);
}

int main( int argc, char** argv )
{
  //**********************************************************Image DisparityMap Generation****************************************
  
   cv::Mat img1, img2, g1, g2;
    cv::Mat disp, disp8;
    char* method = argv[3];
    img1 = cv::imread(argv[1]);
    img2 = cv::imread(argv[2]);
    cv::cvtColor(img1, g1, CV_BGR2GRAY);
    cv::cvtColor(img2, g2, CV_BGR2GRAY);

    if (!(strcmp(method, "BM")))
    {
        cv::StereoBM sbm;
        sbm.state->SADWindowSize = 9;
        sbm.state->numberOfDisparities = 112;
        sbm.state->preFilterSize = 5;
        sbm.state->preFilterCap = 61;
        sbm.state->minDisparity = -39;
        sbm.state->textureThreshold = 507;
        sbm.state->uniquenessRatio = 0;
        sbm.state->speckleWindowSize = 0;
        sbm.state->speckleRange = 8;
        sbm.state->disp12MaxDiff = 1;
        sbm(g1, g2, disp);
    }
    else if (!(strcmp(method, "SGBM")))
    {
        cv::StereoSGBM sbm;
        sbm.SADWindowSize = 3;
        sbm.numberOfDisparities = 144;
        sbm.preFilterCap = 63;
        sbm.minDisparity = -39;
        sbm.uniquenessRatio = 10;
        sbm.speckleWindowSize = 100;
        sbm.speckleRange = 32;
        sbm.disp12MaxDiff = 1;
        sbm.fullDP = false;
        sbm.P1 = 216;
        sbm.P2 = 864;
        sbm(g1, g2, disp);
    }

    
    normalize(disp, disp8, 0, 255, CV_MINMAX, CV_8U);

   // cv::imshow("left", img1);
    //cv::imshow("right", img2);
    //cv::imshow("disp", disp8);

    //cv::waitKey(0);

    //return(0);
  
  
  //*******************************************************************************************************************************
  //Check arguments
  
  if (argc != 5)
  {
    std::cerr << "Usage: " << argv[0] << " <rgb-image-filename> <disparity-image-filename> <path-to-Q-matrix>" << std::endl;
    return 1;
  }

  //Load Matrix Q
  cv::FileStorage fs(argv[4], cv::FileStorage::READ);   //changed  3 to 4
  cv::Mat Q;
  
  fs["Q"] >> Q;
  
  //If size of Q is not 4x4 exit
  if (Q.cols != 4 || Q.rows != 4)
  {
    std::cerr << "ERROR: Could not read matrix Q (doesn't exist or size is not 4x4)" << std::endl;
    return 1;
  }

#ifdef CUSTOM_REPROJECT
  //Get the interesting parameters from Q
  double Q03, Q13, Q23, Q32, Q33;
  Q03 = Q.at<double>(0,3);
  Q13 = Q.at<double>(1,3);
  Q23 = Q.at<double>(2,3);
  Q32 = Q.at<double>(3,2);
  Q33 = Q.at<double>(3,3);
  
  std::cout << "Q(0,3) = "<< Q03 <<"; Q(1,3) = "<< Q13 <<"; Q(2,3) = "<< Q23 <<"; Q(3,2) = "<< Q32 <<"; Q(3,3) = "<< Q33 <<";" << std::endl;
  
#endif  
  
  
  std::cout << "Read matrix in file " << argv[4] << std::endl;       //changed 3 to 4

  //Show the values inside Q (for debug purposes)
  /*
  for (int y = 0; y < Q.rows; y++)
  {
    const double* Qy = Q.ptr<double>(y);
    for (int x = 0; x < Q.cols; x++)
    {
      std::cout << "Q(" << x << "," << y << ") = " << Qy[x] << std::endl;
    }
  }
  */
  
  //Load rgb-image
  cv::Mat img_rgb = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
  if (img_rgb.data == NULL)
  {
    std::cerr << "ERROR: Could not read rgb-image: " << argv[1] << std::endl;
    return 1;
  }
  
  //Load disparity image
 // cv::Mat img_disparity = cv::imread(disp8, CV_LOAD_IMAGE_GRAYSCALE);    //changed argv[2] to disp8
  cv::Mat img_disparity=disp8;                                             //changed replacement for above line 
  if (img_disparity.data == NULL)
  {
    std::cerr << "ERROR: Could not read disparity-image: " << argv[2] << std::endl;
    return 1;
  }
  
  //Both images must be same size
  if (img_rgb.size() != img_disparity.size())
  {
    std::cerr << "ERROR: rgb-image and disparity-image have different sizes " << std::endl;
    return 1;
  }
  
  //Show both images (for debug purposes)
  cv::namedWindow("rgb-image");
  cv::namedWindow("disparity-image");
  cv::imshow("rbg-image", img_rgb);
  cv::imshow("disparity-image", img_disparity);
  std::cout << "Press a key to continue..." << std::endl;
  //cv::waitKey(0);
  cv::destroyWindow("rgb-image");
  cv::destroyWindow("disparity-image");
  
#ifndef CUSTOM_REPROJECT
  //Create matrix that will contain 3D corrdinates of each pixel
  cv::Mat recons3D(img_disparity.size(), CV_32FC3);
  
  //Reproject image to 3D
  std::cout << "Reprojecting image to 3D..." << std::endl;
  cv::reprojectImageTo3D( img_disparity, recons3D, Q, false, CV_32F );
#endif  
  //Create point cloud and fill it
  std::cout << "Creating Point Cloud..." <<std::endl;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  double px, py, pz;
  uchar pr, pg, pb;
  
  for (int i = 0; i < img_rgb.rows; i++)
  {
    uchar* rgb_ptr = img_rgb.ptr<uchar>(i);
#ifdef CUSTOM_REPROJECT
    uchar* disp_ptr = img_disparity.ptr<uchar>(i);
#else
    double* recons_ptr = recons3D.ptr<double>(i);
#endif
    for (int j = 0; j < img_rgb.cols; j++)
    {
      //Get 3D coordinates
#ifdef CUSTOM_REPROJECT
      uchar d = disp_ptr[j];
      if ( d == 0 ) continue; //Discard bad pixels
      double pw = -1.0 * static_cast<double>(d) * Q32 + Q33; 
      px = static_cast<double>(j) + Q03;
      py = static_cast<double>(i) + Q13;
      pz = Q23;
      
      px = px/pw;
      py = py/pw;
      pz = pz/pw;

#else
      px = recons_ptr[3*j];
      py = recons_ptr[3*j+1];
      pz = recons_ptr[3*j+2];
#endif
      
      //Get RGB info
      pb = rgb_ptr[3*j];
      pg = rgb_ptr[3*j+1];
      pr = rgb_ptr[3*j+2];
      
      //Insert info into point cloud structure
      pcl::PointXYZRGB point;
      point.x = px;
      point.y = py;
      point.z = pz;
      uint32_t rgb = (static_cast<uint32_t>(pr) << 16 |
              static_cast<uint32_t>(pg) << 8 | static_cast<uint32_t>(pb));
      point.rgb = *reinterpret_cast<float*>(&rgb);
      point_cloud_ptr->points.push_back (point);
    }
  }
  point_cloud_ptr->width = (int) point_cloud_ptr->points.size();
  point_cloud_ptr->height = 1;
  
  //Create visualizer
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  viewer = createVisualizer( point_cloud_ptr );
  
  //Main loop
  while ( !viewer->wasStopped())
  {
    viewer->spinOnce(100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
  
  return 0;
}
