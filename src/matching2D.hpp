#ifndef matching2D_hpp
#define matching2D_hpp

#include <stdio.h>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>

// Try to include xfeatures2d for SIFT, BRIEF, FREAK support
// Available in OpenCV >= 4.3 or with opencv-contrib module
#ifdef __has_include
  #if __has_include(<opencv2/xfeatures2d.hpp>)
    #include <opencv2/xfeatures2d.hpp>
    #define HAS_XFEATURES2D 1
  #elif __has_include(<opencv2/xfeatures2d/nonfree.hpp>)
    #include <opencv2/xfeatures2d/nonfree.hpp>
    #define HAS_XFEATURES2D 1
  #else
    #define HAS_XFEATURES2D 0
  #endif
#else
  #define HAS_XFEATURES2D 0  // Assume not available if compiler doesn't support __has_include
#endif

#include "dataStructures.h"


void detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis=false);


void detKeypointsShiTomasi(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis=false);


void detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, 
                        std::string detectorType, bool bVis=false);


void descKeypoints(std::vector<cv::KeyPoint> &keypoints, 
                  cv::Mat &img, cv::Mat &descriptors, 
                  std::string descriptorType);


void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, 
                      std::vector<cv::KeyPoint> &kPtsRef, 
                      cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, 
                      std::string descriptorType, 
                      std::string matcherType, 
                      std::string selectorType);

#endif /* matching2D_hpp */
