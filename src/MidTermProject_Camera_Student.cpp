/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"

using namespace std;

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{

    /* INIT VARIABLES AND DATA STRUCTURES */

    // data location
    string dataPath = "../";

    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 9;   // last file index to load - 10 images TOTAL
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // misc
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
    vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
    bool bVis = false;            // visualize results

    // TASK MP.7, MP.8, MP.9: Logging to CSV files
    ofstream keypointLog("../keypoint_log.csv");
    ofstream matchLog("../match_log.csv");
    
    // Write headers
    keypointLog << "ImageIndex,DetectorType,NumKeypoints,MinSize,MaxSize,MeanSize" << endl;
    matchLog << "ImageIndex,DetectorType,DescriptorType,NumMatches" << endl;

    // All detector and descriptor types to test
    vector<string> detectorTypes = {"SHITOMASI", "HARRIS", "FAST", "BRISK", "ORB", "AKAZE", "SIFT"};
    vector<string> descriptorTypes = {"BRISK", "ORB", "AKAZE", "SIFT", "BRIEF", "FREAK"};

    /* MAIN LOOP OVER ALL DETECTORS AND DESCRIPTORS */
    for (string detectorType : detectorTypes)
    {
        for (string descriptorTypeSelected : descriptorTypes)
        {
            // AKAZE descriptors can only be used with AKAZE detector
            if (descriptorTypeSelected == "AKAZE" && detectorType != "AKAZE")
            {
                continue; // Skip this combination
            }

            cout << "\n========================================" << endl;
            cout << "Testing: " << detectorType << " + " << descriptorTypeSelected << endl;
            cout << "========================================" << endl;

            dataBuffer.clear(); // Clear buffer for new detector/descriptor combination

            /* MAIN LOOP OVER ALL IMAGES */
            for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
            {
                /* LOAD IMAGE INTO BUFFER */

                // assemble filenames for current index
                ostringstream imgNumber;
                imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
                string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

                // load image from file and convert to grayscale
                cv::Mat img, imgGray;
                img = cv::imread(imgFullFilename);
                cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

                //// STUDENT ASSIGNMENT
                //// TASK MP.1 -> replace the following code with ring buffer of size dataBufferSize

                // push image into data frame buffer
                DataFrame frame;
                frame.cameraImg = imgGray;
                
                // Maintain ring buffer: remove oldest element if at capacity, then add new element
                if (dataBuffer.size() == dataBufferSize)
                {
                    dataBuffer.erase(dataBuffer.begin());
                }
                dataBuffer.push_back(frame);

                //// EOF STUDENT ASSIGNMENT
                cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;

                /* DETECT IMAGE KEYPOINTS */

                // extract 2D keypoints from current image
                vector<cv::KeyPoint> keypoints; // create empty feature list for current image

                //// STUDENT ASSIGNMENT
                //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
                //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
                
                if (detectorType.compare("SHITOMASI") == 0)
                {
                    detKeypointsShiTomasi(keypoints, imgGray, false);
                }
                else if(detectorType.compare("HARRIS") == 0)
                {
                    detKeypointsHarris(keypoints, imgGray, false);
                }
                else if(detectorType.compare("FAST") == 0)
                {
                    detKeypointsModern(keypoints, imgGray, "FAST", false);
                }
                else if(detectorType.compare("BRISK") == 0)
                {
                    detKeypointsModern(keypoints, imgGray, "BRISK", false);
                }
                else if(detectorType.compare("ORB") == 0)
                {
                    detKeypointsModern(keypoints, imgGray, "ORB", false);
                }
                else if(detectorType.compare("AKAZE") == 0)
                {
                    detKeypointsModern(keypoints, imgGray, "AKAZE", false);
                }
                else if(detectorType.compare("SIFT") == 0)
                {
                    detKeypointsModern(keypoints, imgGray, "SIFT", false);
                }

                else
                {
                    cout << "Detector type not recognized. Please choose one of the following: SHITOMASI, HARRIS, FAST, BRISK, ORB, AKAZE, SIFT" << endl;
                }

                //// EOF STUDENT ASSIGNMENT

                //// STUDENT ASSIGNMENT
                //// TASK MP.3 -> only keep keypoints on the preceding vehicle


                // only keep keypoints on the preceding vehicle
                bool bFocusOnVehicle = true;
                cv::Rect vehicleRect(535, 180, 180, 150);
                if (bFocusOnVehicle)
                {
                    auto it = keypoints.begin();
                    while(it != keypoints.end())
                    {
                        if(!vehicleRect.contains(it->pt))
                        {
                            it = keypoints.erase(it);
                        }
                        else
                        {
                            ++it;
                        }
                    }
                }

                // 7. Log keypoint statistics
                int num_kpts_precedingVehicle = keypoints.size();
                float minSize = 1e9, maxSize = 0.0, meanSize = 0.0;
                if (num_kpts_precedingVehicle > 0)
                {
                    for (auto& kpt : keypoints)
                    {
                        minSize = std::min(minSize, kpt.size);
                        maxSize = std::max(maxSize, kpt.size);
                        meanSize += kpt.size;
                    }
                    meanSize /= num_kpts_precedingVehicle;
                }
                else
                {
                    minSize = 0.0;
                }
                keypointLog << imgIndex << "," << detectorType << "," << num_kpts_precedingVehicle 
                            << "," << minSize << "," << maxSize << "," << meanSize << endl;
                cout << "Image " << imgIndex << " - " << detectorType << ": " << num_kpts_precedingVehicle 
                    << " keypoints (Size - Min: " << minSize << ", Max: " << maxSize << ", Mean: " << meanSize << ")" << endl;

                //// EOF STUDENT ASSIGNMENT

                // optional : limit number of keypoints (helpful for debugging and learning)
                bool bLimitKpts = false;
                if (bLimitKpts)
                {
                    int maxKeypoints = 50;

                    if (detectorType.compare("SHITOMASI") == 0)
                    { // there is no response info, so keep the first 50 as they are sorted in descending quality order
                        keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
                    }
                    cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
                    cout << " NOTE: Keypoints have been limited!" << endl;
                }

                // push keypoints and descriptor for current frame to end of data buffer
                (dataBuffer.end() - 1)->keypoints = keypoints;
                cout << "#2 : DETECT KEYPOINTS done" << endl;

                /* EXTRACT KEYPOINT DESCRIPTORS */

                //// STUDENT ASSIGNMENT
                //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
                //// -> BRIEF, ORB, FREAK, AKAZE, SIFT

                cv::Mat descriptors;
                descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorTypeSelected);
                //// EOF STUDENT ASSIGNMENT

                // push descriptors for current frame to end of data buffer
                (dataBuffer.end() - 1)->descriptors = descriptors;

                cout << "#3 : EXTRACT DESCRIPTORS done" << endl;

                if (dataBuffer.size() > 1) // wait until at least two images have been processed
                {

                    /* MATCH KEYPOINT DESCRIPTORS */

                    vector<cv::DMatch> matches;
                    string matcherType = "MAT_BF";        // MAT_BF, MAT_FLANN
                    string selectorType = "SEL_KNN";       // SEL_NN, SEL_KNN (using KNN with ratio test)

                    //// STUDENT ASSIGNMENT
                    //// TASK MP.5 -> add FLANN matching in file matching2D_Student.cpp

                    //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D_Student.cpp

                    matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                                    (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                                    matches, descriptorTypeSelected, matcherType, selectorType);

                    //// EOF STUDENT ASSIGNMENT

                    // store matches in current data frame
                    (dataBuffer.end() - 1)->kptMatches = matches;

                    // TASK MP.8: Log match statistics
                    matchLog << imgIndex << "," << detectorType << "," << descriptorTypeSelected << "," << matches.size() << endl;
                    cout << "Image " << imgIndex << " - " << detectorType << "/" << descriptorTypeSelected << ": " << matches.size() << " matches" << endl;

                    cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << endl;

                    // visualize matches between current and previous image
                    cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
                    cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints,
                                    (dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints,
                                    matches, matchImg,
                                    cv::Scalar::all(-1), cv::Scalar::all(-1),
                                    vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                    // Save match visualization image to outputs folder
                    string outputDir = dataPath + "images/outputs/";
                    stringstream ss;
                    ss << outputDir << "match_" << detectorType << "_" << descriptorTypeSelected 
                       << "_frames_" << (imgIndex-1) << "_" << imgIndex << ".png";
                    cv::imwrite(ss.str(), matchImg);

                    bVis = false;
                    if (bVis)
                    {
                        string windowName = "Matching keypoints between two camera images";
                        cv::namedWindow(windowName, 7);
                        cv::imshow(windowName, matchImg);
                        cout << "Press key to continue to next image" << endl;
                        cv::waitKey(0); // wait for key to be pressed
                    }
                    bVis = false;
                }

            } // eof loop over all images
        } // eof loop over all descriptors
    } // eof loop over all detectors

    // Close log files
    keypointLog.close();
    matchLog.close();
    cout << "\n=== Analysis Complete ==="  << endl;
    cout << "Keypoint statistics saved to: ../keypoint_log.csv" << endl;
    cout << "Match statistics saved to: ../match_log.csv" << endl;
    cout << "Match images saved to: ../images/outputs/" << endl;
    cout << "Timing information is printed during execution." << endl;

    return 0;
}