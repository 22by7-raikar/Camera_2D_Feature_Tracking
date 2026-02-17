#include <numeric>
#include "matching2D.hpp"

using namespace std;

// 5. Find best matches for keypoints in two camera images based on several matching methods
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, std::string descriptorType, std::string matcherType, std::string selectorType)
{
    // configure matcher
    bool crossCheck = false;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    // Determine norm type based on descriptor type
    int normType;
    if (descriptorType.compare("SIFT") == 0)
    {
        normType = cv::NORM_L2;  // SIFT uses float descriptors with L2 norm
    }
    else
    {
        normType = cv::NORM_HAMMING;  // BRISK, ORB, AKAZE, BRIEF, FREAK use binary descriptors with Hamming norm
    }

    if (matcherType.compare("MAT_BF") == 0)
    {
        matcher = cv::BFMatcher::create(normType, crossCheck);
    }
    else if (matcherType.compare("MAT_FLANN") == 0)
    {
        // FLANN matcher automatically handles different descriptor types
        matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    }

    // perform matching task
    if (selectorType.compare("SEL_NN") == 0)
    { // nearest neighbor (best match)

        matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in desc1
    }
    else if (selectorType.compare("SEL_KNN") == 0)
    { // k nearest neighbors (k=2)

        std::vector<std::vector<cv::DMatch>> knn_matches;
        /* DMatch contains:
        The index of the keypoint in the first image (queryIdx)
        The index of the keypoint in the second image (trainIdx)
        The index of the image in the training set (imgIdx, used for multi-image matching)
        The distance between the two descriptors (distance), where a smaller value means a better match*/
        
        matcher->knnMatch(descSource, descRef, knn_matches, 2);

        // 6. filter matches using descriptor distance ratio test
        const float ratio_thresh = 0.8f;
        for(const auto& knn_match : knn_matches)
        {
            if(knn_match[0].distance < ratio_thresh * knn_match[1].distance)
            {
                matches.push_back(knn_match[0]);
            }
        }
    }
}

// 4. Use one of several types of state-of-art descriptors to uniquely identify keypoints
void descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, string descriptorType)
{
    // select appropriate descriptor
    cv::Ptr<cv::DescriptorExtractor> extractor;
            }
        }
    }
}

// 4. Use one of several types of state-of-art descriptors to uniquely identify keypoints
void descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, string descriptorType)
{
    // select appropriate descriptor
    cv::Ptr<cv::DescriptorExtractor> extractor;
    if (descriptorType.compare("BRISK") == 0)
    {

        int threshold = 30;        // FAST/AGAST detection threshold score.
        int octaves = 3;           // detection octaves (use 0 to do single scale)
        float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

        extractor = cv::BRISK::create(threshold, octaves, patternScale);
    }

    else if (descriptorType.compare("ORB") == 0)
    {
        int nfeatures = 500;                    // The maximum number of features to retain
        float scaleFactor = 1.2f;               // Pyramid decimation ratio
        int nlevels = 8;                        // The number of pyramid levels
        int edgeThreshold = 31;                 // This is size of the border where the features are not detected
        int firstLevel = 0;                     // The level of pyramid to put source image to
        int WTA_K = 2;                          // The number of random pixels within each cell of each level of the pyramid
        cv::ORB::ScoreType scoreType = cv::ORB::HARRIS_SCORE; // HARRIS_SCORE, FAST_SCORE
        int patchSize = 31;                     // size of the patch used by the oriented BRIEF descriptor
        int fastThreshold = 20;                 // the fast threshold

        extractor = cv::ORB::create(nfeatures, scaleFactor, nlevels, edgeThreshold, firstLevel, WTA_K, scoreType, patchSize, fastThreshold);
    }

    else if (descriptorType.compare("AKAZE") == 0)
    {
        cv::AKAZE::DescriptorType akDescriptorType = cv::AKAZE::DESCRIPTOR_MLDB;
        int descriptorSize = 0;
        int descriptorChannels = 3;
        float threshold = 0.001f;
        int nOctaves = 4;
        int nOctaveLayers = 4;
        cv::KAZE::DiffusivityType diffusivity = cv::KAZE::DIFF_PM_G2;

        extractor = cv::AKAZE::create(akDescriptorType, descriptorSize, descriptorChannels, threshold, nOctaves, nOctaveLayers, diffusivity);
    }

    else if (descriptorType.compare("SIFT") == 0)
    {
#if HAS_XFEATURES2D
        // SIFT from opencv_contrib (OpenCV >= 4.3 or with opencv-contrib installed)
        extractor = cv::xfeatures2d::SIFT::create();
#else
        // Fallback: xfeatures2d not available, use BRISK instead
        cout << "NOTE: SIFT descriptor requires opencv-contrib module (see README). Falling back to BRISK." << endl;
        int threshold = 30;
        int octaves = 3;
        float patternScale = 1.0f;
        extractor = cv::BRISK::create(threshold, octaves, patternScale);
#endif
    }

    else if (descriptorType.compare("BRIEF") == 0)
    {
#if HAS_XFEATURES2D
        // BRIEF from opencv_contrib
        int bytes = 32;  // length of brief descriptor in bytes
        bool useOrientation = false;  // default BRIEF is not rotation invariant
        extractor = cv::xfeatures2d::BriefDescriptorExtractor::create(bytes, useOrientation);
#else
        // Fallback: xfeatures2d not available, use ORB instead
        cout << "NOTE: BRIEF descriptor requires opencv-contrib module (see README). Falling back to ORB." << endl;
        int nfeatures = 500;
        float scaleFactor = 1.2f;
        int nlevels = 8;
        int edgeThreshold = 31;
        int firstLevel = 0;
        int WTA_K = 2;
        cv::ORB::ScoreType scoreType = cv::ORB::HARRIS_SCORE;
        int patchSize = 31;
        int fastThreshold = 20;
        extractor = cv::ORB::create(nfeatures, scaleFactor, nlevels, edgeThreshold, firstLevel, WTA_K, scoreType, patchSize, fastThreshold);
#endif
    }

    else if (descriptorType.compare("FREAK") == 0)
    {
#if HAS_XFEATURES2D
        // FREAK from opencv_contrib
        bool orientationNormalized = true;
        bool scaleNormalized = true;
        float patternScale = 22.0f;
        int nOctaves = 4;
        extractor = cv::xfeatures2d::FREAK::create(orientationNormalized, scaleNormalized, patternScale, nOctaves);
#else
        // Fallback: xfeatures2d not available, use ORB instead
        cout << "NOTE: FREAK descriptor requires opencv-contrib module (see README). Falling back to ORB." << endl;
        int nfeatures = 500;
        float scaleFactor = 1.2f;
        int nlevels = 8;
        int edgeThreshold = 31;
        int firstLevel = 0;
        int WTA_K = 2;
        cv::ORB::ScoreType scoreType = cv::ORB::HARRIS_SCORE;
        int patchSize = 31;
        int fastThreshold = 20;
        extractor = cv::ORB::create(nfeatures, scaleFactor, nlevels, edgeThreshold, firstLevel, WTA_K, scoreType, patchSize, fastThreshold);
#endif
    }

    else
    {
        cout << "Descriptor type not recognized. Please choose one of the following: BRISK, ORB, AKAZE, SIFT, BRIEF, FREAK" << endl;
        return;
    }
    
    // perform feature description
    double t = (double)cv::getTickCount();
    extractor->compute(img, keypoints, descriptors);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << descriptorType << " descriptor extraction in " << 1000 * t / 1.0 << " ms" << endl;
}

// Detect keypoints in image using the traditional Shi-Thomasi detector
void detKeypointsShiTomasi(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    // compute detector parameters based on image size
    int blockSize = 4;       //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
    double maxOverlap = 0.0; // max. permissible overlap between two features in %
    double minDistance = (1.0 - maxOverlap) * blockSize;
    int maxCorners = img.rows * img.cols / max(1.0, minDistance); // max. num. of keypoints

    double qualityLevel = 0.01; // minimal accepted quality of image corners
    double k = 0.04;

    // Apply corner detection
    double t = (double)cv::getTickCount();
    vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, false, k);

    // add corners to result vector
    for (auto it = corners.begin(); it != corners.end(); ++it)
    {

        cv::KeyPoint newKeyPoint;
        newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
        newKeyPoint.size = blockSize;
        keypoints.push_back(newKeyPoint);
    }
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "Shi-Tomasi detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "Shi-Tomasi Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}

// Detect keypoints in image using the Harris corner detector
void detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    // compute detector parameters based on image size
    int blockSize = 2;     // for every pixel, a blockSize × blockSize neighborhood is considered
    int apertureSize = 3;  // aperture parameter for Sobel operator (must be odd)
    /* Sobel operator is used to compute image gradients in x and y direction
    which are then used to calculate the Harris response for each pixel. 
    The aperture is the size of the Sobel kernel used to compute these gradients.
    A larger aperture will consider a wider neighborhood for gradient calculation,
    which can lead to more stable keypoint detection but may also increase computational cost and reduce sensitivity to fine details.
    */
   
    int minResponse = 100; // minimum value for a corner in the 8bit scaled response matrix
    double k = 0.04;       // Harris parameter
    double maxOverlap = 0.0; // max. permissible overlap between two features in %

    // Apply Harris corner detection
    double t = (double)cv::getTickCount();
    cv::Mat harrisRes = cv::Mat::zeros(img.size(), CV_32FC1);
    cv::Mat harrisResNorm, harrisResNormScaled;
    cv::cornerHarris(img, harrisRes, blockSize, apertureSize, k);
    cv::normalize(harrisRes, harrisResNorm, 0, 255, cv::NORM_MINMAX, CV_32F, cv::Mat());
    cv::convertScaleAbs(harrisResNorm, harrisResNormScaled);

    // Non-maximum suppression and thresholding to add keypoints
    for (size_t j = 0; j < harrisResNorm.rows; j++)
    {
        for (size_t i = 0; i < harrisResNorm.cols; i++)
        {
            int response = (int)harrisResNorm.at<float>(j, i); // response value of current pixel (row-major order)

            if (response > minResponse)
            {
                cv::KeyPoint newKeyPoint;
                newKeyPoint.pt = cv::Point2f(i, j);
                newKeyPoint.size = 2 * apertureSize;
                newKeyPoint.response = response;

                // perform non-maximum suppression (NMS) in local neighbourhood
                bool bOverlap = false; // flag indicating whether overlap has been found
                for (auto it = keypoints.begin(); it != keypoints.end(); ++it)
                {
                    double kptOverlap = cv::KeyPoint::overlap(newKeyPoint, *it);
                    if (kptOverlap > maxOverlap)
                    {
                        bOverlap = true;
                        if (newKeyPoint.response > (*it).response)
                        {
                            *it = newKeyPoint;
                            break;
                        }
                    }
                }
                if (!bOverlap)
                {
                    keypoints.push_back(newKeyPoint);
                }
            }
        }
    }

    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "Harris detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "Harris Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}

// Detect keypoints using modern methods (SIFT, FAST, BRISK, ORB, AKAZE)
void detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, std::string detectorType, bool bVis)
{
    cv::Ptr<cv::FeatureDetector> detector;

    if(detectorType.compare("SIFT")==0)
    {
#if HAS_XFEATURES2D
        // Scale-Invariant Feature Transform (SIFT) detector from opencv_contrib
        // Requires OpenCV >= 4.3 or opencv-contrib module
        detector = cv::xfeatures2d::SIFT::create();
#else
        // Fallback: xfeatures2d not available, use AKAZE instead
        cout << "NOTE: SIFT detector requires opencv-contrib module (see README). Falling back to AKAZE." << endl;
        cv::AKAZE::DescriptorType akDescriptorType = cv::AKAZE::DESCRIPTOR_MLDB;
        int descriptorSize = 0;
        int descriptorChannels = 3;
        float threshold = 0.001f;
        int nOctaves = 4;
        int nOctaveLayers = 4;
        cv::KAZE::DiffusivityType diffusivity = cv::KAZE::DIFF_PM_G2;
        detector = cv::AKAZE::create(akDescriptorType, descriptorSize, descriptorChannels, threshold, nOctaves, nOctaveLayers, diffusivity);
#endif
    }

    else if(detectorType.compare("FAST")==0)
    {
        int threshold = 30; // difference between intensity of the central pixel and pixels of a circle around this pixel
        
        /* FAST(Features from Accelerated Segment Test) tests 16 pixels in a circle around a candidate pixel. 
        A corner is detected if n contiguous pixels are all brighter or darker than the center pixel by a set threshold.*/

        bool bNMS = true;   // perform non-maxima suppression on keypoints
        cv::FastFeatureDetector::DetectorType type = cv::FastFeatureDetector::TYPE_9_16; // TYPE_9_16, TYPE_7_12, TYPE_5_8
        //TYPE_9_16 is the values of n and the total number of pixels in the circle
        detector = cv::FastFeatureDetector::create(threshold, bNMS, type);
    }


    else if(detectorType.compare("BRISK")==0)
    {
        /* Binary Robust Invariant Scalable Keypoints : 
        It is a multi-scale extension of the FAST detector that adds scale and rotation invariance while maintaining high computational efficiency.*/
        int threshold = 30;        // FAST detection threshold score
        int octaves = 3;           // detection octaves (use 0 to do single scale) - Number of pyramid levels -scales of feature detection
        float patternScale = 1.0f; 
        /* Rescales the sampling pattern applied to the image - apply this scale to the pattern used for sampling the neighbourhood of a keypoint.
        It essentially controls how much of the surrounding area is "read" to generate the 512-bit binary descriptor.*/
        detector = cv::BRISK::create(threshold, octaves, patternScale);
    }


    else if(detectorType.compare("ORB")==0)
    {
        /*oFAST (Oriented FAST): It uses FAST to find keypoints but adds a Harris corner measure to rank them, keeping only the best nfeatures. 
                                It calculates orientation by finding the intensity-weighted centroid of the patch around the keypoint.
        rBRIEF (Rotated BRIEF): It uses the BRIEF descriptor but "steers" (rotates) the sampling pattern according to the keypoint's orientation, making the descriptor rotation-invariant. */
        
        int nfeatures = 500;                    // The maximum number of features to retain
        float scaleFactor = 1.2f;               // Pyramid decimation ratio
        int nlevels = 8;                        // The number of pyramid levels
        int edgeThreshold = 31;                 // This is size of the border where the features are not detected
        int firstLevel = 0;                     // The level of pyramid to put source image to
        int WTA_K = 2;                          // The number of random pixels within each cell of each level of the pyramid
        cv::ORB::ScoreType scoreType = cv::ORB::HARRIS_SCORE; // HARRIS_SCORE, FAST_SCORE
        int patchSize = 31;                     // size of the patch used by the oriented BRIEF descriptor
        int fastThreshold = 20;                 // the fast threshold
        detector = cv::ORB::create(nfeatures, scaleFactor, nlevels, edgeThreshold, firstLevel, WTA_K, scoreType, patchSize, fastThreshold);
    }


    else if(detectorType.compare("AKAZE")==0)
    {
        cv::AKAZE::DescriptorType akDescriptorType = cv::AKAZE::DESCRIPTOR_MLDB;
        int descriptorSize = 0;
        int descriptorChannels = 3;
        float threshold = 0.001f;
        int nOctaves = 4;
        int nOctaveLayers = 4;
        cv::KAZE::DiffusivityType diffusivity = cv::KAZE::DIFF_PM_G2;
        detector = cv::AKAZE::create(akDescriptorType, descriptorSize, descriptorChannels, threshold, nOctaves, nOctaveLayers, diffusivity);
    }

    else
    {
        cout << "Detector type not recognized. Please choose one of the following: SIFT, FAST, BRISK, ORB, AKAZE" << endl;
        return;
    }


    double t = (double)cv::getTickCount();
    detector->detect(img, keypoints); // Detect keypoints
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << detectorType << " detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = detectorType + " Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}