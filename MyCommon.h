//
// Created by nannan on 2019/11/15.
//

#ifndef TRIANGULATIONFROM2IMGS_MYCOMMON_H
#define TRIANGULATIONFROM2IMGS_MYCOMMON_H

# include <string>
#include <opencv2/core/mat.hpp>
#include <map>

using namespace std;
using namespace cv;


const string totalPath="/Users/nannan/CLionProjects/TriangulationFrom2Imgs/";
const string imageSourcePath=totalPath+"MyPump01/";
const string myOutputPath=totalPath+"traingle_out/";

const float mDownscaleFactor=1.0;
const double NN_MATCH_RATIO = 1.0f; // Nearest-neighbour matching ratio,used by matchingutils,1.0f initial
const float MIN_REPROJECTION_ERROR = 10.0; // Maximum 10-pixel allowed re-projection error,used in StereoUtils
const int findHInliers_match_size=4; //original is 4, used in stereoUtils
const double RANSAC_THRESHOLD_H=10.0f;//original 10.0f, RANSAC inlier threshold ,used in StereoUtils,findHomography

typedef std::vector<string> ImgNamesVect;
typedef std::vector<Mat> ImgsVect;

///1. Used in Features.h/.cpp

typedef std::vector<cv::KeyPoint> Keypoints;
typedef std::vector<cv::Point2f> Points2f;

struct Features {
    Keypoints keyPoints;
    Points2f points;
    cv::Mat descriptors;
};


typedef std::vector<Features> FeaturesVect;

/**
 * Convert Keypoints to Points2f
 * @param kps keypoints
 * @param ps  points
 */
void KeyPointsToPoints(const Keypoints &kps, Points2f &ps);

/**
 * Convert Points2f to Keypoints.
 * Note: distance on Keypoint will be set to 1.0.
 * @param ps  Points
 * @param kps Keypoints
 */
void PointsToKeyPoints(const Points2f &ps, Keypoints &kps);



///2. Used in MatchingUtils.h/cpp

typedef std::vector<cv::DMatch> Matching;
struct ImagePair {
    size_t left, right;
};

/////////////////cv::Ptr<cv::DescriptorMatcher> mMatcher;
typedef std::vector<std::vector<Matching> > MatchMatrix;


///3. used in StereoUtils
struct Point3DInMap {
    // 3D point.
    cv::Point3f p;
    // A mapping from image index to 2D point index in that image's list of features.
    std::map<int, int> originatingViews;
};
typedef std::vector<cv::Point3f> Points3f;
typedef std::vector<Point3DInMap> PointCloud;
//intrisic parameter of the camera
struct Intrinsics {
    cv::Mat K;
    cv::Mat Kinv;
    cv::Mat distortion;
};


#endif //TRIANGULATIONFROM2IMGS_MYCOMMON_H
