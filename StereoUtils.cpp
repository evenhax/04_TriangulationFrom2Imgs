//
// Created by nannan on 2019/11/15.
//


#include <opencv/cv.hpp>
#include "StereoUtils.h"

using namespace std;
using namespace cv;


StereoUtils::StereoUtils() {

}

void StereoUtils::findCameraMatricesFromMatch(
        const Intrinsics &intrinsics,
        const Matching &matches,
        const Features &featuresLeft,
        const Features &featuresRight,
        Matching &prunedMatches,
        cv::Matx34f &Pleft,
        cv::Matx34f &Pright) {

    if (intrinsics.K.empty()) {
        cerr << "Intrinsics matrix (K) must be initialized." << endl;
        //return false;
    }

    cout << "Start the findCameraMatricesFromMatch" << endl;
    double focal = intrinsics.K.at<float>(0, 0); //Note: assuming fx = fy
    cv::Point2d pp(intrinsics.K.at<float>(0, 2), intrinsics.K.at<float>(1, 2));

    Features alignedLeft;
    Features alignedRight;
    MatchingUtils::GetAlignedPointsFromMatch(featuresLeft, featuresRight, matches, alignedLeft, alignedRight);

    cv::Mat E, R, t;
    cv::Mat mask;
    E = findEssentialMat(alignedLeft.points, alignedRight.points, focal, pp, cv::RANSAC, 0.99, 1.5, mask);

    //Find Pright camera matrix from the essential matrix
    //Cheirality check (all points are in front of camera) is performed internally.
    recoverPose(E, alignedLeft.points, alignedRight.points, R, t, focal, pp, mask);

    //TODO: stratify over Pleft
    Pleft = cv::Matx34f::eye();
    Pright = cv::Matx34f(R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0),
                         R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1),
                         R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2));

    //populate pruned matches
    prunedMatches.clear();
    for (size_t i = 0; i < mask.rows; i++) {
        if (mask.at<uchar>(i)) {
            prunedMatches.push_back(matches[i]);
        }
    }


    //return true;
}

void StereoUtils::triangulateViews(
        const Intrinsics &intrinsics,
        const ImagePair imagePair,
        const Matching &matches,
        const Features &featuresLeft,
        const Features &featuresRight,
        const cv::Matx34f &Pleft,
        const cv::Matx34f &Pright,
        PointCloud &pointCloud) {

    //get aligned features left-right, with back reference to original indexing
    std::vector<int> leftBackReference;
    std::vector<int> rightBackReference;
    Features alignedLeft;
    Features alignedRight;
    MatchingUtils::GetAlignedPointsFromMatch(
            featuresLeft,
            featuresRight,
            matches,
            alignedLeft,
            alignedRight,
            leftBackReference,
            rightBackReference);

    cv::Mat normalizedLeftPts;
    cv::Mat normalizedRightPts;
    undistortPoints(alignedLeft.points, normalizedLeftPts, intrinsics.K, cv::Mat());
    undistortPoints(alignedRight.points, normalizedRightPts, intrinsics.K, cv::Mat());

    cv::Mat points3dHomogeneous;
    triangulatePoints(Pleft, Pright, normalizedLeftPts, normalizedRightPts, points3dHomogeneous);

    cv::Mat points3d;
    convertPointsFromHomogeneous(points3dHomogeneous.t(), points3d);

    cv::Mat rvecLeft;
    Rodrigues(Pleft.get_minor<3, 3>(0, 0), rvecLeft);
    cv::Mat tvecLeft(Pleft.get_minor<3, 1>(0, 3).t());

    std::vector<cv::Point2f> projectedOnLeft(alignedLeft.points.size());
    projectPoints(points3d, rvecLeft, tvecLeft, intrinsics.K, cv::Mat(), projectedOnLeft);

    cv::Mat rvecRight;
    Rodrigues(Pright.get_minor<3, 3>(0, 0), rvecRight);
    cv::Mat tvecRight(Pright.get_minor<3, 1>(0, 3).t());

    std::vector<cv::Point2f> projectedOnRight(alignedRight.points.size());
    projectPoints(points3d, rvecRight, tvecRight, intrinsics.K, cv::Mat(), projectedOnRight);

//    {
//		Mat outLeft(2048, 3072, CV_8UC3, Colors::BLACK);
//		drawKeypoints(outLeft, alignedLeft.keyPoints, outLeft, Colors::RED);
//		drawKeypoints(outLeft, PointsToKeyPoints(projectedOnLeft), outLeft, Colors::GREEN);
//		Mat outRight(2048, 3072, CV_8UC3, Colors::BLACK);
//		drawKeypoints(outRight, alignedRight.keyPoints, outRight, Colors::RED);
//		drawKeypoints(outRight, PointsToKeyPoints(projectedOnRight), outRight, Colors::GREEN);
//		Mat tmp;
//		hconcat(outLeft, outRight, tmp);
//		imshow("features", tmp, 0.25);
//		waitKey(0);
//    }

    //Note: cheirality check (all points z > 0) was already performed at camera pose calculation

    for (size_t i = 0; i < points3d.rows; i++) {
        //check if point reprojection error is small enough
        if (norm(projectedOnLeft[i] - alignedLeft.points[i]) > MIN_REPROJECTION_ERROR or
            norm(projectedOnRight[i] - alignedRight.points[i]) > MIN_REPROJECTION_ERROR) {
            continue;
        }

        Point3DInMap p;
        p.p = cv::Point3f(points3d.at<float>(i, 0),
                          points3d.at<float>(i, 1),
                          points3d.at<float>(i, 2)
        );

        //use back reference to point to original features in images
        p.originatingViews[imagePair.left] = leftBackReference[i];
        p.originatingViews[imagePair.right] = rightBackReference[i];

        pointCloud.push_back(p);
    }

    //return true;
}