//
// Created by nannan on 2019/11/15.
//


#include <opencv/cv.hpp>
#include <fstream>
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

    cv::Mat myE, myR, myT;
    cv::Mat mask;
    myE = findEssentialMat(alignedLeft.points, alignedRight.points, focal, pp, cv::RANSAC, 0.99, 1.5, mask);

    //Find Pright camera matrix from the essential matrix
    //Cheirality check (all points are in front of camera) is performed internally.
    recoverPose(myE, alignedLeft.points, alignedRight.points, myR, myT, focal, pp, mask);

    //TODO: stratify over Pleft
    Pleft = cv::Matx34f::eye();
    Pright = cv::Matx34f(myR.at<double>(0, 0), myR.at<double>(0, 1), myR.at<double>(0, 2), myT.at<double>(0),
                         myR.at<double>(1, 0), myR.at<double>(1, 1), myR.at<double>(1, 2), myT.at<double>(1),
                         myR.at<double>(2, 0), myR.at<double>(2, 1), myR.at<double>(2, 2), myT.at<double>(2));

    //populate pruned matches
    prunedMatches.clear();
    for (size_t i = 0; i < mask.rows; i++) {
        if (mask.at<uchar>(i)) {
            prunedMatches.push_back(matches[i]);
        }
    }

    ofstream fout(myOutputPath+"cameraMatrix.txt");
    fout << myE <<endl;
    fout << myR << endl;
    fout << myT <<endl;

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


    for (size_t i = 0; i < points3d.rows; i++) {

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
    cout<<pointCloud.size()<<endl;
    ofstream fout(myOutputPath+"pointcloud.ply");
    fout << reinterpret_cast<Range &>(pointCloud) << endl;

    //return true;
}
