//
// Created by nannan on 2019/11/15.
//

#ifndef TRIANGULATIONFROM2IMGS_STEREOUTILS_H
#define TRIANGULATIONFROM2IMGS_STEREOUTILS_H

#include "MyCommon.h"
#include "MatchingUtils.h"
#include <iostream>

using namespace cv;

class StereoUtils {
public:

    StereoUtils();


    /**
     * Find camera matrices (3x4 poses) from stereo point matching.
     * @param intrinsics      Camera intrinsics (assuming both cameras have the same parameters)
     * @param featureMatching Matching between left and right features
     * @param featuresLeft    Features in left image
     * @param featuresRight   Features in right image
     * @param prunedMatches   Output: matching after pruning using essential matrix
     * @param Pleft           Output: left image matrix (3x4)
     * @param Pright          Output: right image matrix (3x4)
     * @return true on success.
     */
    void findCameraMatricesFromMatch(
            const Intrinsics &intrinsics,
            const Matching &featureMatching,
            const Features &featuresLeft,
            const Features &featuresRight,
            Matching &prunedMatches,
            cv::Matx34f &Pleft,
            cv::Matx34f &Pright);

    /**
     * Triangulate (recover 3D locations) from point matching.
     * @param imagePair     Indices of left and right views
     * @param leftFeatures  Left image features
     * @param rightFeatures Right image features
     * @param Pleft         Left camera matrix
     * @param Pright        Right camera matrix
     * @param pointCloud    Output: point cloud with image associations
     * @return true on success.
     */
    void triangulateViews(
            const Intrinsics &intrinsics,
            const ImagePair imagePair,
            const Matching &matches,
            const Features &leftFeatures,
            const Features &rightFeatures,
            const cv::Matx34f &Pleft,
            const cv::Matx34f &Pright,
            PointCloud &pointCloud
    );


};


#endif //TRIANGULATIONFROM2IMGS_STEREOUTILS_H
