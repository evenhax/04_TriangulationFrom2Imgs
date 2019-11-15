//
// Created by nannan on 2019/11/15.
//

#ifndef TRIANGULATIONFROM2IMGS_MATCHINGUTILS_H
#define TRIANGULATIONFROM2IMGS_MATCHINGUTILS_H

#include "MyCommon.h"

class MatchingUtils {


public:

    /**
 * Get the features for left and right images after keeping only the matched features and aligning them.
 * Alignment: i-th feature in left is a match to i-th feature in right.
 * @param leftFeatures       Left image features.
 * @param rightFeatures      Right image features.
 * @param matches            Matching over the features.
 * @param alignedLeft        Output: aligned left features.
 * @param alignedRight       Output: aligned right features.
 * @param leftBackReference  Output: back reference from aligned index to original index
 * @param rightBackReference Output: back reference from aligned index to original index
 */
    static void GetAlignedPointsFromMatch(const Features &leftFeatures,
                                          const Features &rightFeatures,
                                          const Matching &matches,
                                          Features &alignedLeft,
                                          Features &alignedRight,
                                          std::vector<int> &leftBackReference,
                                          std::vector<int> &rightBackReference);

/**
 * Get the features for left and right images after keeping only the matched features and aligning them.
 * Alignment: i-th feature in left is a match to i-th feature in right.
 * @param leftFeatures  Left image features.
 * @param rightFeatures Right image features.
 * @param matches       Matching over the features.
 * @param alignedLeft   Output: aligned left features.
 * @param alignedRight  Output: aligned right features.
 */
    static void GetAlignedPointsFromMatch(const Features &leftFeatures,
                                          const Features &rightFeatures,
                                          const Matching &matches,
                                          Features &alignedLeft,
                                          Features &alignedRight);

/**
 * Get a Matching for an aligned set: i -> i
 * @param size size of maching vector
 * @return aligned matching.
 */
    //Matching GetAlignedMatching(size_t size);

    static Matching matchFeatures(const Features &featuresLeft, const Features &featuresRight);

    MatchMatrix createFeatureMatchMatrix(int imgCount, FeaturesVect mFeaturesVect,ImgsVect aImgsVect);


};


#endif //TRIANGULATIONFROM2IMGS_MATCHINGUTILS_H
