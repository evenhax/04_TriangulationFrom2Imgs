//
// Created by nannan on 2019/11/15.
//

#ifndef TRIANGULATIONFROM2IMGS_FEATUREUTILS_H
#define TRIANGULATIONFROM2IMGS_FEATUREUTILS_H

#include <opencv2/features2d.hpp>
#include "MyCommon.h"

class FeatureUtils {

public:
    FeatureUtils();

    Features extractSingleImgKeys(const cv::Mat &image, size_t ImgNo);

    FeaturesVect extractMultiImgKeys(ImgsVect aImgsVect);

//    std::vector<Features> returnMImageFeatures();
private:
    cv::Ptr<cv::Feature2D> mDetector;
};


#endif //TRIANGULATIONFROM2IMGS_FEATUREUTILS_H
