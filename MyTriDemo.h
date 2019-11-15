//
// Created by nannan on 2019/11/15.
//

#ifndef TRIANGULATIONFROM2IMGS_MYTRIDEMO_H
#define TRIANGULATIONFROM2IMGS_MYTRIDEMO_H
#include "StereoUtils.h"
#include "MatchingUtils.h"
#include "FeatureUtils.h"
#include "MyFileUtils.h"

class MyTriDemo {

public:
    MyTriDemo();
    void triangleSteps();
private:

    MyFileUtils myFileTool;
    FeatureUtils myFeatureTool;
    MatchingUtils myMatchTool;
    StereoUtils myStereoTool;

    ImgsVect myImgs;
    FeaturesVect myFeatureVects;
    MatchMatrix myMatchMatrix;
    Intrinsics mIntrinsics;
    std::vector<cv::Matx34f> mCameraPoses;



};


#endif //TRIANGULATIONFROM2IMGS_MYTRIDEMO_H
