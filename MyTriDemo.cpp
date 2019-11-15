//
// Created by nannan on 2019/11/15.
//

#include "MyTriDemo.h"

MyTriDemo::MyTriDemo() {

}

void MyTriDemo::triangleSteps() {

    myFileTool.makeMyDirs();
    myImgs = myFileTool.setImagesDirectory(imageSourcePath);
    myFeatureVects = myFeatureTool.extractMultiImgKeys(myImgs);

    //initialize intrinsics
    mIntrinsics.K = (Mat_<float>(3, 3) << 4137.8, 0, 2128,
            0, 4147.3, 1416,
            0, 0, 1);

//    mIntrinsics.K = (Mat_<float>(3, 3) << 2500, 0, myImgs[0].cols / 2,
//            0, 2500, myImgs[0].rows / 2,
//            0, 0, 1);
    mIntrinsics.Kinv = mIntrinsics.K.inv();
    mIntrinsics.distortion = Mat_<float>::zeros(1, 4);

    mCameraPoses.resize(myImgs.size());

    //Create a matching matrix between all images' features
    myMatchMatrix = myMatchTool.createFeatureMatchMatrix(myImgs.size(), myFeatureVects, myImgs);
    cout << "The pipleline has been set successfully, the features have been extracted and match matrix is created."
         << endl;

    Matx34f Pleft = Matx34f::eye();
    Matx34f Pright = Matx34f::eye();

//
//    size_t i = imagePair.second.left;
//    size_t j = imagePair.second.right;

    Matching prunedMatching;
    //recover camera matrices (poses) from the point matching
    myStereoTool.findCameraMatricesFromMatch(
            mIntrinsics,
            myMatchMatrix[0][1],
            myFeatureVects[0],
            myFeatureVects[1],
            prunedMatching,
            Pleft, Pright
    );
    myMatchMatrix[0][1] = prunedMatching;

    //triangulate the matching points
    PointCloud pointCloud;

    myStereoTool.triangulateViews(
            mIntrinsics,
            {0,1},
            myMatchMatrix[0][1],
            myFeatureVects[0],
            myFeatureVects[1],
            mCameraPoses[0],
            mCameraPoses[1],
            pointCloud
    );

}
