//
// Created by nannan on 2019/11/15.
//

#ifndef TRIANGULATIONFROM2IMGS_MYFILEUTILS_H
#define TRIANGULATIONFROM2IMGS_MYFILEUTILS_H

#include <string>
#include <boost/filesystem.hpp>
#include <iostream>
#include <boost/algorithm/string.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <map>
#include <chrono>
#include "MyCommon.h"

class MyFileUtils {

public:
    void makeMyDirs();

/**
    * Set the directory with images to perform the SfM operation on.
    * Image file with extensions "jpg" and "png" will be used.
    * @return true on success.
    */
ImgsVect setImagesDirectory(const string &directoryPath);
};


#endif //TRIANGULATIONFROM2IMGS_MYFILEUTILS_H
