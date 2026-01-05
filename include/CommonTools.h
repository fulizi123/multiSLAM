

#ifndef COMMONTOOLS_H
#define COMMONTOOLS_H

//#include "CommonTools.h"
//#include "Frame.h"
//#include "KeyFrame.h"
//#include "LocalMapping.h"
//#include "Map.h"
//#include "MapPoint.h"
//#include "Optimizer.h"
//#include "FeatureExtractor.h"
//#include "System.h"
//#include "Tracking.h"
//#include "Viewer.h"

#include <mutex>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string>
#include <thread>
#include <vector>
#include <list>
#include <map>
#include <unordered_map>
#include <set>
#include <unordered_set>
#include <string>
#include <iostream>
#include <chrono>
#include <ctime>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <Eigen/Dense>
#include "Thirdparty/Sophus/sophus/geometry.hpp"
#include "Thirdparty/Sophus/sophus/sim3.hpp"

using namespace std;

namespace LL_SLAM
{

    class CommonTools {
    public:
        static Eigen::Matrix<float,4,4> toMatrix4f(const cv::Mat &cvMat4);

        static cv::Mat toCvMat(const Eigen::Matrix<float,4,4> &m);
        static cv::Mat toCvMat(const Eigen::Matrix<float,3,3> &m);
        static cv::Mat toCvMat(const Eigen::Matrix<float,3,1> &m);


        static Eigen::Matrix<float,3,3> toSkewMatrix(const Eigen::Vector3f &t) ;

        static Eigen::Matrix<float,4,4> Rt2T(const Eigen::Matrix<float, 3, 3> R, const Eigen::Matrix<float, 3, 1> t);
        static Eigen::Matrix<float,4,4> P2T(const Eigen::Matrix<float, 3, 4> P);
        static Eigen::Matrix<float,3,3> T2R(const Eigen::Matrix<float, 4, 4> T);
        static Eigen::Matrix<float,3,1> T2t(const Eigen::Matrix<float, 4, 4> T);
        static Eigen::Matrix<float,3,4> T2P(const Eigen::Matrix<float, 4, 4> T);


        static bool Triangulate(Eigen::Vector3f &x_c1, Eigen::Vector3f &x_c2,
                         Eigen::Matrix<float,4,4> &Tc1w ,Eigen::Matrix<float,4,4> &Tc2w ,
                         Eigen::Vector3f &x3D);

    };
    
}


#endif // COMMONTOOLS_H
	

