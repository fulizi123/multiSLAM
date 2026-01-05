

#ifndef VIEWER_H
#define VIEWER_H

#include "CommonTools.h"
#include "System.h"
#include "Frame.h"
#include "KeyFrame.h"
#include "MapPoint.h"


namespace LL_SLAM
{

    class System;
    class Frame;
    class KeyFrame;
    class MapPoint;
    class Viewer{
    public:
        Viewer(System *pSystem);

        void Run();

        void Visualization(const vector<cv::Mat> &vImCams, Frame *pCurrentFrame) ;

        void VisualizationXZ(const vector<cv::Mat> &vImCams, Frame *pCurrentFrame) ;
        void VisualizationXY(const vector<cv::Mat> &vImCams, Frame *pCurrentFrame) ;
        void VisualizationYZ(const vector<cv::Mat> &vImCams, Frame *pCurrentFrame) ;


        void InsertFrame(const vector<cv::Mat> &vImCams, Frame *pCurrentFrame) ;

        vector<vector<cv::Mat>> mvvImCams;
        vector<Frame *> mvpFrame;
        std::mutex mMutexMsg;

        vector<Eigen::Matrix4f> Twbs;

        System *mpSystem;
    };

}

#endif // VIEWER_H
	

