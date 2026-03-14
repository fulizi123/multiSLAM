

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
        enum Mp4LayoutMode {
            MP4_LAYOUT_SINGLE_INSET = 0,
            MP4_LAYOUT_SURROUND_8 = 1
        };

        Viewer(System *pSystem);
        ~Viewer();

        void Run();
        void RequestFinish();

        void Visualization(const vector<cv::Mat> &vImCams, Frame *pCurrentFrame) ;

        void VisualizationXZ(const vector<cv::Mat> &vImCams, Frame *pCurrentFrame) ;
        void VisualizationXY(const vector<cv::Mat> &vImCams, Frame *pCurrentFrame) ;
        void VisualizationYZ(const vector<cv::Mat> &vImCams, Frame *pCurrentFrame) ;


        void InsertFrame(const vector<cv::Mat> &vImCams, Frame *pCurrentFrame) ;

        vector<vector<cv::Mat>> mvvImCams;
        vector<Frame *> mvpFrame;
        std::mutex mMutexMsg;
        bool mbFinishRequested = false;

        vector<Eigen::Matrix4f> Twbs;

        System *mpSystem;
        cv::VideoWriter mVideoWriter;
        Mp4LayoutMode mMp4LayoutMode = MP4_LAYOUT_SURROUND_8;
        int mSingleWidth = 1000;
        int mSingleHeight = 720;
        int mSurroundWidth = 2400;
        int mSurroundHeight = 1300;
        int mWidth = 1000;
        int mHeight = 720;
    };

}

#endif // VIEWER_H
	
