

#ifndef TRACKING_H
#define TRACKING_H

#include "CommonTools.h"
#include "System.h"
#include "FeatureExtractor.h"
#include "FeatureMatcher.h"
#include "Frame.h"
#include "KeyFrame.h"
#include "Map.h"
#include "LocalMapping.h"
#include "Viewer.h"
#include "Optimizer.h"
#include "ObjectObservation.h"
#include "CarlaTopologyStatus.h"
namespace LL_SLAM
{
    class System;
    class Frame;
    class KeyFrame;
    class FeatureExtractor;
    class FeatureMatcher;
    class Map;
    class LocalMapping;
    class Viewer;
    class Optimizer;

    class Tracking{
    public:
        Tracking(System *pSystem);

        Eigen::Matrix4f GrabImageMultiCamera(const vector<cv::Mat> &vImCams, const double &timestamp,
                                             vector<vector<vector<int>>> * pvKeyPoints , vector<vector<vector<float>>> * pvDescriptor,
                                             vector<ObjectObservation> * pvObjectObservations,
                                             CarlaTopologyStatus * pCarlaTopologyStatus);

        void Track();

        void MultiCameraInitialization();

        bool TrackWithMotionModel();
        bool TrackReferenceKeyFrame();

        bool TrackLocalMap();

        void Visualization(const vector<cv::Mat> &vImCams);

        bool NeedNewKeyFrame();

        void CreateNewKeyFrame();

        void ReleaseResource(Frame *pF);

        System* mpSystem;
        vector<FeatureExtractor*> mvpFeatureExtractor;
        Frame *mpCurrentFrame;
        Frame *pPreFrame;
        FeatureMatcher *mpMatcher ;
        Map *mpMap;
        LocalMapping *mpLocalMapper;
        Viewer* mpViewer;
        KeyFrame *mpReferenceKF;

        int mNumCam = 0;
        vector<Eigen::Matrix4f> mvTbc_cams;
        vector<Eigen::Matrix3f> mvK_cams;
        cv::FileStorage mSettings;


        bool bUseDesc = false;
        int mnFeatures;
        float mfScaleFactor;
        int mnLevels;
        int mfIniThFAST;
        int mfMinThFAST;


        enum eTrackingState{
            SYSTEM_NOT_READY=-1,
            NO_IMAGES_YET=0,
            NOT_INITIALIZED=1,
            OK=2,
            RECENTLY_LOST=3,
            LOST=4,
            OK_KLT=5
        };

        eTrackingState mState;
        bool mbCurrentFrameSaveViewerSnapshot = false;

        std::mutex mMutexUpdate;
    };

}

#endif // TRACKING_H
