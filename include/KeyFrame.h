
#ifndef KEYFRAME_H
#define KEYFRAME_H

#include "Frame.h"
#include "System.h"
#include "Map.h"
#include "MapPoint.h"

#include "CommonTools.h"
namespace LL_SLAM
{
    class Frame;
    class System;
    class Map;
    class MapPoint;
    class KeyFrame{
    public:
        KeyFrame(Frame *pFrame);

        void AddObservation(MapPoint *pMP, int cam_i, int KeyPoint_i);
        void EraseObservation(int cam_i, int KeyPoint_i);

        void SetPose(const Eigen::Matrix4f &Tbw) ;
        Eigen::Matrix4f GetPose() const ;
        Eigen::Matrix4f GetTwb() const ;
        Eigen::Matrix3f GetRwb() const ;
        Eigen::Vector3f Gettwb() const ;

        void AddConnection(KeyFrame *pKF, int weight);
        void EraseConnection(KeyFrame *pKF);
        void UpdateConnections();
        vector<KeyFrame*> GetConnectedKeyFrames();
        vector<KeyFrame*> GetBestCovisibilityKeyFrames(int N);
        int GetWeight(KeyFrame *pKF);

        void SetBadFlag();
        bool isBad() const;

        Frame *mpFrame;
        System* mpSystem;

        long unsigned int mnId;
        static long unsigned int nNextId;

        double mTimeStamp;

        int mNumCam = 0;
        vector<Eigen::Matrix4f> mvTbc_cams;
        vector<Eigen::Matrix3f> mvK_cams;
        cv::FileStorage mSettings;

        vector<int> mvNCams;
        vector<vector<cv::KeyPoint>> mvCamKeys;
        vector<vector<cv::KeyPoint>> mvCamKeysUn;
        vector<cv::Mat> mvCamDescriptors;
        vector<vector<cv::Vec3b>> mvColor;
        vector<pair<int, int>> mvWidthHeight;
        vector<vector<MapPoint*>> mvMapPoints;
//        //todo temp
//        vector<vector<int>> mvMatchResult;
//        vector<Eigen::Vector3f> mvMapPointPosition;
        vector<Eigen::Vector3f> mvMapPointPositionVisual;
        //KF Cam KP
        vector<vector<vector<int>>> mvMatchResult;
        vector<Eigen::Vector3f> mvMapPointPosition;
        std::mutex mMutexMatch;


        vector<vector<vector<vector<size_t>>>> mvGridMultiCamera;


        Eigen::Matrix4f mTbw;
        Eigen::Matrix4f mTwb;
        map<KeyFrame*, int> mConnectedKeyFrameWeights;
        vector<KeyFrame*> mvpOrderedConnectedKeyFrames;
        vector<int> mvOrderedWeights;
        mutable std::mutex mMutexConnections;
        bool mbBad = false;
    };

}
#endif // KEYFRAME_H
