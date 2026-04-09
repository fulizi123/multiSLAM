#ifndef FRAME_H
#define FRAME_H

#include "CommonTools.h"
#include "System.h"
#include "MapPoint.h"
#include "FeatureExtractor.h"
#include "FeatureMatcher.h"
#include "ObjectObservation.h"
#include "CarlaTopologyStatus.h"
namespace LL_SLAM
{
//    #define FRAME_GRID_ROWS 48
//    #define FRAME_GRID_COLS 64

    class System;
    class MapObject;
    class MapPoint;
    class FeatureExtractor;
    class FeatureMatcher;


    class Frame{
    public:

        Frame(const vector<cv::Mat> &vImColorCams, const vector<cv::Mat> &vImCams,  vector<vector<vector<int>>> * pvKeyPoints,  vector<vector<vector<float>>> * pvDescriptor,
              vector<ObjectObservation> * pvObjectObservations, CarlaTopologyStatus * pCarlaTopologyStatus, const double &timeStamp,
              vector<FeatureExtractor*> vextractors, System *pSystem);

        void ExtractORBMultiCamera(int Camid, const cv::Mat &im, FeatureExtractor* extractor);
        //vector<vector<vector<int>>> *vKeyPoints, vector<vector<vector<float>>> *vDescriptor is for copy , 50ms - > 1ms
        void ExtractORBMultiCameraSuperPoint(int Camid, const cv::Mat &im, FeatureExtractor* extractor, vector<vector<vector<int>>> * vKeyPoints, vector<vector<vector<float>>> * vDescriptor);

        void ComputeMultiCameraMatches();

        void AssignFeaturesToGridMultiCamera(const vector<cv::Mat> &vImCams);

        void UndistortKeyPointsMultiCamera();

        vector<int> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int  &imCami, const int minLevel=-1, const int maxLevel=-1) const;




        void SetPose(const Eigen::Matrix4f &Tbw) ;
        Eigen::Matrix4f GetPose() const ;
        Eigen::Matrix4f GetTwb() const ;
        Eigen::Matrix3f GetRwb() const ;
        Eigen::Vector3f Gettwb() const ;

        void SetVelocity(const Eigen::Matrix4f &VelocityTwb) ;
        Eigen::Matrix4f  PredictPose() const;

        static int FRAME_GRID_ROWS ;
        static int FRAME_GRID_COLS ;
        //time
        double mTimeStamp;

        //Imput calib
        int mNumCam = 0;
        vector<Eigen::Matrix4f> mvTbc_cams;
        vector<Eigen::Matrix3f> mvK_cams;
        cv::FileStorage mSettings;
        System* mpSystem;
        Frame *pPreFrame;



        // Current and Next Frame id.
        static long unsigned int nNextId;
        long unsigned int mnId;
        cv::Mat mDistCoef;
        bool mbUseDesc = false;
        bool mbCanBeRelease = true;

        //ORB
        vector<FeatureExtractor*> mvpFeatureExtractors;
        vector<int> mvNCams;
        vector<vector<cv::KeyPoint>> mvCamKeys;
        vector<vector<cv::KeyPoint>> mvCamKeysUn;
        vector<cv::Mat> mvCamDescriptors;
        vector<vector<cv::Vec3b>> mvColor;
        vector<vector<float>> mvScaleFactors;
        vector<vector<vector<int>>> mvSuperPointKeyPoints;
        vector<vector<vector<float>>> mvSuperPointDescriptor;
        vector<pair<int, int>> mvWidthHeight;
        vector<ObjectObservation> mvObjectObservations;
        vector<MapObject*> mvMapObjects;
        CarlaTopologyStatus mCarlaTopologyStatus;

        //cam_i, iL : mapid
        vector<vector<int>> mvMatchResult;
        //For quickly init map points
        vector<Eigen::Vector3f> mvMapPointPosition;
        vector<vector<MapPoint*>> mvMapPoints;

        vector<vector<vector<vector<size_t>>>> mvGridMultiCamera;

//        mvpMapPoints

        //Geometry
        Eigen::Matrix4f mTbw;
        Eigen::Matrix4f mTwb;
        //T_wblast_wbnow
        Eigen::Matrix4f mVelocityTwb;


    };

}

#endif // FRAME_H
