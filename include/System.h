

#ifndef SYSTEM_H
#define SYSTEM_H


#include "CommonTools.h"
#include "Tracking.h"
#include "LocalMapping.h"
#include "Map.h"
#include "Viewer.h"
#include "ObjectObservation.h"
#include "CarlaTopologyStatus.h"

namespace LL_SLAM
{
    class Viewer;
    class Tracking;
    class LocalMapping;
    class Map;

    class System{
    public:
        System(const cv::FileStorage Settings,
                      int NumCam = 0, vector<Eigen::Matrix4f> Tbc_cams = {}, vector<Eigen::Matrix3f> K_cams = {}    );

        void Shutdown();

        Eigen::Matrix4f TrackMultiCamera(const vector<cv::Mat> &vImCams, const double &timestamp,
                                         vector<vector<vector<int>>> * pvKeyPoints, vector<vector<vector<float>>> * pvDescriptor,
                                         vector<ObjectObservation> * pvObjectObservations = nullptr,
                                         CarlaTopologyStatus * pCarlaTopologyStatus = nullptr);

        Tracking* mpTracker;
        LocalMapping* mpLocalMapper;
        Viewer* mpViewer;
        Map* mpMap;

        std::thread* mptLocalMapping;
        std::thread* mptViewer;


        //MultiCam
        int mNumCam = 0;
        vector<Eigen::Matrix4f> mvTbc_cams;
        vector<Eigen::Matrix3f> mvK_cams;
        cv::FileStorage mSettings;
        std::mutex mMutexUpdate;

        bool mbUseDesc = false;
        bool mbUseTime = true;

    };

}

#endif // SYSTEM_H
