
#include "System.h"


namespace LL_SLAM
{
    System::System(const cv::FileStorage Settings, int NumCam, vector<Eigen::Matrix4f> Tbc_cams, vector<Eigen::Matrix3f> K_cams)
    {


        mNumCam = NumCam;
        mvTbc_cams = Tbc_cams;
        mvK_cams = K_cams;
        mSettings = Settings;

        mpMap = new Map(this);
        mpViewer = new Viewer(this);
        mpTracker = new Tracking(this);
        mpLocalMapper = new LocalMapping(this);

        //connection
        mpTracker->mpMap = mpMap;
        mpTracker->mpViewer = mpViewer;
        mpTracker->mpLocalMapper = mpLocalMapper;


        mptViewer = new thread(&Viewer::Run, mpViewer);
        mptLocalMapping = new thread(&LL_SLAM::LocalMapping::Run,mpLocalMapper);


//
//        mpLocalMapper->mpMap = mpMap;
//        mpLocalMapper->mpViewer = mpViewer;
//        mpLocalMapper->mpTracker = mpTracker;


//        mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
//                                 mpAtlas, mpKeyFrameDatabase, strSettingsFile, mSensor, settings_, strSequence, NumCam, Tbc_cams, K_cams);
//
//        //Initialize the Local Mapping thread and launch
//        mpLocalMapper = new LocalMapping(this, mpAtlas, mSensor==MONOCULAR || mSensor==IMU_MONOCULAR,
//                                         mSensor==IMU_MONOCULAR || mSensor==IMU_STEREO || mSensor==IMU_RGBD, strSequence, NumCam, Tbc_cams, K_cams);
//        mptLocalMapping = new thread(&LL_SLAM::LocalMapping::Run,mpLocalMapper);


    }



    Eigen::Matrix4f System::TrackMultiCamera(const vector<cv::Mat> &vImCams, const double &timestamp,
                                             vector<vector<vector<int>>> * pvKeyPoints, vector<vector<vector<float>>> * pvDescriptor)
    {
        if ((*pvKeyPoints).size() == mNumCam) {
            mbUseDesc = true;
        }
//        vector<cv::Mat> vImCamsToFeed(vImCams.size());
//        for (int imCami = 0; imCami < vImCams.size(); imCami++) {
//            vImCamsToFeed[imCami] = vImCams[imCami].clone();
//        }
//
        Eigen::Matrix4f Tcw = mpTracker->GrabImageMultiCamera(vImCams,timestamp, pvKeyPoints, pvDescriptor);


        return Tcw;
    }

    void System::Shutdown() {
        if (mpViewer != nullptr) {
            mpViewer->RequestFinish();
        }
        if (mptViewer != nullptr) {
            mptViewer->join();
            delete mptViewer;
            mptViewer = nullptr;
        }
        if (mpViewer != nullptr) {
            delete mpViewer;
            mpViewer = nullptr;
        }
    }

} //namespace ORB_SLAM
