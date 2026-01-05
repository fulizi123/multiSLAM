
#include "KeyFrame.h"


namespace LL_SLAM
{
    long unsigned int KeyFrame::nNextId=0;

    KeyFrame::KeyFrame(Frame *pFrame) {

        mpFrame = pFrame;

        mTimeStamp = pFrame->mTimeStamp;

        mpSystem = pFrame->mpSystem;
        mNumCam = pFrame->mpSystem->mNumCam;
        mvTbc_cams = pFrame->mpSystem->mvTbc_cams;
        mvK_cams = pFrame->mpSystem->mvK_cams;
        mSettings = pFrame->mpSystem->mSettings;


        mvNCams = pFrame->mvNCams;
//        mvCamKeys = pFrame->mvCamKeys;
        mvCamKeysUn = pFrame->mvCamKeysUn;
        mvCamDescriptors = pFrame->mvCamDescriptors;
        mvColor = pFrame->mvColor;
        mvMapPoints = pFrame->mvMapPoints;
        mvWidthHeight = pFrame->mvWidthHeight;

        mvGridMultiCamera = pFrame->mvGridMultiCamera;


        mvMapPoints = pFrame->mvMapPoints;

        mTbw = pFrame->mTbw;
        mTwb = pFrame->mTwb;

////        Eigen::Matrix4f Twb = pFrame->mTwb;
////        Eigen::Matrix4f Tbw = pFrame->mTbw;
//        mTbw << pFrame->mTbw(0,0), pFrame->mTbw(0,1), pFrame->mTbw(0,2), pFrame->mTbw(0,3),
//                pFrame->mTbw(1,0), pFrame->mTbw(1,1), pFrame->mTbw(1,2), pFrame->mTbw(1,3),
//                pFrame->mTbw(2,0), pFrame->mTbw(2,1), pFrame->mTbw(2,2), pFrame->mTbw(2,3),
//                pFrame->mTbw(3,0), pFrame->mTbw(3,1), pFrame->mTbw(3,2), pFrame->mTbw(3,3) ;
//        mTwb << pFrame->mTwb(0,0), pFrame->mTwb(0,1), pFrame->mTwb(0,2), pFrame->mTwb(0,3),
//                pFrame->mTwb(1,0), pFrame->mTwb(1,1), pFrame->mTwb(1,2), pFrame->mTwb(1,3),
//                pFrame->mTwb(2,0), pFrame->mTwb(2,1), pFrame->mTwb(2,2), pFrame->mTwb(2,3),
//                pFrame->mTwb(3,0), pFrame->mTwb(3,1), pFrame->mTwb(3,2), pFrame->mTwb(3,3) ;
//
////        mTbw = Eigen::Matrix4f(pFrame->mTbw);
////        mTwb = Eigen::Matrix4f(pFrame->mTwb);
////cout << pFrame->mTbw << endl;


        mnId=nNextId++;

//        mpFrame->ComputeMultiCameraMatches();

    }

    void KeyFrame::AddObservation(MapPoint *pMP, int cam_i, int KeyPoint_i) {
        mvMapPoints[cam_i][KeyPoint_i] = pMP;
    }


    void KeyFrame::SetPose(const Eigen::Matrix4f &Tbw) {

        mTbw << Tbw(0,0), Tbw(0,1), Tbw(0,2), Tbw(0,3),
                Tbw(1,0), Tbw(1,1), Tbw(1,2), Tbw(1,3),
                Tbw(2,0), Tbw(2,1), Tbw(2,2), Tbw(2,3),
                Tbw(3,0), Tbw(3,1), Tbw(3,2), Tbw(3,3) ;

        Eigen::Matrix4f Twb = Tbw.inverse();
        mTwb << Twb(0,0), Twb(0,1), Twb(0,2), Twb(0,3),
                Twb(1,0), Twb(1,1), Twb(1,2), Twb(1,3),
                Twb(2,0), Twb(2,1), Twb(2,2), Twb(2,3),
                Twb(3,0), Twb(3,1), Twb(3,2), Twb(3,3) ;
        return ;
    }

    Eigen::Matrix4f KeyFrame::GetPose() const {
        return mTbw;
    }
    Eigen::Matrix4f KeyFrame::GetTwb() const {
        return mTwb;
    }
    Eigen::Matrix3f KeyFrame::GetRwb() const {
        return CommonTools::T2R(mTwb);
    }
    Eigen::Vector3f KeyFrame::Gettwb() const {
        return CommonTools::T2t(mTwb);
    }
} //namespace ORB_SLAM
