
#include "Tracking.h"

using namespace std;

namespace LL_SLAM
{
    Tracking::Tracking(System *pSystem) {

        mpSystem = pSystem;
        mpMap = pSystem->mpMap;

//        cout << "pSystem->mpMap->mvpMPObservations.size() " << pSystem->mpMap->mvpMPObservations.size() << endl;
//        cout << "mpMap->mvpMPObservations.size() " << mpMap->mvpMPObservations.size() << endl;

        mNumCam = pSystem->mNumCam;
        mvTbc_cams = pSystem->mvTbc_cams;
        mvK_cams = pSystem->mvK_cams;
        mSettings = pSystem->mSettings;

        mnFeatures = mSettings["FeatureExtractor.nFeatures"];
        mfScaleFactor = mSettings["FeatureExtractor.scaleFactor"];
        mnLevels = mSettings["FeatureExtractor.nLevels"];
        mfIniThFAST = mSettings["FeatureExtractor.iniThFAST"];
        mfMinThFAST = mSettings["FeatureExtractor.minThFAST"];

        mvpFeatureExtractor.resize(mNumCam);
        for (int imCami = 0; imCami < mNumCam; imCami++) {
            mvpFeatureExtractor[imCami] = new FeatureExtractor(mnFeatures,mfScaleFactor,mnLevels,mfIniThFAST,mfMinThFAST);
        }

        mpMatcher = new FeatureMatcher(0.6,true);

        pPreFrame = NULL;
        mpCurrentFrame = NULL;

        mState = NO_IMAGES_YET;

    }

    Eigen::Matrix4f Tracking::GrabImageMultiCamera(const vector<cv::Mat> &vImCams, const double &timestamp,
                                                   vector<vector<vector<int>>> * pvKeyPoints , vector<vector<vector<float>>> * pvDescriptor)
    {

        vector<cv::Mat> vImGrayCams(vImCams.size());
        for (int imCami = 0; imCami < vImCams.size(); imCami++) {
            vImGrayCams[imCami] = vImCams[imCami];
            cvtColor(vImGrayCams[imCami],vImGrayCams[imCami],cv::COLOR_RGB2GRAY);
        }

        pPreFrame = mpCurrentFrame;

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        /////////////////
        mpCurrentFrame = new Frame(vImCams, vImGrayCams,pvKeyPoints,pvDescriptor,timestamp,mvpFeatureExtractor,mpSystem);
        /////////////////
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        cout << "Frame use time : " << std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count() * 1000.0 << " ms." << endl;

        Track();


        Visualization(vImCams);


        ReleaseResource(mpCurrentFrame);


        return mpCurrentFrame->GetTwb();
        //return Eigen::Matrix4f::Identity();
    }

    void Tracking::Track() {
        unique_lock<mutex> lock1(mpMap->mMutexUpdate);
        unique_lock<mutex> lock2(mpSystem->mMutexUpdate);
        unique_lock<mutex> lock3(mMutexUpdate);

        if (mState == NO_IMAGES_YET) {
            mState = NOT_INITIALIZED;
        }

        if(mState==NOT_INITIALIZED)
        {
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
            /////////////////
            MultiCameraInitialization();
            /////////////////
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
            cout << "MultiCameraInitialization use time : " << std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count() * 1000.0 << " ms." << endl;
            mState = OK;
            return ;
        }

        //normal tracking
        bool bOK = false;
        bOK = TrackWithMotionModel();
//        if(!bOK) {
//            bOK = TrackReferenceKeyFrame();
//        }

//        if (bOK) {
//            //bOK = TrackLocalMap();
//        }

        if (bOK) {
            //success update volocity
            Eigen::Matrix4f T_w_bl = mpCurrentFrame->pPreFrame->GetPose().inverse();
            Eigen::Matrix4f T_w_bn = mpCurrentFrame->GetPose().inverse();
            Eigen::Matrix4f T_bl_bn = T_w_bl.inverse() * T_w_bn;

            mpCurrentFrame->SetVelocity(T_bl_bn);


        }

        if (NeedNewKeyFrame()) {
        //    CreateNewKeyFrame();
            mpLocalMapper->InsertFrame(mpCurrentFrame);
            mpCurrentFrame->mbCanBeRelease = false;
            //usleep(400 * 1000);
        }

        return ;

    }

    void Tracking::MultiCameraInitialization() {

        Eigen::Matrix4f TbwInit = Eigen::Matrix4f::Identity();
        Eigen::Matrix4f TbwVelocityInit = Eigen::Matrix4f::Identity();
        mpCurrentFrame->SetPose(TbwInit);
        mpCurrentFrame->SetVelocity(TbwVelocityInit);
        mpCurrentFrame->ComputeMultiCameraMatches();

        KeyFrame* pKF = new KeyFrame(mpCurrentFrame);

        mpMap->AddKeyFrame(pKF);

        //MapPoint
        vector<vector<int>> &vMatchResult = mpCurrentFrame->mvMatchResult;
        vector<Eigen::Vector3f> &vMapPointPosition = mpCurrentFrame->mvMapPointPosition;
        vector<MapPoint*> vMapPoints(vMapPointPosition.size(), NULL);
        for (int i = 0; i < vMapPointPosition.size(); i++) {
            vMapPoints[i] = new MapPoint(vMapPointPosition[i]);
            mpMap->AddMapPoint(vMapPoints[i]);
        }

        for (int cam_i = 0; cam_i < mNumCam; cam_i++) {
            for (int KeyPoint_i = 0; KeyPoint_i < mpCurrentFrame->mvNCams[cam_i]; KeyPoint_i++) {
                if (vMatchResult[cam_i][KeyPoint_i] >= 0) {
                    //there is a init mappoint
                    MapPoint* pMP = vMapPoints[vMatchResult[cam_i][KeyPoint_i]];

                    pMP->AddObservation(pKF, cam_i, KeyPoint_i);
                    pKF->AddObservation(pMP, cam_i, KeyPoint_i);

                }
            }
        }


        //mpReferenceKF
        mpReferenceKF = pKF;

        mpMap->UpdateLocalMap();

        return ;

    }

    bool Tracking::TrackWithMotionModel() {
        FeatureMatcher matcher(0.8,true);
//
        set<MapPoint*> spLocalMP;
        for (int i = 0; i < mpReferenceKF->mvMapPoints.size(); i++) {
            for (int j = 0; j < mpReferenceKF->mvMapPoints[i].size(); j++) {
                if (mpReferenceKF->mvMapPoints[i][j] == NULL) {continue;}
                //sometimes the map point are newed but never add to map.
                if (mpReferenceKF->mvMapPoints[i][j]->mDescriptor.empty()) {continue;}
                spLocalMP.insert(mpReferenceKF->mvMapPoints[i][j]);
            }
        }

        vector<MapPoint*> vpLocalMP;
        vpLocalMP.reserve(spLocalMP.size());
        for (auto it : spLocalMP) {
            vpLocalMP.push_back(it);
        }
//        vector<MapPoint*> vpLocalMP = mpMap->GetLocalMapPoint();

        std::vector<std::vector<MapPoint*>> vpMapPointMatches;

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        /////////////////
//        int nMatches = matcher.SearchByProject(vpLocalMP, mpCurrentFrame);
//        int nMatches = matcher.SearchByProjectMultiCamera(vpLocalMP, mpCurrentFrame);
        int nMatches = matcher.SearchByProjectMultiCamera(vpLocalMP, mpCurrentFrame, 500);
        /////////////////
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        cout << "SearchByProject use time : " << std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count() * 1000.0 << " ms." << endl;

        mpCurrentFrame->SetPose(mpCurrentFrame->PredictPose());

        std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
        /////////////////
        int nInLiner = Optimizer::PoseOptimization(mpCurrentFrame);
        /////////////////
        std::chrono::steady_clock::time_point t4 = std::chrono::steady_clock::now();
        cout << "Optimizer use time : " << std::chrono::duration_cast<std::chrono::duration<double> >(t4 - t3).count() * 1000.0 << " ms." << endl;
//
//        cout << "Debug Tracking nInLiner " << nInLiner << endl;

        if (nInLiner >= 15) {
            //success
            return true;
        }
        //cv:: waitKey(0);
        return false;
    }

    bool Tracking::TrackReferenceKeyFrame() {
        return false;

        FeatureMatcher matcher(0.8,true);
        //not use
        std::vector<std::vector<MapPoint*>> vpMapPointMatches;
        int nMatches = matcher.SearchByReferenceKFBruteForce(mpReferenceKF, mpCurrentFrame);

        int nInLiner = Optimizer::PoseOptimization(mpCurrentFrame);

        //cv:: waitKey(0);
        return false;
    }

    bool Tracking::TrackLocalMap() {
        FeatureMatcher matcher(0.8,true);
//        vector<MapPoint*> vpLocalMP = mpMap->mvpLocalMP;
        vector<MapPoint*> vpLocalMP = mpMap->GetLocalMapPoint();

        //not use
        std::vector<std::vector<MapPoint*>> vpMapPointMatches;
        int nMatches = matcher.SearchByProject(vpLocalMP, mpCurrentFrame);

        int nInLiner = Optimizer::PoseOptimization(mpCurrentFrame);

        //cv:: waitKey(0);
        return false;
    }


    void Tracking::Visualization(const vector<cv::Mat> &vImCams) {
        mpViewer->InsertFrame(vImCams, mpCurrentFrame);
//        mpViewer->Visualization(vImCams, mpCurrentFrame);

        return;

    }


    bool Tracking::NeedNewKeyFrame() {
        //time
//        if ((mpCurrentFrame->mTimeStamp - mpReferenceKF->mTimeStamp)>=3.0) {
//            return true;
//        }
        if (mpCurrentFrame->mnId % 30 == 0) {
            return true;
        }


        return false;

    }





    void Tracking::CreateNewKeyFrame()
    {
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        /////////////////

        std::chrono::steady_clock::time_point t5 = std::chrono::steady_clock::now();
        /////////////////

        mpCurrentFrame->ComputeMultiCameraMatches();

        /////////////////
        std::chrono::steady_clock::time_point t6 = std::chrono::steady_clock::now();
        cout << "ComputeMultiCameraMatches use time : " << std::chrono::duration_cast<std::chrono::duration<double> >(t6 - t5).count() * 1000.0 << " ms." << endl;

        std::chrono::steady_clock::time_point t7 = std::chrono::steady_clock::now();
        /////////////////

        //track local map but not opt
        FeatureMatcher matcher(0.8,true);
//        vector<MapPoint*> vpLocalMP = mpMap->mvpLocalMP;
        vector<MapPoint*> vpLocalMP = mpMap->GetLocalMapPoint();
//        int nMatches = matcher.SearchByProject(vpLocalMP, mpCurrentFrame);
        matcher.SearchByProjectMultiCamera(vpLocalMP, mpCurrentFrame);

        /////////////////
        std::chrono::steady_clock::time_point t8 = std::chrono::steady_clock::now();
        cout << "SearchByProject Local Map use time : " << std::chrono::duration_cast<std::chrono::duration<double> >(t8 - t7).count() * 1000.0 << " ms." << endl;

        KeyFrame* pKF = new KeyFrame(mpCurrentFrame);
        mpMap->AddKeyFrame(pKF);


        // step 1 : init
//        vector<KeyFrame*> vpLocalKF = mpMap->mvpLocalKF;
        vector<KeyFrame*> vpLocalKF = mpMap->GetLocalKeyFrame();
        while (vpLocalKF.size() > 3) {vpLocalKF.pop_back();}
        // vpLocalKF.pop_back();  //we donot use the last KF, it will be removed
        //reverse(vpLocalKF.begin(), vpLocalKF.end());
        vpLocalKF.insert(vpLocalKF.begin(), pKF);
        int nMinKF = pKF->mnId;
        int nMaxKF = pKF->mnId;
        for (int KF_i = 0; KF_i < vpLocalKF.size(); KF_i++) {
            nMinKF = min(nMinKF, int(vpLocalKF[KF_i]->mnId));
            nMaxKF = max(nMaxKF, int(vpLocalKF[KF_i]->mnId));
        }
        {
            pKF->mvMatchResult = vector<vector<vector<int>>>(nMaxKF - nMinKF + 1);
            pKF->mvMatchResult[nMaxKF - pKF->mnId].resize(pKF->mNumCam);
            for (int cam_i = 0; cam_i < pKF->mNumCam; cam_i++) {
                pKF->mvMatchResult[nMaxKF - pKF->mnId][cam_i].resize(pKF->mvNCams[cam_i], -1);
            }
            for (int KF_i = 0; KF_i < vpLocalKF.size(); KF_i++) {
                pKF->mvMatchResult[nMaxKF - vpLocalKF[KF_i]->mnId].resize(vpLocalKF[KF_i]->mNumCam);
                for (int cam_i = 0; cam_i < vpLocalKF[KF_i]->mNumCam; cam_i++) {
                    pKF->mvMatchResult[nMaxKF - vpLocalKF[KF_i]->mnId][cam_i].resize(vpLocalKF[KF_i]->mvNCams[cam_i], -1);
                }
            }

            Eigen::Matrix4f Twb = mpCurrentFrame->GetTwb();
            Eigen::Matrix3f Rwb = CommonTools::T2R(Twb);
            Eigen::Vector3f twb = CommonTools::T2t(Twb);
            pKF->mvMapPointPosition.reserve((nMaxKF - nMinKF + 1) * mNumCam * mnFeatures);
            pKF->mvMapPointPosition.resize(mpCurrentFrame->mvMapPointPosition.size());
            for (int i = 0; i < mpCurrentFrame->mvMapPointPosition.size(); i++) {
                pKF->mvMapPointPosition[i] = Rwb * mpCurrentFrame->mvMapPointPosition[i] + twb;
            }
        }

        // step 2 : update stereo matches
        for (int cam_i = 0; cam_i < mpCurrentFrame->mvMapPoints.size(); cam_i++) {
            for (int kpi = 0; kpi < mpCurrentFrame->mvMapPoints[cam_i].size(); kpi++) {
                if (mpCurrentFrame->mvMatchResult[cam_i][kpi] >= 0) {
                    pKF->mvMatchResult[nMaxKF - pKF->mnId][cam_i][kpi] = mpCurrentFrame->mvMatchResult[cam_i][kpi];
                }
            }
        }

        std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
        /////////////////


       // step 3 : SearchByEpipolarGeometryBetweenImage
       for (int cam_i = 0; cam_i < pKF->mNumCam; cam_i++) {
           for (int KF_i = 1; KF_i < vpLocalKF.size(); KF_i++) {
               matcher.SearchByEpipolarGeometryBetweenImage(pKF, cam_i, vpLocalKF[KF_i], cam_i, nMinKF, nMaxKF);
           }
       }
        // matcher.SearchByEpipolarGeometryMultiCamera(vpLocalKF, nMinKF, nMaxKF);

        // if (vpLocalKF.size() == 5) {
        //     matcher.SearchByEpipolarGeometryMultiCamera(vpLocalKF, nMinKF, nMaxKF);
        // }
        /////////////////
        std::chrono::steady_clock::time_point t4 = std::chrono::steady_clock::now();
        cout << "SearchByEpipolarGeometryBetweenImage use time : " << std::chrono::duration_cast<std::chrono::duration<double> >(t4 - t3).count() * 1000.0 << " ms." << endl;



        //step 4 : fuse the data
        //[KFi][cam_i][kpi] -> index
        vector< MapPoint * > vMapPoints(pKF->mvMapPointPosition.size(), NULL);
        {
            //step 4.1 : one or more kp is already matched a MapPoint
            //todo matches are not the same
            for (int KF_i = 0; KF_i < vpLocalKF.size(); KF_i++) {
                int KF_index = nMaxKF - vpLocalKF[KF_i]->mnId;
                for (int cam_i = 0; cam_i < vpLocalKF[KF_i]->mNumCam; cam_i++) {
                    for (int kpi = 0; kpi < vpLocalKF[KF_i]->mvNCams[cam_i]; kpi++) {
                        MapPoint *pMP = vpLocalKF[KF_i]->mvMapPoints[cam_i][kpi];
                        if (pMP == NULL) {
                            continue;
                        }
                        if (pKF->mvMatchResult[KF_index][cam_i][kpi] == -1) {
                            continue;
                        }
                        vMapPoints[pKF->mvMatchResult[KF_index][cam_i][kpi]] = pMP;
                    }
                }
            }

            //step 4.2 : Matches those without exist MapPoint , Create one.
            for (int i = 0; i < vMapPoints.size(); i++) {
                //none of its observation is matched with MapPoint
                if (vMapPoints[i] == NULL) {
                    vMapPoints[i] = new MapPoint(pKF->mvMapPointPosition[i]);
                    mpMap->AddMapPoint(vMapPoints[i]);
                }
            }

            //step 4.3 : update mpCurrentFrame->mvMapPoints with vMatchResult
            for (int KF_i = 0; KF_i < vpLocalKF.size(); KF_i++) {
                int KF_index = nMaxKF - vpLocalKF[KF_i]->mnId;
                for (int cam_i = 0; cam_i < vpLocalKF[KF_i]->mNumCam; cam_i++) {
                    for (int kpi = 0; kpi < vpLocalKF[KF_i]->mvNCams[cam_i]; kpi++) {
                        if (vpLocalKF[KF_i]->mvMapPoints[cam_i][kpi] != NULL) {
                            continue;
                        }
                        if (pKF->mvMatchResult[KF_index][cam_i][kpi] == -1) {
                            continue;
                        }

                        int MP_i = pKF->mvMatchResult[KF_index][cam_i][kpi];
                        vpLocalKF[KF_i]->mvMapPoints[cam_i][kpi] = vMapPoints[MP_i];
                    }
                }
            }
        }




        std::chrono::steady_clock::time_point t9 = std::chrono::steady_clock::now();
        /////////////////
        //step 5 :  (so weird, but it works...pKF->AddObservation(pMP, cam_i, kpi) already done)
        //now only Update pKF
        for (int KF_i = 0; KF_i < 1; KF_i++) {
            int KF_index = nMaxKF - vpLocalKF[KF_i]->mnId;
            for (int cam_i = 0; cam_i < vpLocalKF[KF_i]->mNumCam; cam_i++) {
                for (int kpi = 0; kpi < vpLocalKF[KF_i]->mvNCams[cam_i]; kpi++) {
                    if (vpLocalKF[KF_i]->mvMapPoints[cam_i][kpi] == NULL) {
                        continue;
                    }
                    MapPoint* pMP = vpLocalKF[KF_i]->mvMapPoints[cam_i][kpi];

                    pMP->AddObservation(vpLocalKF[KF_i], cam_i, kpi);
                    vpLocalKF[KF_i]->AddObservation(pMP, cam_i, kpi);
                }
            }
        }
        for (int i = 0; i < vMapPoints.size(); i++) {
            //none of its observation is matched with MapPoint
            if (vMapPoints[i] == NULL) {
                continue;
            }
            vMapPoints[i]->UpdateDescriptor();
        }
//        for (int i = 0; i < vMapPoints.size(); i++) {
//            //none of its observation is matched with MapPoint
//            if (vMapPoints[i] == NULL) {
//                continue;
//            }
//            if (vMapPoints[i]->mDescriptor.empty()) {
//
//                vMapPoints[i]->UpdateDescriptor();
//            }
//        }

        /////////////////
        std::chrono::steady_clock::time_point t10 = std::chrono::steady_clock::now();
        cout << "pMP->AddObservation use time : " << std::chrono::duration_cast<std::chrono::duration<double> >(t10 - t9).count() * 1000.0 << " ms." << endl;


        mpReferenceKF = pKF;

        mpMap->UpdateLocalMap();

        //    mpLocalMapper->InsertKeyFrame(pKF);


        /////////////////
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        cout << "CreateNewKeyFrame use time : " << std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count() * 1000.0 << " ms." << endl;
        return ;
    }


    void Tracking::ReleaseResource(Frame *pF) {
        //100 frames : 5.6GB without ALL ReleaseResource
        //100 frames : 2.8GB without Tracking::ReleaseResource
        //100 frames : 300MB

        if (pF->mbCanBeRelease == false) {return;}
        vector<vector<cv::KeyPoint>>().swap( pF->mvCamKeys);
        vector<vector<int>>().swap( pF->mvMatchResult);
        vector<Eigen::Vector3f>().swap( pF->mvMapPointPosition);
        vector<cv::Mat>().swap( pF->mvCamDescriptors);
        vector<vector<vector<vector<size_t>>>>().swap( pF->mvGridMultiCamera);

    }


}






//
//void Tracking::CreateNewKeyFrame()
//{
//    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
//    /////////////////
//
//    std::chrono::steady_clock::time_point t5 = std::chrono::steady_clock::now();
//    /////////////////
//
//    mpCurrentFrame->ComputeMultiCameraMatches();
//
//    /////////////////
//    std::chrono::steady_clock::time_point t6 = std::chrono::steady_clock::now();
//    cout << "ComputeMultiCameraMatches use time : " << std::chrono::duration_cast<std::chrono::duration<double> >(t6 - t5).count() * 1000.0 << " ms." << endl;
//
//    std::chrono::steady_clock::time_point t7 = std::chrono::steady_clock::now();
//    /////////////////
//
//    //track local map but not opt
//    FeatureMatcher matcher(0.8,true);
//    vector<MapPoint*> vpLocalMP = mpMap->mvpLocalMP;
//    int nMatches = matcher.SearchByProject(vpLocalMP, mpCurrentFrame);
//
//    /////////////////
//    std::chrono::steady_clock::time_point t8 = std::chrono::steady_clock::now();
//    cout << "SearchByProject Local Map use time : " << std::chrono::duration_cast<std::chrono::duration<double> >(t8 - t7).count() * 1000.0 << " ms." << endl;
//
//
//    {
//        //Match MP and Create Stereo MapPoint,
////            vector <vector<int>> &vMatchResult = mpCurrentFrame->mvMatchResult; //[cam_i][kpi] -> index
////            vector <Eigen::Vector3f> &vMapPointPosition = mpCurrentFrame->mvMapPointPosition; //
//        vector < MapPoint * > vMapPoints(mpCurrentFrame->mvMapPointPosition.size(), NULL);
//        //step 1 : one or more stereo kp is already matched a MapPoint
//        for (int cam_i = 0; cam_i < mpCurrentFrame->mvMapPoints.size(); cam_i++) {
//            for (int kpi = 0; kpi < mpCurrentFrame->mvMapPoints[cam_i].size(); kpi++) {
//                MapPoint *pMP = mpCurrentFrame->mvMapPoints[cam_i][kpi];
//                if (pMP == NULL) {
//                    continue;
//                }
//                if (mpCurrentFrame->mvMatchResult[cam_i][kpi] == -1) {
//                    continue;
//                }
//                vMapPoints[mpCurrentFrame->mvMatchResult[cam_i][kpi]] = pMP;
//            }
//        }
//
//        //step 2 : stereo those without exist MapPoint , Create one.
//
//        Eigen::Matrix4f Twb = mpCurrentFrame->GetTwb();
//        Eigen::Matrix3f Rwb = CommonTools::T2R(Twb);
//        Eigen::Vector3f twb = CommonTools::T2t(Twb);
//        for (int i = 0; i < mpCurrentFrame->mvMapPointPosition.size(); i++) {
//            //none of its observation is matched with MapPoint
//            if (vMapPoints[i] == NULL) {
//                vMapPoints[i] = new MapPoint(Rwb * mpCurrentFrame->mvMapPointPosition[i] + twb);
//                mpMap->AddMapPoint(vMapPoints[i]);
//            }
//        }
//
//        //step 3 : update mpCurrentFrame->mvMapPoints with vMatchResult
//        for (int cam_i = 0; cam_i < mpCurrentFrame->mvMapPoints.size(); cam_i++) {
//            for (int kpi = 0; kpi < mpCurrentFrame->mvMapPoints[cam_i].size(); kpi++) {
//                if (mpCurrentFrame->mvMapPoints[cam_i][kpi] != NULL) {
//                    continue;
//                }
//                if (mpCurrentFrame->mvMatchResult[cam_i][kpi] == -1) {
//                    continue;
//                }
//
//                mpCurrentFrame->mvMapPoints[cam_i][kpi] = vMapPoints[mpCurrentFrame->mvMatchResult[cam_i][kpi]];
//            }
//        }
//
//    }
//
//    KeyFrame* pKF = new KeyFrame(mpCurrentFrame);
//    mpMap->AddKeyFrame(pKF);
//
//    std::chrono::steady_clock::time_point t9 = std::chrono::steady_clock::now();
//    /////////////////
//    //step 4 :  (so weird, but it works...pKF->AddObservation(pMP, cam_i, kpi) already done)
//    for (int cam_i = 0; cam_i < pKF->mvMapPoints.size(); cam_i++) {
//        for (int kpi = 0; kpi < pKF->mvMapPoints[cam_i].size(); kpi++) {
//            if (pKF->mvMapPoints[cam_i][kpi] == NULL) {
//                continue;
//            }
//
//            MapPoint* pMP = pKF->mvMapPoints[cam_i][kpi];
//
//            pMP->AddObservation(pKF, cam_i, kpi);
//            pKF->AddObservation(pMP, cam_i, kpi);
//        }
//    }
//
//    /////////////////
//    std::chrono::steady_clock::time_point t10 = std::chrono::steady_clock::now();
//    cout << "Match MP step 4 use time : " << std::chrono::duration_cast<std::chrono::duration<double> >(t10 - t9).count() * 1000.0 << " ms." << endl;
//
//    //step 5 :
//
//    std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
//    /////////////////
////      //350~450
////        matcher.SearchByEpipolarGeometryBetweenImage(pKF, 3, mpReferenceKF, 3);
////        matcher.SearchByEpipolarGeometryBetweenImage(pKF, 4, mpReferenceKF, 4);
////        matcher.SearchByEpipolarGeometryBetweenImage(pKF, 5, mpReferenceKF, 5);
////        matcher.SearchByEpipolarGeometryBetweenImage(pKF, 6, mpReferenceKF, 6);
//
////        for (int cam_i = 0; cam_i < pKF->mNumCam; cam_i++) {
////            matcher.SearchByEpipolarGeometryBetweenImage(pKF, cam_i, mpReferenceKF, cam_i);
////        }
////        matcher.SearchByEpipolarGeometryBetweenImage(pKF, 0, pKF, 2);
//
//
//    // step 1 : init
//    vector<KeyFrame*> vpLocalKF = mpMap->mvpLocalKF;
//    vpLocalKF.pop_back();  //we donot use the last KF, it will be removed
//    int nMinKF = pKF->mnId;
//    int nMaxKF = pKF->mnId;
//    for (int KF_i = 0; KF_i < vpLocalKF.size(); KF_i++) {
//        nMinKF = min(nMinKF, int(vpLocalKF[KF_i]->mnId));
//        nMaxKF = max(nMaxKF, int(vpLocalKF[KF_i]->mnId));
//    }
//    {
//        pKF->mvMatchResult = vector<vector<vector<int>>>(nMaxKF - nMinKF + 1);
//        pKF->mvMatchResult[pKF->mnId - nMinKF].resize(pKF->mNumCam);
//        for (int cam_i = 0; cam_i < pKF->mNumCam; cam_i++) {
//            pKF->mvMatchResult[pKF->mnId - nMinKF][cam_i].resize(pKF->mvNCams[cam_i]);
//        }
//        for (int KF_i = 0; KF_i < vpLocalKF.size(); KF_i++) {
//            pKF->mvMatchResult[vpLocalKF[KF_i]->mnId - nMinKF].resize(vpLocalKF[KF_i]->mNumCam);
//            for (int cam_i = 0; cam_i < vpLocalKF[KF_i]->mNumCam; cam_i++) {
//                pKF->mvMatchResult[vpLocalKF[KF_i]->mnId - nMinKF][cam_i].resize(vpLocalKF[KF_i]->mvNCams[cam_i]);
//            }
//        }
//
//        Eigen::Matrix4f Twb = mpCurrentFrame->GetTwb();
//        Eigen::Matrix3f Rwb = CommonTools::T2R(Twb);
//        Eigen::Vector3f twb = CommonTools::T2t(Twb);
//        pKF->mvMapPointPosition.reserve((nMaxKF - nMinKF + 1) * mNumCam * mnFeatures);
//        pKF->mvMapPointPosition.resize(mpCurrentFrame->mvMapPointPosition.size());
//        for (int i = 0; i < mpCurrentFrame->mvMapPointPosition.size(); i++) {
//            pKF->mvMapPointPosition[i] = Rwb * mpCurrentFrame->mvMapPointPosition[i] + twb;
//        }
//    }
//    // step 2 : update stereo matches
//
//    for (int cam_i = 0; cam_i < mpCurrentFrame->mvMapPoints.size(); cam_i++) {
//        for (int kpi = 0; kpi < mpCurrentFrame->mvMapPoints[cam_i].size(); kpi++) {
//            if (mpCurrentFrame->mvMatchResult[cam_i][kpi] >= 0) {
//                pKF->mvMatchResult[pKF->mnId - nMinKF][cam_i][kpi] = mpCurrentFrame->mvMatchResult[cam_i][kpi];
//            }
//        }
//    }
//
//
//    for (int cam_i = 0; cam_i < pKF->mNumCam; cam_i++) {
//        for (int KF_i = 0; KF_i < vpLocalKF.size(); KF_i++) {
//            matcher.SearchByEpipolarGeometryBetweenImage(pKF, cam_i, vpLocalKF[KF_i], cam_i, nMinKF);
//        }
//    }
//
//
//
//
//
//
//
//
//
//
//
//    /////////////////
//    std::chrono::steady_clock::time_point t4 = std::chrono::steady_clock::now();
//    cout << "SearchByEpipolarGeometryBetweenImage use time : " << std::chrono::duration_cast<std::chrono::duration<double> >(t4 - t3).count() * 1000.0 << " ms." << endl;
//
//
//
//    mpReferenceKF = pKF;
//
//    mpMap->UpdateLocalMap();
//
//    //    mpLocalMapper->InsertKeyFrame(pKF);
//
//
//    /////////////////
//    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
//    cout << "CreateNewKeyFrame use time : " << std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count() * 1000.0 << " ms." << endl;
//    return ;
//}
//
//




//namespace ORB_SLAM
//
//        cv::Mat imShow = vImCams[0].clone();
//        vector<cv::KeyPoint> vKeys = mpCurrentFrame->mvCamKeys[0];
//        for (int i = 0; i < vKeys.size(); i++) { if (vKeys[i].octave == 0) {cv::circle(imShow, cv::Point2f(vKeys[i].pt),2,cv::Scalar(0, 255, 0),-1);} }
//        cv::imshow("imShow", imShow);
//
////        Eigen::Vector3f t_w_viewer = mpReferenceKF->Gettwb();
//        Eigen::Vector3f t_w_viewer = mpCurrentFrame->Gettwb();
//
//        ///////visual
////        int w = 1000;
//        int w = 2500;
//        int h = 1400;
//        float fbl = 0.03;
////        float fbl = 0.3;
//        cv::Mat img_map = cv::Mat::zeros(cv::Size(w, h), CV_8UC3);
////      img_map.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);
//
//        //connection
//        for (int cam_i = 0; cam_i < mpCurrentFrame->mvMapPoints.size(); cam_i++) {
//
//            Eigen::Matrix4f Tbc = mpCurrentFrame->mvTbc_cams[cam_i];
//            Eigen::Matrix4f Tbw = mpCurrentFrame->GetPose();
//            Eigen::Matrix4f Twb = Tbw.inverse();
//            Eigen::Matrix4f Twc = Twb * Tbc;
//            int cols = mpCurrentFrame->mvWidthHeight[cam_i].first;
//            int rows = mpCurrentFrame->mvWidthHeight[cam_i].second;
//
//            for (int kpi = 0; kpi < mpCurrentFrame->mvMapPoints[cam_i].size(); kpi++) {
//                MapPoint *pMP = mpCurrentFrame->mvMapPoints[cam_i][kpi];
//                if (pMP == NULL) {
//                    continue;
//                }
//                Eigen::Vector3f Pw = pMP->GetWorldPos();
//
//
//                Eigen::Vector3f Pw_cameravertex_i = CommonTools::T2t(Twc) - t_w_viewer;
//                Eigen::Vector3f Pw_cameravertex_j = Pw - t_w_viewer;
//
//                int Pi_u = int( Pw_cameravertex_i.x() / fbl + w / 2.0);
//                int Pi_v = int(-Pw_cameravertex_i.z() / fbl + h / 2.0);
//                int Pj_u = int( Pw_cameravertex_j.x() / fbl + w / 2.0);
//                int Pj_v = int(-Pw_cameravertex_j.z() / fbl + h / 2.0);
//
//                cv::line(img_map, Point(Pi_u, Pi_v), Point(Pj_u, Pj_v), cv::Scalar(64, 64, 64), 1);
//            }
//        }
//
//        //connection in viewer frame
//        for (int cam_i = 0; cam_i < mpCurrentFrame->mvMapPoints.size(); cam_i++) {
//
//            Eigen::Matrix4f Tbc = mpCurrentFrame->mvTbc_cams[cam_i];
//            Eigen::Matrix4f Tbw = mpCurrentFrame->GetPose();
//            Eigen::Matrix4f Twb = Tbw.inverse();
//            Eigen::Matrix4f Twc = Twb * Tbc;
//            int cols = mpCurrentFrame->mvWidthHeight[cam_i].first;
//            int rows = mpCurrentFrame->mvWidthHeight[cam_i].second;
//
//            for (int kpi = 0; kpi < mpCurrentFrame->mvMapPoints[cam_i].size(); kpi++) {
//                MapPoint *pMP = mpCurrentFrame->mvMapPoints[cam_i][kpi];
//                if (pMP == NULL) {
//                    continue;
//                }
//                Eigen::Vector3f Pw = pMP->GetWorldPos();
//
//
//                Eigen::Vector3f Pw_cameravertex_i = CommonTools::T2t(Twc) - t_w_viewer;
//                Eigen::Vector3f Pw_cameravertex_j = Pw - t_w_viewer;
//
//                int Pi_u = int( Pw_cameravertex_i.x() / fbl + w / 2.0);
//                int Pi_v = int(-Pw_cameravertex_i.z() / fbl + h / 2.0);
//                int Pj_u = int( Pw_cameravertex_j.x() / fbl + w / 2.0);
//                int Pj_v = int(-Pw_cameravertex_j.z() / fbl + h / 2.0);
//
//                if (Pj_u >= 0 && Pj_u < w && Pj_v >= 0 && Pj_v < h) {
//                    cv::line(img_map, Point(Pi_u, Pi_v), Point(Pj_u, Pj_v), cv::Scalar(0, 128, 128), 1);
//                }
//
//            }
//        }
//
//
//        //ALL MapPoint
//        for (int MP_i = 0; MP_i < mpMap->mvpLocalMP.size(); MP_i++) {
//            MapPoint *pMP = mpMap->mvpLocalMP[MP_i];
//            if (pMP == NULL) {
//                continue;
//            }
//            Eigen::Vector3f Pw = pMP->GetWorldPos() - t_w_viewer;
//            int Pi_u = int( Pw.x() / fbl + w / 2.0);
//            int Pi_v = int(-Pw.z() / fbl + h / 2.0);
//            cv::circle(img_map, cv::Point2f(Pi_u, Pi_v),2,cv::Scalar(128, 128, 0),-1);
//
//        }
//        //Tracked MapPoint
//        for (int cam_i = 0; cam_i < mpCurrentFrame->mvMapPoints.size(); cam_i++) {
//            for (int kpi = 0; kpi < mpCurrentFrame->mvMapPoints[cam_i].size(); kpi++) {
//                MapPoint *pMP = mpCurrentFrame->mvMapPoints[cam_i][kpi];
//                if (pMP == NULL) {
//                    continue;
//                }
//                Eigen::Vector3f Pw = pMP->GetWorldPos() - t_w_viewer;
//                int Pi_u = int( Pw.x() / fbl + w / 2.0);
//                int Pi_v = int(-Pw.z() / fbl + h / 2.0);
//                cv::circle(img_map, cv::Point2f(Pi_u, Pi_v),2,cv::Scalar(255, 255, 0),-1);
//
//            }
//        }
//
//        //camera
//        float camera_size = 0.3;
//        vector<vector<float>> camera_vertex = {
//                {0, 0, 0},
//                {1.73 * camera_size, camera_size, camera_size},
//                {1.73 * camera_size, -camera_size, camera_size},
//                {-1.73 * camera_size, camera_size, camera_size},
//                {-1.73 * camera_size, -camera_size, camera_size}        };
//        for (int cam_i = 0; cam_i < mpCurrentFrame->mvMapPoints.size(); cam_i++) {
//
//            Eigen::Matrix4f Tbc = mpCurrentFrame->mvTbc_cams[cam_i];
//            Eigen::Matrix4f Tbw = mpCurrentFrame->GetPose();
//            Eigen::Matrix4f Twb = Tbw.inverse();
//            Eigen::Matrix4f Twc = Twb * Tbc;
//            for (int i = 0; i < camera_vertex.size(); i++) {
//                for (int j = i + 1; j < camera_vertex.size(); j++) {
//                    Eigen::Vector3f Pc_cameravertex_i(camera_vertex[i][0], camera_vertex[i][1], camera_vertex[i][2]);
//                    Eigen::Vector3f Pc_cameravertex_j(camera_vertex[j][0], camera_vertex[j][1], camera_vertex[j][2]);
//                    Eigen::Vector3f Pw_cameravertex_i = CommonTools::T2R(Twc) * Pc_cameravertex_i + CommonTools::T2t(Twc) - t_w_viewer;
//                    Eigen::Vector3f Pw_cameravertex_j = CommonTools::T2R(Twc) * Pc_cameravertex_j + CommonTools::T2t(Twc) - t_w_viewer;
//
//                    int Pi_u = int( Pw_cameravertex_i.x() / fbl + w / 2.0);
//                    int Pi_v = int(-Pw_cameravertex_i.z() / fbl + h / 2.0);
//                    int Pj_u = int( Pw_cameravertex_j.x() / fbl + w / 2.0);
//                    int Pj_v = int(-Pw_cameravertex_j.z() / fbl + h / 2.0);
//
//                    cv::line(img_map,
//                             Point(Pi_u, Pi_v),
//                             Point(Pj_u, Pj_v),
//                             cv::Scalar(0, 255, 0), 2);
//                }
//            }
//
//            Eigen::Vector3f Pc_cameravertex_i(camera_vertex[0][0], camera_vertex[0][1], camera_vertex[0][2]);
//            Eigen::Vector3f Pw_cameravertex_i = CommonTools::T2R(Twc) * Pc_cameravertex_i + CommonTools::T2t(Twc) - t_w_viewer;
//
//            int Pi_u = int( Pw_cameravertex_i.x() / fbl + w / 2.0);
//            int Pi_v = int(-Pw_cameravertex_i.z() / fbl + h / 2.0);
//            cv::circle(img_map, cv::Point2f(Pi_u, Pi_v),4,cv::Scalar(128, 255, 128),-1);
//
//            {
//                //axis_z
//                Eigen::Vector3f Pc_axis_z_i(0, 0, 0);
//                Eigen::Vector3f Pc_axis_z_j(0, 0, 0.75);
//                Eigen::Vector3f Pw_cameravertex_i = CommonTools::T2R(Twb) * Pc_axis_z_i + CommonTools::T2t(Twb) - t_w_viewer;
//                Eigen::Vector3f Pw_cameravertex_j = CommonTools::T2R(Twb) * Pc_axis_z_j + CommonTools::T2t(Twb) - t_w_viewer;
//                int Pi_u = int( Pw_cameravertex_i.x() / fbl + w / 2.0);
//                int Pi_v = int(-Pw_cameravertex_i.z() / fbl + h / 2.0);
//                int Pj_u = int( Pw_cameravertex_j.x() / fbl + w / 2.0);
//                int Pj_v = int(-Pw_cameravertex_j.z() / fbl + h / 2.0);
//                cv::line(img_map, Point(Pi_u, Pi_v), Point(Pj_u, Pj_v), cv::Scalar(0, 0, 255), 4);
//            }
//            {
//                //axis_x
//                Eigen::Vector3f Pc_axis_x_i(0, 0, 0);
//                Eigen::Vector3f Pc_axis_x_j(0.75, 0, 0);
//                Eigen::Vector3f Pw_cameravertex_i = CommonTools::T2R(Twb) * Pc_axis_x_i + CommonTools::T2t(Twb) - t_w_viewer;
//                Eigen::Vector3f Pw_cameravertex_j = CommonTools::T2R(Twb) * Pc_axis_x_j + CommonTools::T2t(Twb) - t_w_viewer;
//                int Pi_u = int( Pw_cameravertex_i.x() / fbl + w / 2.0);
//                int Pi_v = int(-Pw_cameravertex_i.z() / fbl + h / 2.0);
//                int Pj_u = int( Pw_cameravertex_j.x() / fbl + w / 2.0);
//                int Pj_v = int(-Pw_cameravertex_j.z() / fbl + h / 2.0);
//                cv::line(img_map, Point(Pi_u, Pi_v), Point(Pj_u, Pj_v), cv::Scalar(255, 0, 0), 4);
//            }
//        }
//
//
//
//        // 将所有像素设置为白色（最高亮度值）
//        cv::imshow("img_map", img_map);
////
//
//
//        cv::waitKey(2);



//        matcher.SearchByEpipolarGeometryBetweenImage(
//                pKF, 3, pKF->mvCamKeysUn[3],pKF->mvCamDescriptors[3],
//                pKF->mvK_cams[3], pKF->GetTwb() * pKF->mvTbc_cams[3],
//                mpReferenceKF, 3, mpReferenceKF->mvCamKeysUn[3],mpReferenceKF->mvCamDescriptors[3],
//                mpReferenceKF->mvK_cams[3], mpReferenceKF->GetTwb() * mpReferenceKF->mvTbc_cams[3]);
//
//        matcher.SearchByEpipolarGeometryBetweenImage(
//                pKF, 4, pKF->mvCamKeysUn[4],pKF->mvCamDescriptors[4],
//                pKF->mvK_cams[4], pKF->GetTwb() * pKF->mvTbc_cams[4],
//                mpReferenceKF, 4, mpReferenceKF->mvCamKeysUn[4],mpReferenceKF->mvCamDescriptors[4],
//                mpReferenceKF->mvK_cams[4], mpReferenceKF->GetTwb() * mpReferenceKF->mvTbc_cams[4]);
//
//        matcher.SearchByEpipolarGeometryBetweenImage(
//                pKF, 5, pKF->mvCamKeysUn[5],pKF->mvCamDescriptors[5],
//                pKF->mvK_cams[5], pKF->GetTwb() * pKF->mvTbc_cams[5],
//                mpReferenceKF, 5, mpReferenceKF->mvCamKeysUn[5],mpReferenceKF->mvCamDescriptors[5],
//                mpReferenceKF->mvK_cams[5], mpReferenceKF->GetTwb() * mpReferenceKF->mvTbc_cams[5]);
//
//        matcher.SearchByEpipolarGeometryBetweenImage(
//                pKF, 10, pKF->mvCamKeysUn[10],pKF->mvCamDescriptors[10],
//                pKF->mvK_cams[10], pKF->GetTwb() * pKF->mvTbc_cams[10],
//                mpReferenceKF, 10, mpReferenceKF->mvCamKeysUn[10],mpReferenceKF->mvCamDescriptors[10],
//                mpReferenceKF->mvK_cams[10], mpReferenceKF->GetTwb() * mpReferenceKF->mvTbc_cams[10]);


//        matcher.SearchByEpipolarGeometryBetweenImage(
//                pKF, 3, pKF->mvCamKeysUn[3],pKF->mvCamDescriptors[3],
//                pKF->mvK_cams[3], pKF->GetTwb() * pKF->mvTbc_cams[3],
//                pKF, 4, pKF->mvCamKeysUn[4],pKF->mvCamDescriptors[4],
//                pKF->mvK_cams[4], pKF->GetTwb() * pKF->mvTbc_cams[4]);
//
//        matcher.SearchByEpipolarGeometryBetweenImage(
//                pKF, 4, pKF->mvCamKeysUn[4],pKF->mvCamDescriptors[4],
//                pKF->mvK_cams[4], pKF->GetTwb() * pKF->mvTbc_cams[4],
//                pKF, 5, pKF->mvCamKeysUn[5],pKF->mvCamDescriptors[5],
//                pKF->mvK_cams[5], pKF->GetTwb() * pKF->mvTbc_cams[5]);
//
//        matcher.SearchByEpipolarGeometryBetweenImage(
//                pKF, 2, pKF->mvCamKeysUn[2],pKF->mvCamDescriptors[2],
//                pKF->mvK_cams[2], pKF->GetTwb() * pKF->mvTbc_cams[2],
//                pKF, 5, pKF->mvCamKeysUn[5],pKF->mvCamDescriptors[5],
//                pKF->mvK_cams[5], pKF->GetTwb() * pKF->mvTbc_cams[5]);
//
//        matcher.SearchByEpipolarGeometryBetweenImage(
//                pKF, 3, pKF->mvCamKeysUn[3],pKF->mvCamDescriptors[3],
//                pKF->mvK_cams[3], pKF->GetTwb() * pKF->mvTbc_cams[3],
//                pKF, 6, pKF->mvCamKeysUn[6],pKF->mvCamDescriptors[6],
//                pKF->mvK_cams[6], pKF->GetTwb() * pKF->mvTbc_cams[6]);
//
//        matcher.SearchByEpipolarGeometryBetweenImage(
//                pKF, 0, pKF->mvCamKeysUn[0],pKF->mvCamDescriptors[0],
//                pKF->mvK_cams[0], pKF->GetTwb() * pKF->mvTbc_cams[0],
//                pKF, 2, pKF->mvCamKeysUn[2],pKF->mvCamDescriptors[2],
//                pKF->mvK_cams[2], pKF->GetTwb() * pKF->mvTbc_cams[2]);

//        matcher.SearchByEpipolarGeometryBetweenImage(
//                pKF, 5, pKF->mvCamKeysUn[5],pKF->mvCamDescriptors[5],
//                pKF->mvK_cams[5], pKF->GetTwb() * pKF->mvTbc_cams[5],
//                mpReferenceKF, 2, mpReferenceKF->mvCamKeysUn[2],mpReferenceKF->mvCamDescriptors[2],
//                mpReferenceKF->mvK_cams[2], mpReferenceKF->GetTwb() * mpReferenceKF->mvTbc_cams[2]);