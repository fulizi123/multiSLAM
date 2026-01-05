
#include "LocalMapping.h"


namespace LL_SLAM
{
    LocalMapping::LocalMapping(System *pSystem) {
        mpSystem = pSystem;
        mpMap = pSystem->mpMap;
        mpTracker = pSystem->mpTracker;
    }

    void LocalMapping::Run() {
        while (1) {

            if (mvpFrame.empty()) {
                usleep(2000);
            } else {
                //continue;
                vector<cv::Mat> vImCams;
                Frame *pCurrentFrame;
                {
                    unique_lock<mutex> lock(mMutexMsg);
                    pCurrentFrame = mvpFrame.front();
                    mvpFrame.clear();
                }

                CreateNewKeyFrame(pCurrentFrame);

            }
        }
    }

    void LocalMapping::InsertFrame(Frame *pCurrentFrame)
    {
        unique_lock<mutex> lock(mMutexMsg);
        mvpFrame.push_back(pCurrentFrame);

    }




    void LocalMapping::CreateNewKeyFrame(Frame *pCurrentFrame)
    {
        cout << "LocalMapping::CreateNewKeyFrame : " << endl;
        cout << "pCurrentFrame : " << pCurrentFrame << endl;
        cout << "mpSystem : " << mpSystem << endl;

        Frame *mpCurrentFrame = pCurrentFrame;
        int mNumCam = mpSystem->mpTracker->mNumCam;
        int mnFeatures = mpSystem->mpTracker->mnFeatures;

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        /////////////////

        std::chrono::steady_clock::time_point t5 = std::chrono::steady_clock::now();
        /////////////////
        cout << "LocalMapping::ComputeMultiCameraMatches : " << endl;

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
        //todo
        int nMatches = matcher.SearchByProject(vpLocalMP, mpCurrentFrame);
        //matcher.SearchByProjectMultiCamera(vpLocalMP, mpCurrentFrame);


        /////////////////
        std::chrono::steady_clock::time_point t8 = std::chrono::steady_clock::now();
        cout << "SearchByProject Local Map use time : " << std::chrono::duration_cast<std::chrono::duration<double> >(t8 - t7).count() * 1000.0 << " ms." << endl;

        KeyFrame* pKF = new KeyFrame(mpCurrentFrame);


        // step 1 : init
//        vector<KeyFrame*> vpLocalKF = mpMap->mvpLocalKF;
        vector<KeyFrame*> vpLocalKF = mpMap->GetLocalKeyFrame();
        while (vpLocalKF.size() > 3) {vpLocalKF.pop_back();}
        // vpLocalKF.pop_back();
        //if (vpLocalKF.size() == 5) {vpLocalKF.pop_back();}  //we donot use the last KF, it will be removed
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
        // if (vpLocalKF.size() >= 2) {
        //     matcher.SearchByEpipolarGeometryMultiCamera(vpLocalKF, nMinKF, nMaxKF);
        // }
        // matcher.SearchByEpipolarGeometryMultiCamera(vpLocalKF, nMinKF, nMaxKF);

        /////////////////
        std::chrono::steady_clock::time_point t4 = std::chrono::steady_clock::now();
        cout << "SearchByEpipolarGeometryBetweenImage use time : " << std::chrono::duration_cast<std::chrono::duration<double> >(t4 - t3).count() * 1000.0 << " ms." << endl;



        //step 4 : fuse the data
        //[KFi][cam_i][kpi] -> index
        vector< MapPoint * > vMapPoints(pKF->mvMapPointPosition.size(), NULL);
        vector<MapPoint*> vNewMapPoints;
        {
            cout << "Debug mMutexUpdate step 4.1 "  << endl ;

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

            cout << "Debug mMutexUpdate step 4.2 "  << endl ;

            //step 4.2 : Matches those without exist MapPoint , Create one.
            vNewMapPoints.reserve(vMapPoints.size());
            for (int i = 0; i < vMapPoints.size(); i++) {
                //none of its observation is matched with MapPoint
                if (vMapPoints[i] == NULL) {
                    vMapPoints[i] = new MapPoint(pKF->mvMapPointPosition[i]);
                    vNewMapPoints.push_back(vMapPoints[i]);
                }
            }

            cout << "Debug mMutexUpdate step 4.3 "  << endl ;

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



        cout << "Debug mMutexUpdate step 5 "  << endl ;

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


//        mpReferenceKF = pKF;
        {
            unique_lock<mutex> lock(mpSystem->mpTracker->mMutexUpdate);
            mpSystem->mpTracker->mpReferenceKF = pKF;
        }
        cout << "Debug mMutexUpdate 1 "  << endl ;
        {
            unique_lock<mutex> lock(mpMap->mMutexUpdate);
            mpMap->AddKeyFrame(pKF);

            for (int i = 0; i < vNewMapPoints.size(); i++) {
                mpMap->AddMapPoint(vNewMapPoints[i]);
            }

            mpMap->UpdateLocalMap();
        }

        cout << "Debug mMutexUpdate 2 "  << endl ;

        mpCurrentFrame->mbCanBeRelease = true;
        mpSystem->mpTracker->ReleaseResource(mpCurrentFrame);


        /////////////////
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        cout << "CreateNewKeyFrame use time : " << std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count() * 1000.0 << " ms." << endl;
        return ;
    }


} //namespace ORB_SLAM
