/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include "FeatureMatcher.h"

#include<limits.h>

#include<opencv2/core/core.hpp>

//#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"

#include<stdint-gcc.h>

using namespace std;

namespace LL_SLAM
{

    const int FeatureMatcher::TH_HIGH = 100;
    const int FeatureMatcher::TH_LOW = 50;
    // const int FeatureMatcher::TH_HIGH = 200;
    // const int FeatureMatcher::TH_LOW = 60;
    const int FeatureMatcher::HISTO_LENGTH = 30;

    FeatureMatcher::FeatureMatcher(float nnratio, bool checkOri): mfNNratio(nnratio), mbCheckOrientation(checkOri)
    {
    }






    int FeatureMatcher::SearchByProjectMultiCamera(std::vector<MapPoint*> &vpLocalMP, Frame *pF, int nMaxMatchNum) {

        cout << "vpLocalMP.size() " << vpLocalMP.size() << endl ;
        int maxLevel = 8;
        int minLevel = -1;
        if (pF->mpSystem->mbUseDesc == true) {
            maxLevel = -1;
            minLevel = -1;
        }

        Eigen::Matrix4f Tbw_prediction = pF->PredictPose();
        int NumCam = pF->mNumCam;
        vector<pair<int, int>> vWidthHeight = pF->mvWidthHeight;

        vector<Eigen::Matrix4f> vTcw(NumCam);
        vector<Eigen::Matrix3f> vRcw(NumCam);
        vector<Eigen::Vector3f> vtcw(NumCam);
        for (int cam_i = 0; cam_i < NumCam; cam_i++) {
            vTcw[cam_i] = pF->mvTbc_cams[cam_i].inverse() * Tbw_prediction;
            vRcw[cam_i] = CommonTools::T2R(vTcw[cam_i]);
            vtcw[cam_i] = CommonTools::T2t(vTcw[cam_i]);
        }
        vector<Eigen::Matrix3f> &vK_cams = pF->mvK_cams;

        vector<int> vnMatches(NumCam, 0);
        int nMaxMatchNumPerImage = nMaxMatchNum == -1 ? -1 : nMaxMatchNum / NumCam;

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
//        std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
        /////////////////
        vector<thread> vthreads;
        vthreads.reserve(NumCam);
        cout << "Debug SearchByProjectMultiCamera 1 "  << endl ;
        for (int cam_i = 0; cam_i < NumCam; cam_i++) {
            vthreads.emplace_back(&FeatureMatcher::SearchByProjectMultiThread,this, &vpLocalMP, pF, cam_i, minLevel, maxLevel, vWidthHeight[cam_i].first, vWidthHeight[cam_i].second, nMaxMatchNumPerImage,
                                  vRcw[cam_i], vtcw[cam_i], vK_cams[cam_i],
                                  &vnMatches);
        }
        cout << "Debug SearchByProjectMultiCamera 2 "  << endl ;
//        /////////////////
//        std::chrono::steady_clock::time_point t4 = std::chrono::steady_clock::now();
//        cout << "vthreads.emplace_back use time : " << std::chrono::duration_cast<std::chrono::duration<double> >(t4 - t3).count() * 1000.0 << " ms." << endl;

        for (int cam_i = 0; cam_i < NumCam; cam_i++) {
            vthreads[cam_i].join();
        }
        cout << "Debug SearchByProjectMultiCamera 3 "  << endl ;
        /////////////////
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        cout << "SearchByProjectMultiCamera use time : " << std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count() * 1000.0 << " ms." << endl;

        cout << "vnMatches[cam_i].size() " ;
        for (int cam_i = 0; cam_i < NumCam; cam_i++) {
            cout << " " << vnMatches[cam_i];
        }
        cout << endl;


        int nmatches = 0;
        for (int cam_i = 0; cam_i < NumCam; cam_i++) {
            nmatches += vnMatches[cam_i];
        }

        return nmatches;

    }





    void FeatureMatcher::SearchByProjectMultiThread(vector<MapPoint*> *pvpLocalMP, Frame *pF, int cam_i, int minLevel, int maxLevel, int w, int h, int nMaxMatchNum,
                                                const Eigen::Matrix3f &Rcw, const Eigen::Vector3f &tcw, const Eigen::Matrix3f &K_cam,
                                                vector<int> *pvnMatches) {

        int NumCam = pF->mNumCam;
        int nProjNum = nMaxMatchNum == -1 ? INT_MAX : nMaxMatchNum * 4;

        std::vector<int> IndexShuffle((*pvpLocalMP).size(), 0);
        for (int i = 0; i < (*pvpLocalMP).size(); i++) { IndexShuffle[i] = i; }
        if (nMaxMatchNum != -1) {
            shuffle(IndexShuffle.begin(), IndexShuffle.end(),
                    std::default_random_engine(std::chrono::system_clock::now().time_since_epoch().count()));
        }

        vector<cv::KeyPoint> vToDistributeKeys;
        vToDistributeKeys.reserve(nProjNum);
        //search loop
        for (int MPi = 0; MPi < (*pvpLocalMP).size(); MPi++) {
//            MapPoint* pMP = (*pvpLocalMP)[MPi];
            int MPOrderIndex = IndexShuffle[MPi];
            MapPoint* pMP = (*pvpLocalMP)[MPOrderIndex];
            if (pMP == NULL) {continue;}
            Eigen::Vector3f Pw = pMP->GetWorldPos();
            Eigen::Vector3f Pc = Rcw * Pw + tcw;
            if (Pc.z() <= 0 ) {continue;}
            Eigen::Vector3f p_2D =  1.0 / Pc.z() * Pc;
            Eigen::Vector3f pixel_2D = K_cam * p_2D;

            int u = pixel_2D.x();
            int v = pixel_2D.y();
            if (u < 0 || u >= w || v < 0 || v >= h ) {continue;}

            vector<int> vIndices = pF->GetFeaturesInArea(u, v, 15, cam_i, minLevel, maxLevel);

            if(!vIndices.empty()){
                const cv::Mat MPdescriptor = pMP->GetDescriptor();

                int bestDist=256;
                int bestLevel= -1;
                int bestDist2=256;
                int bestLevel2 = -1;
                int bestIdx =-1 ;

                // Get best and second matches with near keypoints
                for(vector<int>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
                {
                    const size_t idx = *vit;

                    const cv::Mat &d = pF->mvCamDescriptors[cam_i].row(idx);

                    //30ms
                    const int dist = DescriptorDistance(MPdescriptor,d);

                    if(dist<bestDist)
                    {
                        bestDist2=bestDist;
                        bestDist=dist;
                        bestLevel2 = bestLevel;
                        bestLevel = pF->mvCamKeysUn[cam_i][idx].octave;
                        bestIdx=idx;
                    }
                    else if(dist<bestDist2)
                    {
                        bestLevel2 = pF->mvCamKeysUn[cam_i][idx].octave;
                        bestDist2=dist;
                    }
                }

                // Apply ratio to second match (only if best and second are in the same scale level)
                if(bestDist<=TH_HIGH)
                {
                    if(bestLevel==bestLevel2 && bestDist>mfNNratio*bestDist2)
                        continue;

                    if(bestLevel!=bestLevel2 || bestDist<=mfNNratio*bestDist2){



                        cv::KeyPoint &kp = pF->mvCamKeysUn[cam_i][bestIdx];
                        kp.class_id = MPOrderIndex + bestIdx * int(1e5);

                        vToDistributeKeys.push_back(kp);

//                        pF->mvMapPoints[cam_i][bestIdx] = pMP;

                        (*pvnMatches)[cam_i]++;

                        if ( (*pvnMatches)[cam_i] >= nProjNum) {
                            break;
                        }
//                        pF->mvMapPoints[cam_i][bestIdx] = pMP;
//
//                        (*pvnMatches)[cam_i]++;
//
//                        if ( (*pvnMatches)[cam_i] >= nProjNum) {
//                            break;
//                        }
                    }
                }
            }

        }

        if (nMaxMatchNum == -1) {
            for (int i = 0; i < vToDistributeKeys.size(); i++) {
                cv::KeyPoint &kp = vToDistributeKeys[i];
                int MPOrderIndex = kp.class_id % int(1e5);
                int bestIdx = kp.class_id / int(1e5);
                pF->mvMapPoints[cam_i][bestIdx] = (*pvpLocalMP)[MPOrderIndex];
                kp.class_id = -1;
            }
        } else {
            const int minBorderX = 0;
            const int minBorderY = 0;
            const int maxBorderX = pF->mvWidthHeight[cam_i].first;
            const int maxBorderY = pF->mvWidthHeight[cam_i].second;

            vector<cv::KeyPoint> keypoints = pF->mpSystem->mpTracker->mvpFeatureExtractor[cam_i]->DistributeOctTree(
                                        vToDistributeKeys, minBorderX, maxBorderX,
                                          minBorderY, maxBorderY,nMaxMatchNum, -1);

            for (int i = 0; i < keypoints.size(); i++) {
                cv::KeyPoint &kp = keypoints[i];
                int MPOrderIndex = kp.class_id % int(1e5);
                int bestIdx = kp.class_id / int(1e5);
                pF->mvMapPoints[cam_i][bestIdx] = (*pvpLocalMP)[MPOrderIndex];
                kp.class_id = -1;
            }

            (*pvnMatches)[cam_i] = keypoints.size();

        }


        return ;
    }







    int FeatureMatcher::SearchByProject(std::vector<MapPoint*> &vpLocalMP, Frame *pF) {

        int maxLevel = 8;
        int minLevel = -1;
        if (pF->mpSystem->mbUseDesc == true) {
            maxLevel = -1;
            minLevel = -1;
        }

        Eigen::Matrix4f Tbw_prediction = pF->PredictPose();
        Eigen::Matrix3f Rbw = CommonTools::T2R(Tbw_prediction);
        Eigen::Vector3f tbw = CommonTools::T2t(Tbw_prediction);
        int nmatches = 0;
        int NumCam = pF->mNumCam;
        vector<pair<int, int>> vWidthHeight = pF->mvWidthHeight;

        vector<Eigen::Matrix4f> vTcb_cams(NumCam);
        vector<Eigen::Matrix3f> vRcb_cams(NumCam);
        vector<Eigen::Vector3f> vtcb_cams(NumCam);
        for (int cam_i = 0; cam_i < NumCam; cam_i++) {
            vTcb_cams[cam_i] = pF->mvTbc_cams[cam_i].inverse();
            vRcb_cams[cam_i] = CommonTools::T2R(vTcb_cams[cam_i]);
            vtcb_cams[cam_i] = CommonTools::T2t(vTcb_cams[cam_i]);
        }
        vector<Eigen::Matrix3f> &vK_cams = pF->mvK_cams;

        //search loop
        for (int MPi = 0; MPi < vpLocalMP.size(); MPi++) {
            MapPoint* pMP = vpLocalMP[MPi];
            if (pMP == NULL) {continue;}
            Eigen::Vector3f Pw = pMP->GetWorldPos();
            Eigen::Vector3f Pb = Rbw * Pw + tbw;
            for (int cam_i = 0; cam_i < NumCam; cam_i++) {
                Eigen::Vector3f Pc = vRcb_cams[cam_i] * Pb + vtcb_cams[cam_i];
                if (Pc.z() <= 0 ) {continue;}
                Eigen::Vector3f p_2D =  1.0 / Pc.z() * Pc;
                Eigen::Vector3f pixel_2D = vK_cams[cam_i] * p_2D;

                int u = pixel_2D.x();
                int v = pixel_2D.y();
                if (u < 0 || u >= vWidthHeight[cam_i].first || v < 0 || v >= vWidthHeight[cam_i].second ) {continue;}

                vector<int> vIndices = pF->GetFeaturesInArea(u, v, 15, cam_i, minLevel, maxLevel);

//                cout << "vIndices " << vIndices.size() << endl;
                if(!vIndices.empty()){
                    const cv::Mat MPdescriptor = pMP->GetDescriptor();

                    int bestDist=256;
                    int bestLevel= -1;
                    int bestDist2=256;
                    int bestLevel2 = -1;
                    int bestIdx =-1 ;

                    // Get best and second matches with near keypoints
                    for(vector<int>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
                    {
                        const size_t idx = *vit;

                        const cv::Mat &d = pF->mvCamDescriptors[cam_i].row(idx);

                        //30ms
                        const int dist = DescriptorDistance(MPdescriptor,d);
//                        const int dist = max(min(int( - 256.410 * MPdescriptor.dot(d) + 252.564), 256), 0);
                        //const int dist = MPdescriptor.at<float>(0, 0);


                        if(dist<bestDist)
                        {
                            bestDist2=bestDist;
                            bestDist=dist;
                            bestLevel2 = bestLevel;
                            bestLevel = pF->mvCamKeysUn[cam_i][idx].octave;
                            bestIdx=idx;
                        }
                        else if(dist<bestDist2)
                        {
                            bestLevel2 = pF->mvCamKeysUn[cam_i][idx].octave;
                            bestDist2=dist;
                        }
                    }

                    // Apply ratio to second match (only if best and second are in the same scale level)
                    if(bestDist<=TH_HIGH)
                    {
                        if(bestLevel==bestLevel2 && bestDist>mfNNratio*bestDist2)
                            continue;

                        if(bestLevel!=bestLevel2 || bestDist<=mfNNratio*bestDist2){

                            pF->mvMapPoints[cam_i][bestIdx] = pMP;

                            nmatches++;
                        }
                    }
                }
                //cout << "vIndices " << vIndices.size() << endl;

            }
            //cout << 1 << endl;
        }


        return nmatches ;
    }







    int FeatureMatcher::SearchByReferenceKFBruteForce(KeyFrame *pKF, Frame *pF) {
        int nmatches = 0;


        for (int cam_i = 0; cam_i < pF->mNumCam; cam_i++) {

            vector<cv::KeyPoint> &vCamKeysCamF = pF->mvCamKeysUn[cam_i];
            cv::Mat &CamDescriptorsCamF = pF->mvCamDescriptors[cam_i];

//            vector<cv::KeyPoint> vCamKeysCamKFALL = pKF->mvCamKeys[cam_i];
//            cv::Mat CamDescriptorsCamKFALL = pKF->mvCamDescriptors[cam_i];

            //vector<cv::KeyPoint> vCamKeysCamKFPart;
            //vector<cv::Mat> CamDescriptorsCamKFPart;
            vector<pair<int, int>> vIndexToCamAndKeyPoint;
            //vCamKeysCamKFPart.reserve(pKF->mvNCams[cam_i]);
            //CamDescriptorsCamKFPart.reserve(pKF->mvNCams[cam_i]);
            vIndexToCamAndKeyPoint.reserve(pKF->mvNCams[cam_i]);

            for (int KeyPoint_i = 0; KeyPoint_i < pKF->mvNCams[cam_i]; KeyPoint_i++) {
                if ( pKF->mvMapPoints[cam_i][KeyPoint_i] != NULL) {
                    //there is a init mappoint
                    //vCamKeysCamKFPart.push_back(pKF->mvCamKeys[cam_i][KeyPoint_i]);
                    //CamDescriptorsCamKFPart.push_back(pKF->mvCamDescriptors[cam_i].row(KeyPoint_i));
                    vIndexToCamAndKeyPoint.push_back({cam_i, KeyPoint_i});
                }
            }
            vector<bool> vIsUsed(vIndexToCamAndKeyPoint.size(), false);




            for (int i = 0; i < vCamKeysCamF.size(); i++) {
                cv::KeyPoint pKF1 = vCamKeysCamF[i];
                //cv::Mat Desc1 = CamDescriptorsCamF.row(i);

                //best / second > nnratio
                //|pt1 - pt2| < 100.0
                int bestDist1=256;
                int bestIdx2 =-1 ;
                int bestDist2=256;
                int num = 0;
                for (int j = 0; j < vIndexToCamAndKeyPoint.size(); j++) {
//                    cv::KeyPoint pKF2 = vCamKeysCamKFPart[j];
                    cv::KeyPoint pKF2 = pKF->mvCamKeysUn[vIndexToCamAndKeyPoint[j].first][vIndexToCamAndKeyPoint[j].second];
                    if (vIsUsed[j]) {continue;}
                    if (abs(pKF1.pt.x - pKF2.pt.x)+abs(pKF1.pt.y - pKF2.pt.y) >= 150) {continue;}

                    //cv::Mat Desc2 = CamDescriptorsCamKFPart[j];
                    //const int dist = FeatureMatcher::DescriptorDistance(Desc1,Desc2);
                    //const int dist = FeatureMatcher::DescriptorDistance(CamDescriptorsCamF.row(i),CamDescriptorsCamKFPart[j]);
                    const int dist = FeatureMatcher::DescriptorDistance(CamDescriptorsCamF.row(i),pKF->mvCamDescriptors[vIndexToCamAndKeyPoint[j].first].row(vIndexToCamAndKeyPoint[j].second));
                    num++;
                    if(dist<bestDist1)
                    {
                        bestDist2=bestDist1;
                        bestDist1=dist;
                        bestIdx2=j;
                    }
                    else if(dist<bestDist2)
                    {
                        bestDist2=dist;
                    }
                }

                //successful
                if (bestDist1 >= TH_LOW) {
                    continue;
                }
                if (bestDist1 == 256 || bestDist2 == 256) {
                    continue;
                }
                if (static_cast<float>(bestDist1)<mfNNratio*static_cast<float>(bestDist2)) {
                    vIsUsed[bestIdx2] = true;
                    pF->mvMapPoints[cam_i][i] = pKF->mvMapPoints[vIndexToCamAndKeyPoint[bestIdx2].first][vIndexToCamAndKeyPoint[bestIdx2].second];
                    nmatches++;
                }
            }



            cout << 1 << endl;
        }

        return nmatches;
    }





    inline int signum(int x) {return x == 0 ? 0 : x < 0 ? -1 : 1;}

    inline float intbound(float s, float ds) {
        // Find the smallest positive t such that s+t*ds is an integer.
        if (ds < 0) {
            return intbound(-s, -ds);
        } else {
            float s_zhengshu;
            s = std::modf(s, &s_zhengshu);
            // problem is now s+t*ds = 1
            return (1 - s) / ds;
        }
    }

    //find p2 on 1
    int FeatureMatcher::SearchByEpipolarGeometryBetweenImage(KeyFrame *pKF2, int cam_2, KeyFrame *pKF1, int cam_1, int nMinKF, int nMaxKF)
    {
        //
        cout << "Debug SearchByEpipolarGeometryBetweenImage 1" << endl;
        vector<cv::KeyPoint> &CamKeysUn2 = pKF2->mvCamKeysUn[cam_2];
        cv::Mat &CamDescriptors2 = pKF2->mvCamDescriptors[cam_2];
        Eigen::Matrix3f K2 = pKF2->mvK_cams[cam_2];
        vector<cv::KeyPoint> &CamKeysUn1 = pKF1->mvCamKeysUn[cam_1];
        cv::Mat &CamDescriptors1 = pKF1->mvCamDescriptors[cam_1];
        Eigen::Matrix3f K1 = pKF1->mvK_cams[cam_1];

        Eigen::Matrix4f Twc1 = pKF1->GetTwb() * pKF1->mvTbc_cams[cam_1];
        Eigen::Matrix3f Rwc1 = CommonTools::T2R(Twc1) ;
        Eigen::Vector3f twc1 = CommonTools::T2t(Twc1) ;
        Eigen::Matrix4f Twc2 = pKF2->GetTwb() * pKF2->mvTbc_cams[cam_2];
        Eigen::Matrix3f Rwc2 = CommonTools::T2R(Twc2) ;
        Eigen::Vector3f twc2 = CommonTools::T2t(Twc2) ;

        Eigen::Matrix4f Tc1w = Twc1.inverse();
        Eigen::Matrix3f Rc1w = CommonTools::T2R(Tc1w) ;
        Eigen::Vector3f tc1w = CommonTools::T2t(Tc1w) ;
        Eigen::Matrix4f Tc2w = Twc2.inverse();
        Eigen::Matrix3f Rc2w = CommonTools::T2R(Tc2w) ;
        Eigen::Vector3f tc2w = CommonTools::T2t(Tc2w) ;


        Eigen::Matrix4f T21 = Twc2.inverse() * Twc1;
        Eigen::Matrix3f R21 = CommonTools::T2R(T21) ;
        Eigen::Vector3f t21 = CommonTools::T2t(T21) ;
        Eigen::Matrix3f K1_inv = K1.inverse() ;
        Eigen::Matrix3f K2_inv = K1.inverse() ;
        Eigen::Matrix3f F = K2_inv.transpose() * CommonTools::toSkewMatrix(t21) * R21 * K1_inv;

        float w1 = pKF1->mvWidthHeight[cam_1].first;
        float h1 = pKF1->mvWidthHeight[cam_1].second;
        float w2 = pKF1->mvWidthHeight[cam_2].first;
        float h2 = pKF1->mvWidthHeight[cam_2].second;
        int N1 = CamKeysUn1.size();
        int N2 = CamKeysUn2.size();

        //MatchResult1[i] = MatchResult2[j] is a match
        //Pw = Pws[MatchResult1[i]];
        vector<int> MatchResult1(N1, -1);
        vector<int> MatchResult2(N2, -1);
        vector<int> Pws;
        Pws.reserve(min(N1, N2));
        float mfGridElementWidthInv = static_cast<float>(Frame::FRAME_GRID_COLS)/(w1);
        float mfGridElementHeightInv = static_cast<float>(Frame::FRAME_GRID_ROWS)/(h1);
        float MaxTimes = 20.0;
        float MinTimes = 1.5;
        float DistTH = t21.norm() * MaxTimes; //too big
        //float DistTH = 80.0;
        int num_matches = 0;

        vector<vector<bool>> visited(Frame::FRAME_GRID_COLS, vector<bool>(Frame::FRAME_GRID_ROWS, false));
        vector<pair<int, int>> vPath;
        vPath.reserve((Frame::FRAME_GRID_COLS + Frame::FRAME_GRID_ROWS) * 2);
        vector<int> vEpipolarPotentialPoints;
        vEpipolarPotentialPoints.reserve(pKF1->mvNCams[cam_1]);

        for (int kpi_2 = 0; kpi_2 < N2; kpi_2++) {
            //step 1 : Fundamental
            Eigen::Vector3f p2(CamKeysUn2[kpi_2].pt.x, CamKeysUn2[kpi_2].pt.y, 1.0);
            Eigen::Matrix<float,1,3> line2 = p2.transpose() * F;
            line2 = line2 / line2.block<1, 2>(0, 0 ).norm();
            //ax + by + c = 0
            float a = line2[0]; float b = line2[1]; float c = line2[2];

            //step 2 : Find begin end point
            //u [0, w-1]  has w points
            //v [0, h-1]  has h points
            vector<Eigen::Vector2f> vPointBoards;
            {
                float u = 0;
                float v = (-a*u-c) / (b + 1e-10);
                if (v >= 0 && v <=h1-1) {vPointBoards.push_back(Eigen::Vector2f(u, v));}
            }
            {
                float u = w1 - 1;
                float v = (-a*u-c) / (b + 1e-10);
                if (v >= 0 && v <=h1-1) {vPointBoards.push_back(Eigen::Vector2f(u, v));}
            }
            {
                float v = 0;
                float u = (-b*v-c) / (a + 1e-10);
                if (u >= 0 && u <=w1-1) {vPointBoards.push_back(Eigen::Vector2f(u, v));}
            }
            {
                float v = h1 - 1;
                float u = (-b*v-c) / (a + 1e-10);
                if (u >= 0 && u <=w1-1) {vPointBoards.push_back(Eigen::Vector2f(u, v));}
            }


            if (vPointBoards.size() != 2) {continue;}
            if (vPointBoards[0][0] > vPointBoards[1][0]) {swap(vPointBoards[0], vPointBoards[1]);}

            vector<Eigen::Vector2f> vPointNearestFarest;
            Eigen::Vector3f Pc2Nearest = K2_inv * p2 * MinTimes;
            Eigen::Vector3f PwNearest = Rwc2 * Pc2Nearest + twc2;
            Eigen::Vector3f Pc1Nearest = Rc1w * PwNearest + tc1w;
            Eigen::Vector3f pc1Nearest = Pc1Nearest / Pc1Nearest.z();
            Eigen::Vector3f pp1Nearest = K1 * pc1Nearest;
            vPointNearestFarest.push_back(Eigen::Vector2f(pp1Nearest.x(), pp1Nearest.y()));
            Eigen::Vector3f Pc2Farest = K2_inv * p2 * MaxTimes;
            Eigen::Vector3f PwFarest = Rwc2 * Pc2Farest + twc2;
            Eigen::Vector3f Pc1Farest = Rc1w * PwFarest + tc1w;
            Eigen::Vector3f pc1Farest = Pc1Farest / Pc1Farest.z();
            Eigen::Vector3f pp1Farest = K1 * pc1Farest;
            vPointNearestFarest.push_back(Eigen::Vector2f(pp1Farest.x(), pp1Farest.y()));
            if (vPointNearestFarest[0][0] > vPointNearestFarest[1][0]) {swap(vPointNearestFarest[0], vPointNearestFarest[1]);}

            if (Pc1Nearest.z() >= 0 && Pc1Farest.z() >= 0  ) {
                //this two is in the picture
                if (vPointNearestFarest[0][0] > vPointBoards[0][0]) {vPointBoards[0] = vPointNearestFarest[0];}
                if (vPointNearestFarest[1][0] < vPointBoards[1][0]) {vPointBoards[1] = vPointNearestFarest[1];}
            }


            //step 3 : find points on the path
//            Eigen::Vector2f PointBoardBegin = vPointBoards[0];
//            Eigen::Vector2f PointBoardEnd = vPointBoards[1];
////            Eigen::Vector2f PointBoardDirection = PointBoardEnd - PointBoardBegin;
////            PointBoardDirection = PointBoardDirection / PointBoardDirection.norm();
//
//            cout << "PointBoardBegin " << PointBoardBegin.transpose() << endl;
//            cout << "PointBoardEnd " << PointBoardEnd.transpose() << endl;


            //in Grid coordinery
            Eigen::Vector2f start(vPointBoards[0].x() * mfGridElementWidthInv, vPointBoards[0].y() * mfGridElementHeightInv );
            Eigen::Vector2f end(vPointBoards[1].x() * mfGridElementWidthInv, vPointBoards[1].y() * mfGridElementHeightInv );

            int x = (int) std::floor(start.x());
            int y = (int) std::floor(start.y());
            int endX = (int) std::floor(end.x());
            int endY = (int) std::floor(end.y());

            float dx = endX - x, dy = endY - y;

            Eigen::Vector2f direction = (end - start);
            float maxDist = direction.squaredNorm();

            int stepX = (int) signum((int) dx);
            int stepY = (int) signum((int) dy);

            float tDeltaX = ((float) stepX) / dx;
            float tDeltaY = ((float) stepY) / dy;

            float tMaxX = intbound(start.x(), dx);
            float tMaxY = intbound(start.y(), dy);

            // Avoids an infinite loop.
            if (stepX == 0 && stepY == 0) {continue;}

            vPath.clear();
            vEpipolarPotentialPoints.clear();
            double dist = 0;
            while (true) {
                if (x >= 0 && x < Frame::FRAME_GRID_COLS &&
                    y >= 0 && y < Frame::FRAME_GRID_ROWS ) {
//                    cout << "grid " << x << " " << y << endl;
                    vPath.push_back({x, y});


                    vPath.push_back({x+1, y});
                    vPath.push_back({x-1, y});
                    vPath.push_back({x, y+1});
                    vPath.push_back({x, y-1});

//                    for (int i = 0; i < pKF1->mvGridMultiCamera[cam_1][x][y].size(); i++) {
//                        vEpipolarPotentialPoints.push_back(pKF1->mvGridMultiCamera[cam_1][x][y][i]);
//                    }
                    dist = (Eigen::Vector2f(x, y) - start).squaredNorm();
                }


                if (dist > maxDist) { break; };
                if (x >= endX && y >= endY) { break; }

                if (tMaxX < tMaxY) {
                        x += stepX;
                        tMaxX += tDeltaX;
                } else {
                        y += stepY;
                        tMaxY += tDeltaY;
                }
            }



            for (int i = 0; i < vPath.size(); i++) {
                int x = vPath[i].first;
                int y = vPath[i].second;

                if (x < 0 || x >= Frame::FRAME_GRID_COLS || y < 0 || y >= Frame::FRAME_GRID_ROWS ) {continue;}
                if (visited[x][y]) {continue;}

                visited[x][y] = true;
                for (int j = 0; j < pKF1->mvGridMultiCamera[cam_1][x][y].size(); j++) {
                    vEpipolarPotentialPoints.push_back(pKF1->mvGridMultiCamera[cam_1][x][y][j]);
                }
            }
            for (int i = 0; i < vPath.size(); i++) {
                int x = vPath[i].first;
                int y = vPath[i].second;

                if (x < 0 || x >= Frame::FRAME_GRID_COLS || y < 0 || y >= Frame::FRAME_GRID_ROWS ) {continue;}
                visited[x][y] = false;
            }


            //step 4 : match
//            cout << "vEpipolarPotentialPoints " << vEpipolarPotentialPoints.size() << endl;

            if(!vEpipolarPotentialPoints.empty()){
                const cv::Mat MPdescriptor = pKF2->mvCamDescriptors[cam_2].row(kpi_2);

                int bestDist=256;
                int bestLevel= -1;
                int bestDist2=256;
                int bestLevel2 = -1;
                int bestIdx =-1 ;
                float EpipolarTH = 15;

                // Get best and second matches with near keypoints
                for(vector<int>::const_iterator vit=vEpipolarPotentialPoints.begin(), vend=vEpipolarPotentialPoints.end(); vit!=vend; vit++)
                {
                    const size_t idx = *vit;

                    if (pKF2->mvMatchResult[nMaxKF - pKF1->mnId][cam_1][idx] >= 0 && pKF2->mvMatchResult[nMaxKF - pKF2->mnId][cam_2][kpi_2] >= 0  ) {
                        continue;
                    }

                    const cv::Mat &d = pKF1->mvCamDescriptors[cam_1].row(idx);


                    //not save time
//                    cv::KeyPoint &kp = pKF1->mvCamKeysUn[cam_1][idx];
//                    Eigen::Vector3f p1(kp.pt.x, kp.pt.y, 1.0);
//                    float dis = abs((line2 * p1)(0, 0));
//                    if (dis >= EpipolarTH) {continue;}

                    //30ms
                    const int dist = DescriptorDistance(MPdescriptor,d);
//                    const int dist = 0;

                    if(dist<bestDist)
                    {
                        bestDist2=bestDist;
                        bestDist=dist;
                        bestLevel2 = bestLevel;
                        bestLevel = pKF1->mvCamKeysUn[cam_1][idx].octave;
                        bestIdx=idx;
                    }
                    else if(dist<bestDist2)
                    {
                        bestLevel2 = pKF1->mvCamKeysUn[cam_1][idx].octave;
                        bestDist2=dist;
                    }
                }

                // Apply ratio to second match (only if best and second are in the same scale level)
                if(bestDist<=TH_HIGH)
                {
                    if(bestLevel==bestLevel2 && bestDist>mfNNratio*bestDist2)
                        continue;

                    if(bestLevel!=bestLevel2 || bestDist<=mfNNratio*bestDist2){
                        int kpi_1 = bestIdx;
                        cv::KeyPoint &kp = pKF1->mvCamKeysUn[cam_1][bestIdx];
                        Eigen::Vector3f p1(kp.pt.x, kp.pt.y, 1.0);
                        float dis = abs((line2 * p1)(0, 0));
//                        cout << "dis " << dis << endl;
//                        cout << "bestDist " << bestDist << endl;
//                        cout << "p1 " << p1.transpose() << endl;
//                        cout << "p2 " << p2.transpose() << endl;
                        Eigen::Vector3f x_c1 = K1_inv * p1;
                        Eigen::Vector3f x_c2 = K2_inv * p2;
                        Eigen::Vector3f Pw;
                        bool bTriangulateSuccess = CommonTools::Triangulate(x_c1, x_c2,
                                         Tc1w , Tc2w ,
                                         Pw);

                        Eigen::Vector3f Pc1 = Rc1w * Pw + tc1w;
                        Eigen::Vector3f Pc2 = Rc2w * Pw + tc2w;
//                        cout << " Pc1 " << Pc1.transpose() << endl;
//                        cout << " Pc2 " << Pc2.transpose() << endl;
                        if (bTriangulateSuccess &&
                            Pc1.z() > 0 && Pc2.z() > 0 &&
                            Pc1.z() < DistTH && Pc2.z() < DistTH) {
                            //success
                            // kpi1
//                            num_matches++;
//                            pKF2->mvMapPointPositionVisual.push_back(Pw);
//                            cout << " Pw " << Pw.transpose() << endl;
//                            if (pKF1->mvMatchResult[cam_1][kpi_1] >= 0) {
//                                //left matches
//                                pKF2->mvMatchResult[cam_2][kpi_2] = pKF1->mvMatchResult[cam_1][kpi_1];
//                            } else if (pKF2->mvMatchResult[cam_2][kpi_2] >= 0) {
//                                //right matches
//                                pKF1->mvMatchResult[cam_1][kpi_1] = pKF2->mvMatchResult[cam_2][kpi_2];
//                            } else {
//                                //nomatches
//
//                                pKF1->mvMatchResult[cam_1][kpi_1] = num_matches;
//                                pKF2->mvMatchResult[cam_2][kpi_2] = num_matches;
//                                pKF1->mvMapPointPosition.push_back(CommonTools::T2R(pKF1->GetPose()) * Pw + CommonTools::T2t(pKF1->GetPose()));
//                                num_matches++;
//                            }

                            if (pKF2->mvMatchResult[nMaxKF - pKF1->mnId][cam_1][kpi_1] >= 0 && pKF2->mvMatchResult[nMaxKF - pKF2->mnId][cam_2][kpi_2] >= 0  ) {
                                if (pKF2->mvMatchResult[nMaxKF - pKF1->mnId][cam_1][kpi_1] != pKF2->mvMatchResult[nMaxKF - pKF2->mnId][cam_2][kpi_2] ) {
//                                    //i guess two point on one image match the same point on another point here
//                                    cout << "pKF2->mvMatchResult error" << endl;
//                                    cout << "pKF2->mvMatchResult[nMaxKF - pKF1->mnId][cam_1][kpi_1]" << pKF2->mvMatchResult[nMaxKF - pKF1->mnId][cam_1][kpi_1] << endl;
//                                    cout << "pKF2->mvMatchResult[nMaxKF - pKF2->mnId][cam_2][kpi_2]" << pKF2->mvMatchResult[nMaxKF - pKF2->mnId][cam_2][kpi_2] << endl;
                                } else {
                                    ////i guess two point on one image match the same point on another point in stereo
                                    //cout << "pKF2->mvMatchResult error 2" << endl;
                                    //cout << "Stereo num " << pKF2->mpFrame->mvMapPointPosition.size() << endl;
                                    //cout << "id " << pKF2->mvMatchResult[nMaxKF - pKF1->mnId][cam_1][kpi_1] << endl;
                                }
                                continue;
                            } else if (pKF2->mvMatchResult[nMaxKF - pKF1->mnId][cam_1][kpi_1] >= 0) {
                                //left matches
                                pKF2->mvMatchResult[nMaxKF - pKF2->mnId][cam_2][kpi_2] = pKF2->mvMatchResult[nMaxKF - pKF1->mnId][cam_1][kpi_1];
                            } else if (pKF2->mvMatchResult[nMaxKF - pKF2->mnId][cam_2][kpi_2] >= 0) {
                                //right matches
                                pKF2->mvMatchResult[nMaxKF - pKF1->mnId][cam_1][kpi_1] = pKF2->mvMatchResult[nMaxKF - pKF2->mnId][cam_2][kpi_2];
                            } else {
                                //nomatches
                                {
                                    unique_lock<mutex> lock(pKF2->mMutexMatch);
                                    pKF2->mvMatchResult[nMaxKF -
                                                        pKF1->mnId][cam_1][kpi_1] = pKF2->mvMapPointPosition.size();
                                    pKF2->mvMatchResult[nMaxKF -
                                                        pKF2->mnId][cam_2][kpi_2] = pKF2->mvMapPointPosition.size();
                                    pKF2->mvMapPointPosition.push_back(Pw);
                                }
                            }
//                            pKF2->mvMatchResult[nMaxKF - pKF1->mnId][cam_1][kpi_1] = pKF2->mvMapPointPosition.size();
//                            pKF2->mvMatchResult[nMaxKF - pKF2->mnId][cam_2][kpi_2] = pKF2->mvMapPointPosition.size();
//                            pKF2->mvMapPointPosition.push_back(Pw);
                            num_matches++;
                        }
                    }
                }
            }




        }

        cout << " N1 " << N1 << " N2 " << N2 << " num_matches " << num_matches << endl;




        return num_matches;
    }


    int FeatureMatcher::SearchByEpipolarGeometryMultiCamera(vector<KeyFrame *> vpLocalKF, int nMinKF, int nMaxKF) {
        //vpLocalKF[0] is the last pKF
        KeyFrame *pKF = vpLocalKF[0];

        for (int cam_i = 0; cam_i < pKF->mNumCam; cam_i++) {

        }
        /////////////////
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        /////////////////
        std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();


        vector<thread> vthreads;
        vthreads.reserve(pKF->mNumCam);
        for (int cam_i = 0; cam_i < pKF->mNumCam; cam_i++) {
            vthreads.emplace_back(&FeatureMatcher::SearchByEpipolarGeometryMultiCameraMultiThread,
                                  this, vpLocalKF, cam_i, nMinKF, nMaxKF);
        }


        /////////////////
        std::chrono::steady_clock::time_point t4 = std::chrono::steady_clock::now();
        cout << "vthreads.emplace_back use time : " << std::chrono::duration_cast<std::chrono::duration<double> >(t4 - t3).count() * 1000.0 << " ms." << endl;

        //usleep(10000);
        for (int cam_i = 0; cam_i < pKF->mNumCam; cam_i++) {
            vthreads[cam_i].join();
        }
        /////////////////
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        cout << "SearchByEpipolarGeometryMultiCamera use time : " << std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count() * 1000.0 << " ms." << endl;

    }


    int FeatureMatcher::SearchByEpipolarGeometryMultiCameraMultiThread(vector<KeyFrame *> vpLocalKF, int cam_i, int nMinKF, int nMaxKF) {
        //vpLocalKF[0] is the last pKF
        cout << "Debug SearchByEpipolarGeometryMultiCameraMultiThread 1" << endl;
        cout << "Debug vpLocalKF.size()" << vpLocalKF.size() << endl;
        KeyFrame *pKF = vpLocalKF[0];

        for (int KF_i = 1; KF_i < vpLocalKF.size(); KF_i++) {
            SearchByEpipolarGeometryBetweenImage(pKF, cam_i, vpLocalKF[KF_i], cam_i, nMinKF, nMaxKF);
        }

        return 0;
    }



//        //460ms
//        int N1 = CamKeysUn1.size();
//        int N2 = CamKeysUn2.size();
//        for (int kpi_2 = 0; kpi_2 < N2; kpi_2++) {
//            Eigen::Vector3f p2(CamKeysUn2[kpi_2].pt.x, CamKeysUn2[kpi_2].pt.y, 1.0);
//            Eigen::Matrix<float,1,3> line2 = p2.transpose() * F;
//            line2 = line2 / (line2.block<1, 2>(0, 0).norm());
//            float min_dis = FLT_MAX;
//            float max_dis = FLT_MIN;
//            for (int kpi_1 = 0; kpi_1 < N1; kpi_1++) {
//                Eigen::Vector3f p1(CamKeysUn1[kpi_1].pt.x, CamKeysUn1[kpi_1].pt.y, 1.0);
//                float dis = abs((line2 * p1)(0, 0));
//                min_dis = min(min_dis, dis);
//                max_dis = max(max_dis, dis);
//            }
//            cout << "min_dis " << min_dis << endl;
//            cout << "max_dis " << max_dis << endl;
//        }


//// Bit set count operation from
//// http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
//    int FeatureMatcher::DescriptorDistance(const cv::Mat &a, const cv::Mat &b)
//    {
//        const int *pa = a.ptr<int32_t>();
//        const int *pb = b.ptr<int32_t>();
//
//        int dist=0;
//
//        for(int i=0; i<8; i++, pa++, pb++)
//        {
//            unsigned  int v = *pa ^ *pb;
//            v = v - ((v >> 1) & 0x55555555);
//            v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
//            dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
//        }
//
//        return dist;
//    }



    int  FeatureMatcher::DescriptorDistance(const cv::Mat &a, const cv::Mat &b)
    {
//        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
//        /////////////////
        int dist=0;
        if (a.empty() || b.empty()) {
            return 256;
        }
        if (a.type() != b.type() || a.rows != b.rows || a.cols != b.cols) {
            return 256;
        }
        if (a.type() == CV_8U) {
            const int *pa = a.ptr<int32_t>();
            const int *pb = b.ptr<int32_t>();

            for(int i=0; i<8; i++, pa++, pb++)
            {
                unsigned  int v = *pa ^ *pb;
                v = v - ((v >> 1) & 0x55555555);
                v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
                dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
            }
        } else if (a.type() == CV_32FC1) {
            //-1, 1 th
            float score_eigen = a.dot(b);

            //score_eigen > 0.755 : dist_eigen < 0.49 : dist < 75            is good
            //score_eigen > 0.595 : dist_eigen < 0.81 : dist < 100            is almost good
            //score_eigen > 0.79 : dist_eigen < 0.42 : dist < 50             is pretty good
            dist = max(min(int( - 256.410 * score_eigen + 252.564), 256), 0);

            // 【修改开始】使用标准 L2 距离
            // XFeat 归一化后，理论最大 L2 距离是 2.0 (sqrt(2)*sqrt(2))
            // 我们乘以 100，把范围 [0, 2.0] 映射到整数 [0, 200]
            // float l2_dist = cv::norm(a, b, cv::NORM_L2);
            // dist = (int)(l2_dist * 60.0f); 
            
            

        }
//        /////////////////
//        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
//        cout << "match use time : " << std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count() * 1000.0 << " ms." << endl;
        return dist;
    }

//            for (int j = 0; j < 256; j+= 1) {
//                score_eigen += a.at<float>(0, j) * b.at<float>(0, j);
//            }

//            for (int j = 0; j < 256; j += 8) {
//                score_eigen +=  a.at<float>(0, j+0) * b.at<float>(0, j+0) + a.at<float>(0, j+1) * b.at<float>(0, j+1) + a.at<float>(0, j+2) * b.at<float>(0, j+2) + a.at<float>(0, j+3) * b.at<float>(0, j+3)
//                           +    a.at<float>(0, j+4) * b.at<float>(0, j+4) + a.at<float>(0, j+5) * b.at<float>(0, j+5) +a.at<float>(0, j+6) * b.at<float>(0, j+6) + a.at<float>(0, j+7) * b.at<float>(0, j+7);
//            }
//
//            const float *pa = a.ptr<float>();
//            const float *pb = b.ptr<float>();
//            for (int j = 0; j < 256; j += 8, pa+=8, pb+=8) {
//                score_eigen +=  (*(pa + 0)) * (*(pb + 0)) +
//                                (*(pa + 1)) * (*(pb + 1)) +
//                                (*(pa + 2)) * (*(pb + 2)) +
//                                (*(pa + 3)) * (*(pb + 3)) +
//                                (*(pa + 4)) * (*(pb + 4)) +
//                                (*(pa + 5)) * (*(pb + 5)) +
//                                (*(pa + 6)) * (*(pb + 6)) +
//                                (*(pa + 7)) * (*(pb + 7)) ;
//            }


//            //score_eigen > 0.755 : dist_eigen < 0.7 : dist < 75            is good
//            //score_eigen > 0.6 : dist_eigen < 0.90 : dist < 100            is almost good
//            //score_eigen > 0.8 : dist_eigen < 0.65 : dist < 50             is pretty good
//            //dist = 200.0 * dist_eigen - 80.0
//            float dist_eigen = sqrt(2 - 2 * score_eigen);
//            dist =  200.0 * dist_eigen - 80.0;
//            dist = max(min(dist, 256), 0);
    //score_eigen > 0.755 : dist_eigen < 0.49 : dist < 75            is good
    //score_eigen > 0.595 : dist_eigen < 0.81 : dist < 100            is almost good
    //score_eigen > 0.79 : dist_eigen < 0.42 : dist < 50             is pretty good
    // dist =  - 256.410 * score_eigen + 252.564
} //namespace ORB_SLAM
