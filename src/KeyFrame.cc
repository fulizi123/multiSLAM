
#include "KeyFrame.h"
#include <algorithm>


namespace LL_SLAM
{
    long unsigned int KeyFrame::nNextId=0;

    namespace
    {
        bool SortByWeightDesc(const pair<int, KeyFrame*> &lhs, const pair<int, KeyFrame*> &rhs)
        {
            if (lhs.first != rhs.first) {
                return lhs.first > rhs.first;
            }
            return lhs.second->mnId > rhs.second->mnId;
        }
    }

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
        unique_lock<mutex> lock(mMutexMatch);
        mvMapPoints[cam_i][KeyPoint_i] = pMP;
    }

    void KeyFrame::EraseObservation(int cam_i, int KeyPoint_i) {
        unique_lock<mutex> lock(mMutexMatch);
        if (cam_i < 0 || cam_i >= mvMapPoints.size()) {
            return;
        }
        if (KeyPoint_i < 0 || KeyPoint_i >= mvMapPoints[cam_i].size()) {
            return;
        }
        mvMapPoints[cam_i][KeyPoint_i] = nullptr;
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

    void KeyFrame::AddConnection(KeyFrame *pKF, int weight) {
        if (pKF == nullptr || pKF == this || weight <= 0) {
            return;
        }
        unique_lock<mutex> lock(mMutexConnections);
        mConnectedKeyFrameWeights[pKF] = weight;

        vector<pair<int, KeyFrame*>> vPairs;
        vPairs.reserve(mConnectedKeyFrameWeights.size());
        for (auto &it : mConnectedKeyFrameWeights) {
            if (it.first == nullptr || it.first->isBad()) {
                continue;
            }
            vPairs.push_back({it.second, it.first});
        }
        sort(vPairs.begin(), vPairs.end(), SortByWeightDesc);

        mvpOrderedConnectedKeyFrames.clear();
        mvOrderedWeights.clear();
        mvpOrderedConnectedKeyFrames.reserve(vPairs.size());
        mvOrderedWeights.reserve(vPairs.size());
        for (auto &it : vPairs) {
            mvOrderedWeights.push_back(it.first);
            mvpOrderedConnectedKeyFrames.push_back(it.second);
        }
    }

    void KeyFrame::EraseConnection(KeyFrame *pKF) {
        if (pKF == nullptr) {
            return;
        }
        unique_lock<mutex> lock(mMutexConnections);
        mConnectedKeyFrameWeights.erase(pKF);
        mvpOrderedConnectedKeyFrames.erase(
            remove(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.end(), pKF),
            mvpOrderedConnectedKeyFrames.end());
        mvOrderedWeights.clear();
        mvOrderedWeights.reserve(mvpOrderedConnectedKeyFrames.size());
        for (KeyFrame *pConnected : mvpOrderedConnectedKeyFrames) {
            auto it = mConnectedKeyFrameWeights.find(pConnected);
            if (it != mConnectedKeyFrameWeights.end()) {
                mvOrderedWeights.push_back(it->second);
            }
        }
    }

    void KeyFrame::UpdateConnections() {
        map<KeyFrame*, int> KFcounter;
        {
            unique_lock<mutex> lock(mMutexMatch);
            for (int cam_i = 0; cam_i < mvMapPoints.size(); cam_i++) {
                for (int kpi = 0; kpi < mvMapPoints[cam_i].size(); kpi++) {
                    MapPoint *pMP = mvMapPoints[cam_i][kpi];
                    if (pMP == nullptr || pMP->isBad()) {
                        continue;
                    }
                    vector<KeyFrame*> vpKFs = pMP->GetKeyFrame();
                    for (KeyFrame *pKF : vpKFs) {
                        if (pKF == nullptr || pKF == this || pKF->isBad()) {
                            continue;
                        }
                        KFcounter[pKF]++;
                    }
                }
            }
        }

        if (KFcounter.empty()) {
            return;
        }

        const int kMinSharedMapPoints = 15;
        int bestWeight = 0;
        KeyFrame *pBestKF = nullptr;
        map<KeyFrame*, int> newConnections;
        for (auto &it : KFcounter) {
            if (it.second > bestWeight) {
                bestWeight = it.second;
                pBestKF = it.first;
            }
            if (it.second >= kMinSharedMapPoints) {
                newConnections[it.first] = it.second;
            }
        }
        if (newConnections.empty() && pBestKF != nullptr) {
            newConnections[pBestKF] = bestWeight;
        }

        {
            unique_lock<mutex> lock(mMutexConnections);
            mConnectedKeyFrameWeights = newConnections;
            vector<pair<int, KeyFrame*>> vPairs;
            vPairs.reserve(mConnectedKeyFrameWeights.size());
            for (auto &it : mConnectedKeyFrameWeights) {
                vPairs.push_back({it.second, it.first});
            }
            sort(vPairs.begin(), vPairs.end(), SortByWeightDesc);

            mvpOrderedConnectedKeyFrames.clear();
            mvOrderedWeights.clear();
            mvpOrderedConnectedKeyFrames.reserve(vPairs.size());
            mvOrderedWeights.reserve(vPairs.size());
            for (auto &it : vPairs) {
                mvOrderedWeights.push_back(it.first);
                mvpOrderedConnectedKeyFrames.push_back(it.second);
            }
        }

        for (auto &it : newConnections) {
            it.first->AddConnection(this, it.second);
        }
    }

    vector<KeyFrame*> KeyFrame::GetConnectedKeyFrames() {
        unique_lock<mutex> lock(mMutexConnections);
        return vector<KeyFrame*>(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.end());
    }

    vector<KeyFrame*> KeyFrame::GetBestCovisibilityKeyFrames(int N) {
        unique_lock<mutex> lock(mMutexConnections);
        if (N <= 0 || mvpOrderedConnectedKeyFrames.empty()) {
            return {};
        }
        int count = min(N, int(mvpOrderedConnectedKeyFrames.size()));
        return vector<KeyFrame*>(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin() + count);
    }

    int KeyFrame::GetWeight(KeyFrame *pKF) {
        unique_lock<mutex> lock(mMutexConnections);
        auto it = mConnectedKeyFrameWeights.find(pKF);
        if (it == mConnectedKeyFrameWeights.end()) {
            return 0;
        }
        return it->second;
    }

    void KeyFrame::SetBadFlag() {
        if (mnId == 0) {
            return;
        }

        vector<pair<MapPoint*, pair<int, int>>> vObservations;
        vector<KeyFrame*> vConnected;
        {
            unique_lock<mutex> lockConn(mMutexConnections);
            if (mbBad) {
                return;
            }
            mbBad = true;
            for (auto &it : mConnectedKeyFrameWeights) {
                if (it.first != nullptr) {
                    vConnected.push_back(it.first);
                }
            }
            mConnectedKeyFrameWeights.clear();
            mvpOrderedConnectedKeyFrames.clear();
            mvOrderedWeights.clear();
        }
        {
            unique_lock<mutex> lockMatch(mMutexMatch);
            for (int cam_i = 0; cam_i < int(mvMapPoints.size()); cam_i++) {
                for (int kpi = 0; kpi < int(mvMapPoints[cam_i].size()); kpi++) {
                    MapPoint *pMP = mvMapPoints[cam_i][kpi];
                    if (pMP == nullptr) {
                        continue;
                    }
                    vObservations.push_back({pMP, {cam_i, kpi}});
                    mvMapPoints[cam_i][kpi] = nullptr;
                }
            }
        }

        for (const auto &obs : vObservations) {
            if (obs.first == nullptr || obs.first->isBad()) {
                continue;
            }
            obs.first->EraseObservation(this, obs.second.first, obs.second.second);
        }

        for (KeyFrame *pConnectedKF : vConnected) {
            if (pConnectedKF == nullptr || pConnectedKF == this) {
                continue;
            }
            pConnectedKF->EraseConnection(this);
        }
    }

    bool KeyFrame::isBad() const {
        unique_lock<mutex> lock(mMutexConnections);
        return mbBad;
    }
} //namespace ORB_SLAM
