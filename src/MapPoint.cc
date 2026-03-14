
#include "MapPoint.h"
#include <algorithm>


namespace LL_SLAM
{
    long unsigned int MapPoint::nNextId=0;

    namespace
    {
        bool MatchObservation(const pair<KeyFrame*, std::pair<int,int>> &obs, KeyFrame* pKF, int cam_i, int KeyPoint_i)
        {
            if (obs.first != pKF) {
                return false;
            }
            if (cam_i >= 0 && obs.second.first != cam_i) {
                return false;
            }
            if (KeyPoint_i >= 0 && obs.second.second != KeyPoint_i) {
                return false;
            }
            return true;
        }
    }

    MapPoint::MapPoint(const Eigen::Vector3f &Pos){
        mWorldPos = Pos;

        mnId=nNextId++;
        nObs = 0;
        nObsKF = 0;
        nUpdate = 0;
    }

    void MapPoint::SetWorldPos(const Eigen::Vector3f &Pos) {
        unique_lock<mutex> lock(mMutexPos);
        mWorldPos = Pos;
    }

    Eigen::Vector3f MapPoint::GetWorldPos() {
        unique_lock<mutex> lock(mMutexPos);
        return mWorldPos;
    }

    int MapPoint::Observations()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return nObs;
    }

    int MapPoint::KeyFrameObservations()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return nObsKF;
    }

    std::vector<KeyFrame*> MapPoint::GetKeyFrame() {
        unique_lock<mutex> lock(mMutexFeatures);
        vector<KeyFrame*> vpKFs;
        vpKFs.reserve(mvpKF.size());
        for (KeyFrame *pKF : mvpKF) {
            if (pKF == nullptr || pKF->isBad()) {
                continue;
            }
            vpKFs.push_back(pKF);
        }
        return vpKFs;
    }

    std::vector<pair<KeyFrame*,std::pair<int,int>>> MapPoint::GetObservations() {
        unique_lock<mutex> lock(mMutexFeatures);
        return vector<pair<KeyFrame*,std::pair<int,int>>>(mvObservations.begin(), mvObservations.end());
    }

    cv::Mat MapPoint::GetDescriptor(){
        unique_lock<mutex> lock(mMutexFeatures);
        return mDescriptor;
    }

    void MapPoint::AddObservation(KeyFrame* pKF, int cam_i, int KeyPoint_i)
    {
        bool needDescriptorUpdate = false;
        {
            unique_lock<mutex> lock(mMutexFeatures);
            if (mbBad || pKF == nullptr || pKF->isBad()) {
                return;
            }

            auto range = mObservations.equal_range(pKF);
            for (auto it = range.first; it != range.second; ++it) {
                if (it->second.first == cam_i && it->second.second == KeyPoint_i) {
                    return;
                }
            }

            bool hasKF = range.first != range.second;
            mObservations.insert(pair<KeyFrame*, std::pair<int,int> >(pKF, pair<int, int>(cam_i, KeyPoint_i)));
            mvObservations.push_back(pair<KeyFrame*, std::pair<int,int> >(pKF, pair<int, int>(cam_i, KeyPoint_i)));

            if (!hasKF) {
                mvpKF.push_back(pKF);
                nObsKF++;
            }

            nObs++;
            needDescriptorUpdate = true;
            mColor = pKF->mvColor[cam_i][KeyPoint_i];
        }

        if (needDescriptorUpdate) {
            UpdateDescriptor();
        }
    }

    void MapPoint::EraseObservation(KeyFrame* pKF, int cam_i, int KeyPoint_i)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        if (pKF == nullptr) {
            return;
        }

        bool removed = false;
        for (auto it = mObservations.begin(); it != mObservations.end(); ) {
            if (it->first == pKF &&
                (cam_i < 0 || it->second.first == cam_i) &&
                (KeyPoint_i < 0 || it->second.second == KeyPoint_i)) {
                it = mObservations.erase(it);
                removed = true;
                nObs = max(0, nObs - 1);
            } else {
                ++it;
            }
        }

        if (!removed) {
            return;
        }

        mvObservations.erase(
            remove_if(mvObservations.begin(), mvObservations.end(),
                      [pKF, cam_i, KeyPoint_i](const pair<KeyFrame*, std::pair<int,int>> &obs) {
                          return MatchObservation(obs, pKF, cam_i, KeyPoint_i);
                      }),
            mvObservations.end());

        bool hasKF = false;
        auto range = mObservations.equal_range(pKF);
        hasKF = range.first != range.second;
        if (!hasKF) {
            mvpKF.erase(remove(mvpKF.begin(), mvpKF.end(), pKF), mvpKF.end());
            nObsKF = max(0, nObsKF - 1);
        }

        if (mObservations.empty()) {
            mbBad = true;
        }
    }


    void MapPoint::UpdateDescriptor() {
        unique_lock<mutex> lock(mMutexFeatures);
        if (mbBad) {
            return;
        }

        if (!mDescriptor.empty() && nObs - nUpdate < 10) {
            return;
        }
        nUpdate = nObs;

        //mDescriptor
        std::vector<pair<KeyFrame*,std::pair<int,int>> >  observations = mvObservations ;
        vector<cv::Mat> vDescriptors;

        vDescriptors.reserve(observations.size());
        for(vector<pair<KeyFrame*,std::pair<int,int>> > ::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFForDesc = mit->first;
            int cam_iForDesc = mit->second.first; int KeyPoint_iForDesc = mit -> second.second;
            vDescriptors.push_back(pKFForDesc->mvCamDescriptors[cam_iForDesc].row(KeyPoint_iForDesc));
        }


        if(vDescriptors.empty())
            return;

        if (vDescriptors.size() > mDistances.size()) {
            if (mDistances.size() == 0) {
               mDistances = vector<vector<float>>(50, vector<float>(50, -1));
            } else {
                vector<vector<float>> Distances(vDescriptors.size()*2, vector<float>(vDescriptors.size()*2, -1));
                for (int i = 0; i < mDistances.size(); i++) {
                    for (int j = 0; j < mDistances[i].size(); j++) {
                        Distances[i][j] = mDistances[i][j];
                    }
                }
                mDistances = Distances;
                //swap(mDistances, Distances);
                if (vDescriptors.size() > mDistances.size()) {
                    cout << "vDescriptors error 1" << endl;
                }
            }
        }

        if (vDescriptors.size() > mDistances.size()) {
            cout << "vDescriptors error 2" << endl;
        }
        // Compute distances between them
        const size_t N = vDescriptors.size();

        for(size_t i=0;i<N;i++)
        {
            mDistances[i][i]=0;
            for(size_t j=i+1;j<N;j++)
            {
                if (mDistances[i][j] >= 0) {continue;}
                int distij = FeatureMatcher::DescriptorDistance(vDescriptors[i],vDescriptors[j]);
                mDistances[i][j]=distij;
                mDistances[j][i]=distij;
            }
        }

        // Take the descriptor with least median distance to the rest
        int BestMedian = INT_MAX;
        int BestIdx = 0;
        for(size_t i=0;i<N;i++)
        {
            vector<int> vDists;
            for (int j = 0; j < mDistances[i].size(); j++) {
                if (mDistances[i][j] == -1) {continue;}
                vDists.push_back(mDistances[i][j]);
            }

            sort(vDists.begin(),vDists.end());
            int median = vDists[0.5*(N-1)];

            if(median<BestMedian)
            {
                BestMedian = median;
                BestIdx = i;
            }
        }

        mDescriptor = vDescriptors[BestIdx].clone();
    }

    void MapPoint::SetBadFlag()
    {
        vector<pair<KeyFrame*,std::pair<int,int>>> vObservations;
        {
            unique_lock<mutex> lock(mMutexFeatures);
            if (mbBad) {
                return;
            }
            mbBad = true;
            vObservations = mvObservations;
            nObs = 0;
            nObsKF = 0;
            mObservations.clear();
            mvObservations.clear();
            mvpKF.clear();
            mDescriptor.release();
        }

        for (const auto &obs : vObservations) {
            KeyFrame *pKF = obs.first;
            if (pKF == nullptr || pKF->isBad()) {
                continue;
            }
            pKF->EraseObservation(obs.second.first, obs.second.second);
        }
    }

    bool MapPoint::isBad() const
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return mbBad;
    }



} //namespace ORB_SLAM
