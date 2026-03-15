
#include "Map.h"
#include <algorithm>


namespace LL_SLAM
{
    Map::Map(System *pSystem) {
        mvpMPObservations.reserve(1e6);
        mvpKFObservations.reserve(1e4);
        mvpObjectObservations.reserve(1e5);
    }

    void Map::AddKeyFrame(KeyFrame *pKF) {
        unique_lock<mutex> lock(mMutexUpdate);
        mvpKFObservations.push_back(pKF);
        mspKFObservations.insert(pKF);

        return ;
    }


    void Map::AddMapPoint(MapPoint *pMP) {
        unique_lock<mutex> lock(mMutexUpdate);
        mvpMPObservations.push_back(pMP);
        mspMPObservations.insert(pMP);

        return ;
    }

    void Map::AddMapObject(MapObject *pObj) {
        unique_lock<mutex> lock(mMutexUpdate);
        mvpObjectObservations.push_back(pObj);
        mspObjectObservations.insert(pObj);
        mTrackId2Object[pObj->GetTrackId()] = pObj;
    }



    KeyFrame* Map::GetLastKeyFrame(){
        unique_lock<mutex> lock(mMutexUpdate);
        if (mvpKFObservations.empty()) {return NULL;}
        return mvpKFObservations.back();
    }

    MapObject *Map::GetMapObjectByTrackId(int track_id) {
        unique_lock<mutex> lock(mMutexUpdate);
        auto it = mTrackId2Object.find(track_id);
        if (it == mTrackId2Object.end()) {
            return nullptr;
        }
        if (it->second == nullptr || it->second->isBad()) {
            return nullptr;
        }
        return it->second;
    }

    void Map::UpdateLocalMap(KeyFrame *pReferenceKF) {
        unique_lock<mutex> lock(mMutexUpdate);
        mvpLocalKF.clear();
        mvpLocalMP.clear();
        mvpLocalObject.clear();
        if (pReferenceKF == nullptr) {
            if (!mvpKFObservations.empty()) {
                pReferenceKF = mvpKFObservations.back();
            }
        }
        if (pReferenceKF == nullptr) {
            return;
        }

        const int nMaxCovisibleKF = 10;
        const int nMinLocalKF = 5;

        vector<KeyFrame*> vCandidates;
        vCandidates.push_back(pReferenceKF);
        vector<KeyFrame*> vBestCovisibility = pReferenceKF->GetBestCovisibilityKeyFrames(nMaxCovisibleKF);
        vCandidates.insert(vCandidates.end(), vBestCovisibility.begin(), vBestCovisibility.end());

        unordered_set<KeyFrame*> usLocalKF;
        usLocalKF.reserve(vCandidates.size() + nMinLocalKF);
        for (KeyFrame *pKF : vCandidates) {
            if (pKF == nullptr || pKF->isBad()) {
                continue;
            }
            usLocalKF.insert(pKF);
        }

        for (int KFi = int(mvpKFObservations.size()) - 1; KFi >= 0 && usLocalKF.size() < nMinLocalKF; KFi--) {
            KeyFrame *pKF = mvpKFObservations[KFi];
            if (pKF == nullptr || pKF->isBad()) {
                continue;
            }
            usLocalKF.insert(pKF);
        }

        mvpLocalKF.reserve(usLocalKF.size());
        for (KeyFrame *pKF : usLocalKF) {
            mvpLocalKF.push_back(pKF);
        }
        sort(mvpLocalKF.begin(), mvpLocalKF.end(),
             [](KeyFrame *lhs, KeyFrame *rhs) { return lhs->mnId > rhs->mnId; });

        unordered_set<MapPoint*> umMapPoints;
        for (int KFi = 0; KFi < mvpLocalKF.size(); KFi++) {
            KeyFrame * pKF = mvpLocalKF[KFi];

            for (int cam_i = 0; cam_i < pKF->mvMapPoints.size(); cam_i++) {
                for (int kpi = 0; kpi < pKF->mvMapPoints[cam_i].size(); kpi++) {
                    MapPoint *pMP = pKF->mvMapPoints[cam_i][kpi];
                    if (pMP == NULL || pMP->isBad()) {
                        continue;
                    }
                    umMapPoints.insert(pMP);
                }
            }
        }

        mvpLocalMP.reserve(umMapPoints.size());
        for (auto it : umMapPoints) {
            mvpLocalMP.push_back(it);
        }

        unordered_set<MapObject*> umObjects;
        for (KeyFrame *pKF : mvpLocalKF) {
            for (MapObject *pObj : pKF->mvMapObjects) {
                if (pObj == nullptr || pObj->isBad()) {
                    continue;
                }
                umObjects.insert(pObj);
            }
        }
        mvpLocalObject.reserve(umObjects.size());
        for (auto it : umObjects) {
            mvpLocalObject.push_back(it);
        }
        return ;
    }

    vector<KeyFrame*> Map::GetLocalKeyFrame() {
        unique_lock<mutex> lock(mMutexUpdate);
        return vector<KeyFrame*>(mvpLocalKF.begin(),mvpLocalKF.end());
    }

    vector<MapPoint*> Map::GetLocalMapPoint() {
        unique_lock<mutex> lock(mMutexUpdate);
        return vector<MapPoint*>(mvpLocalMP.begin(),mvpLocalMP.end());
    }

    vector<MapObject*> Map::GetLocalMapObject() {
        unique_lock<mutex> lock(mMutexUpdate);
        return vector<MapObject*>(mvpLocalObject.begin(), mvpLocalObject.end());
    }

} //namespace ORB_SLAM
