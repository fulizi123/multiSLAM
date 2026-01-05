
#include "Map.h"


namespace LL_SLAM
{
    Map::Map(System *pSystem) {
        mvpMPObservations.reserve(1e6);
        mvpKFObservations.reserve(1e4);
    }

    void Map::AddKeyFrame(KeyFrame *pKF) {
        mvpKFObservations.push_back(pKF);
        mspKFObservations.insert(pKF);

        return ;
    }


    void Map::AddMapPoint(MapPoint *pMP) {
        mvpMPObservations.push_back(pMP);
        mspMPObservations.insert(pMP);

        return ;
    }



    KeyFrame* Map::GetLastKeyFrame(){
        if (mvpKFObservations.empty()) {return NULL;}
        return mvpKFObservations.back();
    }

    void Map::UpdateLocalMap() {

//        mvpLocalKF = mvpKFObservations;
//        mvpLocalMP = mvpMPObservations;

        mvpLocalKF.clear();
        mvpLocalMP.clear();
        mvpLocalKF.reserve(5);
        int nMaxLocalKF = 5;
//        for (int KFi = max(int(mvpKFObservations.size() - nMaxLocalKF), 0); KFi < mvpKFObservations.size(); KFi++) {
//            mvpLocalKF.push_back(mvpKFObservations[KFi]);
//        }
        for (int KFi = mvpKFObservations.size() - 1; KFi >= max(int(mvpKFObservations.size() - nMaxLocalKF), 0); KFi--) {
            mvpLocalKF.push_back(mvpKFObservations[KFi]);
        }

        unordered_set<MapPoint*> umMapPoints;
        for (int KFi = 0; KFi < mvpLocalKF.size(); KFi++) {
            KeyFrame * pKF = mvpLocalKF[KFi];

            for (int cam_i = 0; cam_i < pKF->mvMapPoints.size(); cam_i++) {
                for (int kpi = 0; kpi < pKF->mvMapPoints[cam_i].size(); kpi++) {
                    if (pKF->mvMapPoints[cam_i][kpi] == NULL) {
                        continue;
                    }
                    umMapPoints.insert(pKF->mvMapPoints[cam_i][kpi]);
                }
            }
        }

        mvpLocalMP.reserve(umMapPoints.size());
        for (auto it : umMapPoints) {
            mvpLocalMP.push_back(it);
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

} //namespace ORB_SLAM
