

#ifndef MAP_H
#define MAP_H

#include "CommonTools.h"
#include "System.h"
#include "KeyFrame.h"
#include "MapPoint.h"

using namespace std;
namespace LL_SLAM
{
    class System;
    class KeyFrame;
    class MapPoint;

    class Map{
    public:
        Map(System *pSystem);

        void AddKeyFrame(KeyFrame *pKF);

        void AddMapPoint(MapPoint *pMP);

        KeyFrame *GetLastKeyFrame();

        void UpdateLocalMap(KeyFrame *pReferenceKF = nullptr);

        vector<KeyFrame*> GetLocalKeyFrame();

        vector<MapPoint*> GetLocalMapPoint();

        System* mpSystem;



        vector<MapPoint*> mvpMPObservations;
        std::set<MapPoint*> mspMPObservations;

        vector<KeyFrame*> mvpKFObservations;
        std::set<KeyFrame*> mspKFObservations;

        //LocalMap
        vector<MapPoint*> mvpLocalMP;
        vector<KeyFrame*> mvpLocalKF;

        std::mutex mMutexUpdate;

    };

}
#endif // MAP_H
