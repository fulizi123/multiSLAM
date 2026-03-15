

#ifndef MAP_H
#define MAP_H

#include "CommonTools.h"
#include "System.h"
#include "KeyFrame.h"
#include "MapPoint.h"
#include "MapObject.h"

using namespace std;
namespace LL_SLAM
{
    class System;
    class KeyFrame;
    class MapPoint;
    class MapObject;

    class Map{
    public:
        Map(System *pSystem);

        void AddKeyFrame(KeyFrame *pKF);

        void AddMapPoint(MapPoint *pMP);
        void AddMapObject(MapObject *pObj);

        KeyFrame *GetLastKeyFrame();
        MapObject *GetMapObjectByTrackId(int track_id);

        void UpdateLocalMap(KeyFrame *pReferenceKF = nullptr);

        vector<KeyFrame*> GetLocalKeyFrame();

        vector<MapPoint*> GetLocalMapPoint();
        vector<MapObject*> GetLocalMapObject();

        System* mpSystem;



        vector<MapPoint*> mvpMPObservations;
        std::set<MapPoint*> mspMPObservations;
        vector<MapObject*> mvpObjectObservations;
        std::set<MapObject*> mspObjectObservations;
        std::unordered_map<int, MapObject*> mTrackId2Object;

        vector<KeyFrame*> mvpKFObservations;
        std::set<KeyFrame*> mspKFObservations;

        //LocalMap
        vector<MapPoint*> mvpLocalMP;
        vector<KeyFrame*> mvpLocalKF;
        vector<MapObject*> mvpLocalObject;

        std::mutex mMutexUpdate;

    };

}
#endif // MAP_H
