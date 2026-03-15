#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H


#include "CommonTools.h"
#include "System.h"
#include "Frame.h"
#include "KeyFrame.h"
#include "Map.h"
#include "Tracking.h"
#include "Viewer.h"
#include "Optimizer.h"

namespace LL_SLAM
{
    struct MapObjectUpdateStats {
        int current_static_obs = 0;
        int new_mapobject = 0;
        int updated_mapobject = 0;
        int global_static_mapobject = 0;
    };

    class System;
    class Frame;
    class KeyFrame;
    class Map;
    class Tracking;
    class Viewer;
    class Optimizer;

    class LocalMapping{
    public:
        LocalMapping(System *pSystem);

        void Run();

        void InsertFrame(Frame *pCurrentFrame) ;

        void CreateNewKeyFrame(Frame *pCurrentFrame) ;
        MapObjectUpdateStats CreateOrUpdateMapObjects(KeyFrame *pCurrentKF);
        void MapPointCulling(KeyFrame *pCurrentKF);
        void KeyFrameCulling(KeyFrame *pCurrentKF);


        vector<Frame *> mvpFrame;
        std::mutex mMutexMsg;

        System *mpSystem;
        Tracking *mpTracker;
        Map *mpMap;
    };


}
#endif // LOCALMAPPING_H
