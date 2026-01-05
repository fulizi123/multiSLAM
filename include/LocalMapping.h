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


        vector<Frame *> mvpFrame;
        std::mutex mMutexMsg;

        System *mpSystem;
        Tracking *mpTracker;
        Map *mpMap;
    };


}
#endif // LOCALMAPPING_H
