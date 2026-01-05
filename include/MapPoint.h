

#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "CommonTools.h"
#include "KeyFrame.h"
namespace LL_SLAM
{
    class KeyFrame;

    class MapPoint{
    public:

        MapPoint(const Eigen::Vector3f &Pos);

        int Observations();

        void AddObservation(KeyFrame* pKF, int cam_i, int KeyPoint_i);

        void SetWorldPos(const Eigen::Vector3f &Pos);

        Eigen::Vector3f GetWorldPos();

        std::vector<KeyFrame*> GetKeyFrame();

        cv::Mat GetDescriptor();

        void UpdateDescriptor();

        long unsigned int mnId;
        static long unsigned int nNextId;
        int nObs;
        int nUpdate;
        cv::Mat mDescriptor;
        std::multimap<KeyFrame*,std::pair<int,int> > mObservations;
        std::vector<pair<KeyFrame*,std::pair<int,int>> > mvObservations;
        std::vector<KeyFrame*> mvpKF;
        Eigen::Vector3f mWorldPos;

        cv::Vec3b mColor;

        vector<vector<float>> mDistances;

    };

}

#endif // MAPPOINT_H
