

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
        int KeyFrameObservations();

        void AddObservation(KeyFrame* pKF, int cam_i, int KeyPoint_i);
        void EraseObservation(KeyFrame* pKF, int cam_i = -1, int KeyPoint_i = -1);

        void SetWorldPos(const Eigen::Vector3f &Pos);

        Eigen::Vector3f GetWorldPos();

        std::vector<KeyFrame*> GetKeyFrame();
        std::vector<pair<KeyFrame*,std::pair<int,int>>> GetObservations();

        cv::Mat GetDescriptor();

        void UpdateDescriptor();
        void SetBadFlag();
        bool isBad() const;

        long unsigned int mnId;
        static long unsigned int nNextId;
        int nObs;
        int nObsKF;
        int nUpdate;
        cv::Mat mDescriptor;
        std::multimap<KeyFrame*,std::pair<int,int> > mObservations;
        std::vector<pair<KeyFrame*,std::pair<int,int>> > mvObservations;
        std::vector<KeyFrame*> mvpKF;
        Eigen::Vector3f mWorldPos;

        cv::Vec3b mColor;

        vector<vector<float>> mDistances;
        bool mbBad = false;
        mutable std::mutex mMutexFeatures;
        mutable std::mutex mMutexPos;

    };

}

#endif // MAPPOINT_H
