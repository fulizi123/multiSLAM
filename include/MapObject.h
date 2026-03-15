#ifndef MAPOBJECT_H
#define MAPOBJECT_H

#include "CommonTools.h"
#include "ObjectObservation.h"

namespace LL_SLAM
{
    class KeyFrame;

    class MapObject {
    public:
        MapObject(const ObjectObservation &obs, const Eigen::Matrix4f &Twb);

        void AddObservation(KeyFrame *pKF, int object_idx);
        void EraseObservation(KeyFrame *pKF, int object_idx = -1);
        std::vector<KeyFrame*> GetKeyFrame();
        std::vector<pair<KeyFrame*, int>> GetObservations();

        int Observations();
        int KeyFrameObservations();

        void UpdateFromObservation(const ObjectObservation &obs, const Eigen::Matrix4f &Twb, bool update_pose = true);
        void SetWorldPose(const Eigen::Vector3f &position, const Eigen::Quaternionf &rotation);

        Eigen::Vector3f GetWorldPos() const;
        Eigen::Quaternionf GetWorldRotation() const;
        Eigen::Vector3f GetWorldVelocity() const;
        Eigen::Vector3f GetSize() const;

        int GetTrackId() const;
        int GetClassId() const;
        float GetScore() const;
        bool IsStatic() const;

        void SetBadFlag();
        bool isBad() const;

        long unsigned int mnId;
        static long unsigned int nNextId;

        int mTrackId = -1;
        int mClassId = -1;
        int nObs = 0;
        int nObsKF = 0;
        float mfScore = 0.0f;
        bool mbStatic = false;
        double mLastObservedTime = 0.0;

        std::multimap<KeyFrame*, int> mObservations;
        std::vector<pair<KeyFrame*, int>> mvObservations;
        std::vector<KeyFrame*> mvpKF;

        Eigen::Vector3f mWorldPos = Eigen::Vector3f::Zero();
        Eigen::Quaternionf mWorldRotation = Eigen::Quaternionf::Identity();
        Eigen::Vector3f mWorldVelocity = Eigen::Vector3f::Zero();
        Eigen::Vector3f mSize = Eigen::Vector3f::Zero();

        bool mbBad = false;
        mutable std::mutex mMutexFeatures;
        mutable std::mutex mMutexPose;
    };
}

#endif // MAPOBJECT_H
