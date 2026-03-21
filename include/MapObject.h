#ifndef MAPOBJECT_H
#define MAPOBJECT_H

#include "CommonTools.h"
#include "ObjectObservation.h"

namespace LL_SLAM
{
    class KeyFrame;

    class MapObject {
    public:
        enum ObjectState {
            OBJECT_STATE_STATIC = 0,
            OBJECT_STATE_DYNAMIC = 1,
            OBJECT_STATE_UNKNOWN = 2
        };

        enum SourceType {
            OBJECT_SOURCE_UNKNOWN = 0,
            OBJECT_SOURCE_OBJECT_BIN = 1,
            OBJECT_SOURCE_STREAMPETR = 2
        };

        MapObject(const ObjectObservation &obs,
                  const Eigen::Matrix4f &Twb,
                  double timestamp = 0.0,
                  long unsigned int keyframe_id = 0,
                  SourceType source_type = OBJECT_SOURCE_OBJECT_BIN);

        void AddObservation(KeyFrame *pKF, int object_idx);
        void EraseObservation(KeyFrame *pKF, int object_idx = -1);
        std::vector<KeyFrame*> GetKeyFrame();
        std::vector<pair<KeyFrame*, int>> GetObservations();

        int Observations();
        int KeyFrameObservations();

        void UpdateFromObservation(const ObjectObservation &obs,
                                   const Eigen::Matrix4f &Twb,
                                   double timestamp,
                                   long unsigned int keyframe_id,
                                   SourceType source_type = OBJECT_SOURCE_OBJECT_BIN,
                                   bool update_pose = true);
        void MarkMissed(long unsigned int current_keyframe_id, double current_timestamp);
        void SetWorldPose(const Eigen::Vector3f &position, const Eigen::Quaternionf &rotation);

        Eigen::Vector3f GetWorldPos() const;
        Eigen::Quaternionf GetWorldRotation() const;
        Eigen::Vector3f GetWorldVelocity() const;
        Eigen::Vector3f GetSize() const;

        int GetTrackId() const;
        int GetClassId() const;
        float GetScore() const;
        float GetConfidence() const;
        int GetSeenCount() const;
        int GetLostCount() const;
        int GetAge() const;
        int GetStaticObservationCount() const;
        int GetDynamicObservationCount() const;
        long unsigned int GetFirstObservedKFId() const;
        long unsigned int GetLastObservedKFId() const;
        ObjectState GetState() const;
        SourceType GetSourceType() const;
        bool IsStatic() const;
        bool IsDynamic() const;
        bool IsUnknown() const;

        void SetBadFlag();
        bool isBad() const;

        long unsigned int mnId;
        static long unsigned int nNextId;

        int mTrackId = -1;
        int mClassId = -1;
        int nObs = 0;
        int nObsKF = 0;
        float mfScore = 0.0f;
        float mfConfidence = 0.0f;
        ObjectState mState = OBJECT_STATE_UNKNOWN;
        SourceType mSourceType = OBJECT_SOURCE_UNKNOWN;
        double mFirstObservedTime = 0.0;
        double mLastObservedTime = 0.0;
        long unsigned int mnFirstObservedKFId = 0;
        long unsigned int mnLastObservedKFId = 0;
        long unsigned int mnLastLifecycleKFId = 0;
        int mnSeenCount = 0;
        int mnLostCount = 0;
        int mnAge = 0;
        int mnStaticObservationCount = 0;
        int mnDynamicObservationCount = 0;

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
