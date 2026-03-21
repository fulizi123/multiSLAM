
#include "MapObject.h"

#include "KeyFrame.h"

#include <algorithm>

namespace LL_SLAM
{
    namespace
    {
        bool MatchObjectObservation(const pair<KeyFrame*, int> &obs, KeyFrame *pKF, int object_idx)
        {
            if (obs.first != pKF) {
                return false;
            }
            if (object_idx >= 0 && obs.second != object_idx) {
                return false;
            }
            return true;
        }

        MapObject::ObjectState ResolveObjectState(int static_count, int dynamic_count)
        {
            if (static_count > 0 && dynamic_count == 0) {
                return MapObject::OBJECT_STATE_STATIC;
            }
            if (dynamic_count > 0 && static_count == 0) {
                return MapObject::OBJECT_STATE_DYNAMIC;
            }
            return MapObject::OBJECT_STATE_UNKNOWN;
        }
    }

    long unsigned int MapObject::nNextId = 0;

    MapObject::MapObject(const ObjectObservation &obs,
                         const Eigen::Matrix4f &Twb,
                         double timestamp,
                         long unsigned int keyframe_id,
                         SourceType source_type)
    {
        mnId = nNextId++;
        UpdateFromObservation(obs, Twb, timestamp, keyframe_id, source_type);
    }

    void MapObject::AddObservation(KeyFrame *pKF, int object_idx)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        if (mbBad || pKF == nullptr || pKF->isBad()) {
            return;
        }

        auto range = mObservations.equal_range(pKF);
        for (auto it = range.first; it != range.second; ++it) {
            if (it->second == object_idx) {
                return;
            }
        }

        const bool hasKF = range.first != range.second;
        mObservations.insert({pKF, object_idx});
        mvObservations.push_back({pKF, object_idx});
        if (!hasKF) {
            mvpKF.push_back(pKF);
            nObsKF++;
        }
        nObs++;
    }

    void MapObject::EraseObservation(KeyFrame *pKF, int object_idx)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        if (pKF == nullptr) {
            return;
        }

        bool removed = false;
        for (auto it = mObservations.begin(); it != mObservations.end(); ) {
            if (it->first == pKF && (object_idx < 0 || it->second == object_idx)) {
                it = mObservations.erase(it);
                removed = true;
                nObs = max(0, nObs - 1);
            } else {
                ++it;
            }
        }

        if (!removed) {
            return;
        }

        mvObservations.erase(
            remove_if(mvObservations.begin(), mvObservations.end(),
                      [pKF, object_idx](const pair<KeyFrame*, int> &obs) {
                          return MatchObjectObservation(obs, pKF, object_idx);
                      }),
            mvObservations.end());

        auto range = mObservations.equal_range(pKF);
        if (range.first == range.second) {
            mvpKF.erase(remove(mvpKF.begin(), mvpKF.end(), pKF), mvpKF.end());
            nObsKF = max(0, nObsKF - 1);
        }
        if (mObservations.empty()) {
            mbBad = true;
        }
    }

    std::vector<KeyFrame*> MapObject::GetKeyFrame()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        vector<KeyFrame*> vpKFs;
        vpKFs.reserve(mvpKF.size());
        for (KeyFrame *pKF : mvpKF) {
            if (pKF == nullptr || pKF->isBad()) {
                continue;
            }
            vpKFs.push_back(pKF);
        }
        return vpKFs;
    }

    std::vector<pair<KeyFrame*, int>> MapObject::GetObservations()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return vector<pair<KeyFrame*, int>>(mvObservations.begin(), mvObservations.end());
    }

    int MapObject::Observations()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return nObs;
    }

    int MapObject::KeyFrameObservations()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return nObsKF;
    }

    void MapObject::UpdateFromObservation(const ObjectObservation &obs,
                                          const Eigen::Matrix4f &Twb,
                                          double timestamp,
                                          long unsigned int keyframe_id,
                                          SourceType source_type,
                                          bool updatePose)
    {
        unique_lock<mutex> lockPose(mMutexPose);
        mTrackId = obs.track_id;
        mClassId = obs.class_id;
        mfScore = obs.score;
        mSourceType = source_type;
        mSize = obs.size;

        if (mnSeenCount == 0) {
            mFirstObservedTime = timestamp;
            mLastObservedTime = timestamp;
            mnFirstObservedKFId = keyframe_id;
            mnLastObservedKFId = keyframe_id;
            mnLastLifecycleKFId = keyframe_id;
            mfConfidence = obs.score;
        } else {
            mLastObservedTime = timestamp;
            mnLastObservedKFId = keyframe_id;
            mnLastLifecycleKFId = keyframe_id;
            mfConfidence = 0.7f * mfConfidence + 0.3f * obs.score;
        }
        mnSeenCount++;
        mnLostCount = 0;
        if (keyframe_id >= mnFirstObservedKFId) {
            mnAge = int(keyframe_id - mnFirstObservedKFId);
        }
        if (obs.is_static) {
            mnStaticObservationCount++;
        } else {
            mnDynamicObservationCount++;
        }
        mState = ResolveObjectState(mnStaticObservationCount, mnDynamicObservationCount);

        const Eigen::Matrix3f Rwb = CommonTools::T2R(Twb);
        mWorldVelocity = Rwb * obs.v_ref;

        if (!updatePose) {
            return;
        }

        const Eigen::Vector3f twb = CommonTools::T2t(Twb);
        mWorldPos = Rwb * obs.t_ref + twb;

        Eigen::Quaternionf qwb(Rwb);
        qwb.normalize();
        mWorldRotation = qwb * obs.q_ref;
        mWorldRotation.normalize();
    }

    void MapObject::MarkMissed(long unsigned int current_keyframe_id, double current_timestamp)
    {
        unique_lock<mutex> lockPose(mMutexPose);
        if (mbBad || mnSeenCount <= 0) {
            return;
        }
        if (current_keyframe_id <= mnLastObservedKFId) {
            return;
        }

        if (current_keyframe_id <= mnLastLifecycleKFId) {
            return;
        }

        mnLostCount += int(current_keyframe_id - mnLastLifecycleKFId);
        mnLastLifecycleKFId = current_keyframe_id;
        if (current_keyframe_id >= mnFirstObservedKFId) {
            mnAge = int(current_keyframe_id - mnFirstObservedKFId);
        }
        (void)current_timestamp;
    }

    void MapObject::SetWorldPose(const Eigen::Vector3f &position, const Eigen::Quaternionf &rotation)
    {
        unique_lock<mutex> lockPose(mMutexPose);
        mWorldPos = position;
        mWorldRotation = rotation;
        mWorldRotation.normalize();
    }

    Eigen::Vector3f MapObject::GetWorldPos() const
    {
        unique_lock<mutex> lock(mMutexPose);
        return mWorldPos;
    }

    Eigen::Quaternionf MapObject::GetWorldRotation() const
    {
        unique_lock<mutex> lock(mMutexPose);
        return mWorldRotation;
    }

    Eigen::Vector3f MapObject::GetWorldVelocity() const
    {
        unique_lock<mutex> lock(mMutexPose);
        return mWorldVelocity;
    }

    Eigen::Vector3f MapObject::GetSize() const
    {
        unique_lock<mutex> lock(mMutexPose);
        return mSize;
    }

    int MapObject::GetTrackId() const
    {
        unique_lock<mutex> lock(mMutexPose);
        return mTrackId;
    }

    int MapObject::GetClassId() const
    {
        unique_lock<mutex> lock(mMutexPose);
        return mClassId;
    }

    float MapObject::GetScore() const
    {
        unique_lock<mutex> lock(mMutexPose);
        return mfScore;
    }

    float MapObject::GetConfidence() const
    {
        unique_lock<mutex> lock(mMutexPose);
        return mfConfidence;
    }

    int MapObject::GetSeenCount() const
    {
        unique_lock<mutex> lock(mMutexPose);
        return mnSeenCount;
    }

    int MapObject::GetLostCount() const
    {
        unique_lock<mutex> lock(mMutexPose);
        return mnLostCount;
    }

    int MapObject::GetAge() const
    {
        unique_lock<mutex> lock(mMutexPose);
        return mnAge;
    }

    int MapObject::GetStaticObservationCount() const
    {
        unique_lock<mutex> lock(mMutexPose);
        return mnStaticObservationCount;
    }

    int MapObject::GetDynamicObservationCount() const
    {
        unique_lock<mutex> lock(mMutexPose);
        return mnDynamicObservationCount;
    }

    long unsigned int MapObject::GetFirstObservedKFId() const
    {
        unique_lock<mutex> lock(mMutexPose);
        return mnFirstObservedKFId;
    }

    long unsigned int MapObject::GetLastObservedKFId() const
    {
        unique_lock<mutex> lock(mMutexPose);
        return mnLastObservedKFId;
    }

    MapObject::ObjectState MapObject::GetState() const
    {
        unique_lock<mutex> lock(mMutexPose);
        return mState;
    }

    MapObject::SourceType MapObject::GetSourceType() const
    {
        unique_lock<mutex> lock(mMutexPose);
        return mSourceType;
    }

    bool MapObject::IsStatic() const
    {
        unique_lock<mutex> lock(mMutexPose);
        return mState == OBJECT_STATE_STATIC;
    }

    bool MapObject::IsDynamic() const
    {
        unique_lock<mutex> lock(mMutexPose);
        return mState == OBJECT_STATE_DYNAMIC;
    }

    bool MapObject::IsUnknown() const
    {
        unique_lock<mutex> lock(mMutexPose);
        return mState == OBJECT_STATE_UNKNOWN;
    }

    void MapObject::SetBadFlag()
    {
        vector<pair<KeyFrame*, int>> vObservations;
        {
            unique_lock<mutex> lock(mMutexFeatures);
            if (mbBad) {
                return;
            }
            mbBad = true;
            vObservations = mvObservations;
            nObs = 0;
            nObsKF = 0;
            mObservations.clear();
            mvObservations.clear();
            mvpKF.clear();
        }

        for (const auto &obs : vObservations) {
            KeyFrame *pKF = obs.first;
            if (pKF == nullptr || pKF->isBad()) {
                continue;
            }
            pKF->EraseObjectObservation(obs.second);
        }
    }

    bool MapObject::isBad() const
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return mbBad;
    }
}
