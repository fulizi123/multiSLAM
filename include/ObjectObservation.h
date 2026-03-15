#ifndef OBJECTOBSERVATION_H
#define OBJECTOBSERVATION_H

#include "CommonTools.h"

namespace LL_SLAM
{
    struct ObjectObservation {
        static constexpr int kNumFloatsPerObject = 24;

        int track_id = -1;
        int class_id = -1;
        float score = 0.0f;

        Eigen::Vector3f t_ref = Eigen::Vector3f::Zero();
        Eigen::Vector3f size = Eigen::Vector3f::Zero();
        Eigen::Vector3f v_ref = Eigen::Vector3f::Zero();
        Eigen::Quaternionf q_ref = Eigen::Quaternionf::Identity();

        float speed = 0.0f;
        bool is_static = false;

        Eigen::Vector3f t_world = Eigen::Vector3f::Zero();
        int num_lidar_pts = 0;
        int num_radar_pts = 0;
    };
}

#endif // OBJECTOBSERVATION_H
