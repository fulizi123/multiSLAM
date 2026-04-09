#ifndef CARLA_TOPOLOGY_STATUS_H
#define CARLA_TOPOLOGY_STATUS_H

namespace LL_SLAM
{
    struct CarlaTopologyStatus {
        static constexpr int kNumFloatsPerFrame = 7;

        int road_id = -1;
        int section_id = -1;
        int lane_id = 0;
        int junction_id = -1;
        bool is_junction = false;
        int junction_proximity_id = -1;
        float next_junction_distance_m = -1.0f;
        bool valid = false;
    };
}

#endif // CARLA_TOPOLOGY_STATUS_H
