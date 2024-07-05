#ifndef STATE_ESTIMATOR_STATE_ESTIMATOR_HPP
#define STATE_ESTIMATOR_STATE_ESTIMATOR_HPP

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <optitrack_broadcast/Mocap.h>
#include "geometry_msgs/PoseStamped.h"
#include <std_msgs/Bool.h>

namespace fsc {

class StateEstimatorNode
{
public:

    struct mocapWatchdogState {
        bool mocapRecieved{false};
        uint32_t mocapTimeoutTime{10};
    };

    void Update(void);
    StateEstimatorNode(ros::NodeHandle& n);
    ~StateEstimatorNode() = default;

private:
    void GetMocapMsg(const optitrack_broadcast::Mocap::ConstPtr &msg);
    void GetGPSMsg(const nav_msgs::Odometry::ConstPtr &msg);
    void PubPose(void);
    void CheckEstimator(void);
    void SetMocapFlag(void);
    
    ros::Subscriber localPositionSub;
    ros::Subscriber velocitySub;
    ros::Publisher statePub;
    ros::Publisher estimatorTypePub;
    ros::Publisher visionPosePub;
    bool indoorMode{false};
    uint64_t loopCounter{0};
    uint64_t loopThreshold{20};
    nav_msgs::Odometry state;
    geometry_msgs::PoseStamped vision_pose;
};
};
#endif
