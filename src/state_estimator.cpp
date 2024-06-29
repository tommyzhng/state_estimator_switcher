#include "state_estimator/state_estimator.hpp"

namespace fsc {

    StateEstimatorNode::StateEstimatorNode(ros::NodeHandle& n)
        //statePub(nh.advertise<nav_msgs::Odometry>("/mavros/local_position/odom/UAV0", 1)),
        //estimatorTypePub(nh.advertise<std_msgs::Bool>("/estimator_type", 1))
    {
        // ros::NodeHandle nh("~");
        //nh.param("indoorMode", indoorMode, true);

        n.param("indoorMode", indoorMode, true);

        // initialize subscriber
        if (indoorMode) {
            localPositionSub = n.subscribe("/mocap/UAV0", 1, &StateEstimatorNode::GetMocapMsg, this);
        } else {
            localPositionSub = n.subscribe("/state_estimator/local_position/odom", 1, &StateEstimatorNode::GetGPSMsg, this);
        }
        statePub = n.advertise<nav_msgs::Odometry>("/state_estimator/local_position/odom/UAV0", 1);
        estimatorTypePub = n.advertise<std_msgs::Bool>("/estimator_type", 1);
    }

    void StateEstimatorNode::CheckEstimator(void)
    {
        if (loopCounter >= loopThreshold) {
            loopCounter = 0;
            std_msgs::Bool success;
            success.data = (indoorMode) ? true : false;
            estimatorTypePub.publish(success);
        }
        loopCounter++;
    }

    void SetMocapFlag(void)
    {

    }

    void StateEstimatorNode::GetMocapMsg(const optitrack_broadcast::Mocap::ConstPtr &msg)
    {
        state.header = msg->header;

        state.pose.pose = msg->pose;

        state.twist.twist = msg->twist;
   }

    void StateEstimatorNode::GetGPSMsg(const nav_msgs::Odometry::ConstPtr &msg)
    {
        state = *msg;
    }

    void StateEstimatorNode::PubPose(void)
    {
        statePub.publish(state);
    }

    void StateEstimatorNode::Update(void)
    {
        CheckEstimator();
        PubPose();
    }

}
