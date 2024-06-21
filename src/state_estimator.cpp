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
            localPositionSub = n.subscribe("/mavros/local_position/odom", 1, &StateEstimatorNode::GetGPSMsg, this);
        }
        statePub = n.advertise<nav_msgs::Odometry>("/mavros/local_position/odom/UAV0", 1);
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

    void StateEstimatorNode::GetMocapMsg(const state_estimator::Mocap::ConstPtr &msg)
    {
        state.header = msg->header;

        state.pose.pose.position.x = msg->position[0];
        state.pose.pose.position.y = msg->position[1];
        state.pose.pose.position.z = msg->position[2];

        state.twist.twist.linear.x = msg->velocity[0];
        state.twist.twist.linear.y = msg->velocity[1];
        state.twist.twist.linear.z = msg->velocity[2];

        state.twist.twist.angular.x = msg->angular_velocity[0];
        state.twist.twist.angular.y = msg->angular_velocity[1];
        state.twist.twist.angular.z = msg->angular_velocity[2];

        state.pose.pose.orientation.x = msg->quaternion[1];
        state.pose.pose.orientation.y = msg->quaternion[2];
        state.pose.pose.orientation.z = msg->quaternion[3];
        state.pose.pose.orientation.w = msg->quaternion[0];
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