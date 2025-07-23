#include <slam/action_model.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <utils/geometric/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>
#include <algorithm>

//implement Action Model with defined gains
//describe how the robot moves from one pose to another given a control input.
ActionModel::ActionModel(void): k1_(0.005f), k2_(0.025f), min_dist_(0.0025), min_theta_(0.02), initialized_(false) {
    std::random_device rd;
    numberGenerator_ = std::mt19937(rd());
}

void ActionModel::resetPrevious (const mbot_lcm_msgs::pose2D_t& odometry) {
    previousPose_ = odometry;
}

bool ActionModel::operateAction(const mbot_lcm_msgs::pose2D_t& odometry) {
    if (!initialized) {
        previousPose_ = odometry;
        initialized_ = true;
    }

    dx_ = odometry.x - previousPose_.x;
    dy_ = odometry.y - previousPose_.y;
    dtheta_ = odometry.theta - previousPose_.theta;

    float direction = 1.0;
    rot1_ = angle_diff(std::atan2(dy_, dx_), previousPose_.theta);
    //if robot moves backwards
    if (std::abs(rot1_) > M_PI/2.0) {
        rot1_ = angle_diff(M_PI, rot1_);
        direction = -1.0;
    }

    rot2_ = angle_diff(dtheta_, rot1_);
    trans_ = direction * std::sqrt(dx_ * dx_ + dy_ * dy_);
    moved_ = (std::abs(trans_) > min_dist_) || (dtheta_ >= min_theta_);

    if (moved_) {
        rot1Std_ = std::sqrt(k1_ * std::abs(rot1_));
        rot2Std_ = std::sqrt(k1_ * std::abs(rot2_));
        transStd_ = std::sqrt(k2_ * std::abs(trans_));
    }
    resetPrevious(odometry);
    utime_ = odometry.utime;

    return moved_;        //place holder
}


//Sampling new poses from the distribution computed in updateAction//
mbot_lcm_msgs::particle_t ActionModel::applyAction(const mbot_lcm_msgs::particle_t& sample)
{
    mbot_lcm_msgs::particle_t newSample = sample;

    float sampleRot1 = std::normal_distribution<>(rot1_, rot1Std_)(numberGenerator_);
    float sampleTrans = std::normal_distribution<>(trans_, transStd_)(numberGenerator_);
    float sampleRot2 = std::normal_distribution<>(rot2_, rot2Std_)(numberGenerator_);

    newSample.pose.x += sampleTrans * cos(sample.pose.theta + sampleRot1);
    newSample.pose.y += sampleTrans * sin(sample.pose.theta + sampleRot1);
    newSample.pose.theta = wrap_to_pi(sample.pose.theta + sampleRot1 + sampleRot2);
    
    newSample.pose.utime = utime_;
    newSample.parent_pose = sample.pose;

    return newSample;
}



    

