/*********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018
 * Capable Humanitarian Robotics and Intelligent Systems Lab (CHRISLab)
 * Christopher Newport University
 *
 *  This code is based on sample code from LeapMotionSDK
 *  Copyright (C) 2012-2013 Leap Motion, Inc. All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Christopher Newport University, TU Darmstadt,
 *     Team ViGIR, nor the names of its contributors may be used to endorse
 *     or promote products derived from this software without specific prior
 *     written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#include <Leap.h>
#include "ros/ros.h"
#include <leap_interface/HandPoseStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

class LeapController : public Leap::Listener {
public:
    LeapController(ros::NodeHandle &nh);
    virtual void onInit(const Leap::Controller&);
    virtual void onConnect(const Leap::Controller&);
    virtual void onDisconnect(const Leap::Controller&);
    virtual void onExit(const Leap::Controller&);
    virtual void onFrame(const Leap::Controller&);
    virtual void onFocusGained(const Leap::Controller&);
    virtual void onFocusLost(const Leap::Controller&);
    virtual void onDeviceChange(const Leap::Controller&);
    virtual void onServiceConnect(const Leap::Controller&);
    virtual void onServiceDisconnect(const Leap::Controller&);

    enum HandToSenseEnum {FIRST_HAND, LEFT_HAND, RIGHT_HAND, BOTH_HANDS};
private:
    ros::NodeHandle nh_;
    ros::Publisher hand_pose_publisher_;
    ros::Publisher pose_stamped_publisher_;

    LeapController::HandToSenseEnum hand_to_sense_;
    std::vector<Leap::Finger::Type> fingers_to_track_;
    std::string base_frame_;
    std::string palm_frame_;

    tf::Transform tfLeapInBase_;
    boost::shared_ptr<tf::TransformBroadcaster> tf_broadcaster_;
    tf::StampedTransform  tf_palm_;


    geometry_msgs::PoseStamped current_pose_stamped_msg_;
    leap_interface::HandPoseStamped current_hand_msg_;
    Leap::Frame current_frame_;

    void processFrame();
    void processHand(const Leap::Hand& aHand);
    void processFinger(const Leap::Finger& aFinger);
    void publishHandPose();
    void resetMessageInfo();
    LeapController::HandToSenseEnum convertToHandSenseEnum(std::string const& aString);
    Leap::Finger::Type convertToFingerType(std::string const& aString);
};
