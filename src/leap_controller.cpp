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

/* Authors: Ricardo Flores, Kyle Frizzell, and David Conner */

#include <iostream>
#include <cstring>

#include <leap_controller/FingerPoseStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include "tf/LinearMath/Matrix3x3.h"
#include "tf/transform_datatypes.h"

LeapController::LeapController(ros::NodeHandle &nh):nh_(nh){
  std::string output_pose_topic_name;
  nh_.param<std::string>("leap_output_pose_topic",output_pose_topic_name, "/leap_hand_pose");
  ROS_INFO("Publishing Leap Hand Poses to topic [%s]", output_pose_topic_name.c_str());
  hand_pose_publisher_ = nh_.advertise<leap_controller_capstone::HandPoseStamped>(output_pose_topic_name,10);

  std::string hand_to_sense_tmp;
  nh_.param<std::string>("hand_to_sense",hand_to_sense_tmp, "first");
  hand_to_sense_ = convertToHandSenseEnum(hand_to_sense_tmp);

  std::vector<std::string> fingers_to_track_tmp;
  nh_.getParam("fingers_to_track_list",fingers_to_track_tmp);
  for (std::string finger : fingers_to_track_tmp){
    fingers_to_track_.push_back(convertToFingerType(finger));
  }

  nh_.param<std::string>("base_frame",base_frame_, "/root");

  nh_.param<double>("x_offset_position",xOffset_, 0.0);
  nh_.param<double>("y_offset_position",yOffset_, 0.0);
  nh_.param<double>("z_offset_position",zOffset_, 0.0);
  nh_.param<double>("roll_offset_orientation", rollOffset_,  0.0);
  nh_.param<double>("pitch_offset_orientation",pitchOffset_, 0.0);
  nh_.param<double>("yaw_offset_orientation",  yawOffset_,   0.0);


}

LeapController::HandToSenseEnum LeapController::convertToHandSenseEnum(std::string const& aString){
  if      (aString == "first") return FIRST_HAND;
  else if (aString == "left")  return LEFT_HAND;
  else if (aString == "right") return RIGHT_HAND;
  else if (aString == "both")  return BOTH_HANDS;
  else {
    ROS_WARN("INVALID hand_to_sense value:[%s]. Possible values are [first,left,right,both]",aString.c_str());
    ROS_INFO("Unrecognized hand_to_sense value. Defaulting to 'first'");
    return FIRST_HAND;
  }
}

Leap::Finger::Type LeapController::convertToFingerType(std::string const& aString){
  if (aString == "thumb") return Leap::Finger::TYPE_THUMB;
  else if (aString == "index") return Leap::Finger::TYPE_INDEX;
  else if (aString == "middle") return Leap::Finger::TYPE_MIDDLE;
  else if (aString == "ring") return Leap::Finger::TYPE_RING;
  else if (aString == "pinky") return Leap::Finger::TYPE_PINKY;
  else {
    ROS_WARN("INVALID finger_to_track value:[%s]. Possible values are [thumb,index,middle,ring,pinky]",aString.c_str());
  }
}

void LeapController::onFrame(const Leap::Controller& controller){
  current_frame_ = controller.frame();
  processFrame();
}

void LeapController::processFrame(){
  Leap::HandList hands = current_frame_.hands();
  if (!hands.isEmpty()){
    switch(hand_to_sense_){
      case FIRST_HAND:
        processHand(hands[0]);
        break;

      case LEFT_HAND:
        for (Leap::HandList::const_iterator hl = hands.begin(); hl != hands.end(); ++hl) {
            const Leap::Hand hand = *hl;
            if(hand.isLeft()){
              processHand(hand);
            }
          }
          break;

      case RIGHT_HAND:
        for (Leap::HandList::const_iterator hl = hands.begin(); hl != hands.end(); ++hl) {
            const Leap::Hand hand = *hl;
            if(hand.isRight()){
              processHand(hand);
            }
          }
          break;

      case BOTH_HANDS:
        for (Leap::HandList::const_iterator hl = hands.begin(); hl != hands.end(); ++hl) {
            const Leap::Hand hand = *hl;
            processHand(hand);
          }
        break;
    }//end switch
  }//end if
}

void LeapController::processHand(const Leap::Hand& aHand){
  current_hand_msg_.name = aHand.isLeft() ? "left" : "right";
  current_hand_msg_.header.stamp = ros::Time::now();
  current_hand_msg_.header.frame_id = base_frame_;
  current_hand_msg_.posePalm.header.frame_id = base_frame_;
  current_hand_msg_.posePalm.pose.position.x = -(aHand.palmPosition().x/1000) + xOffset_;
  current_hand_msg_.posePalm.pose.position.y =  (aHand.palmPosition().z/1000) + yOffset_;
  current_hand_msg_.posePalm.pose.position.z =  (aHand.palmPosition().y/1000) + zOffset_;

  current_hand_msg_.posePalm.pose.orientation.x = 0;
  current_hand_msg_.posePalm.pose.orientation.y = 0;
  current_hand_msg_.posePalm.pose.orientation.z = 0;
  current_hand_msg_.posePalm.pose.orientation.w = 1;
  //TODO add logic code to get important information on the palm here

  if(!fingers_to_track_.empty()){
    const Leap::FingerList fingers = aHand.fingers();
    for (Leap::FingerList::const_iterator fl = fingers.begin(); fl != fingers.end(); ++fl) {
        const Leap::Finger finger = *fl;
        if(std::find(fingers_to_track_.begin(),fingers_to_track_.end(),finger.type()) != fingers_to_track_.end()){
          processFinger(finger);
        }
      }
  }
  publishHandPose();
}

void LeapController::processFinger(const Leap::Finger& aFinger){
  leap_controller_capstone::FingerPoseStamped finger_msg;
  switch(aFinger.type()){
    case Leap::Finger::TYPE_THUMB:
      finger_msg.name = "thumb";
      break;
    case Leap::Finger::TYPE_INDEX:
      finger_msg.name = "index";
      break;
    case Leap::Finger::TYPE_MIDDLE:
      finger_msg.name = "middle";
      break;
    case Leap::Finger::TYPE_RING:
      finger_msg.name = "ring";
      break;
    case Leap::Finger::TYPE_PINKY:
      finger_msg.name = "pinky";
      break;
  }

  for (int boneEnum = 0; boneEnum < 4; ++boneEnum) {
      Leap::Bone::Type boneType = static_cast<Leap::Bone::Type>(boneEnum);
      Leap::Bone bone = aFinger.bone(boneType);
      switch (bone.type()) {
        case Leap::Bone::TYPE_DISTAL:
          finger_msg.poseDistalPhalange.pose.position.x = -(double)((bone.nextJoint().x)/1000) + xOffset_;//Used nextJoint to get the tip
          finger_msg.poseDistalPhalange.pose.position.y =  (double)((bone.nextJoint().z)/1000) + yOffset_;
          finger_msg.poseDistalPhalange.pose.position.z =  (double)((bone.nextJoint().y)/1000) + zOffset_;
          //TODO fill in the rest for position and orientation
          //translate to ROS Space here
          //how to do correct orientation?
            //look at both old arm_mimic_capstone code and old leap code
          break;
        case Leap::Bone::TYPE_INTERMEDIATE:
          finger_msg.poseIntermediatePhalange.pose.position.x = -(double)((bone.center().x)/1000) + xOffset_;
          finger_msg.poseIntermediatePhalange.pose.position.y =  (double)((bone.center().z)/1000) + yOffset_;
          finger_msg.poseIntermediatePhalange.pose.position.z =  (double)((bone.center().y)/1000) + zOffset_;
          //TODO fill in
          break;
        case Leap::Bone::TYPE_PROXIMAL:
          finger_msg.poseProximalPhalange.pose.position.x = -(double)((bone.center().x)/1000) + xOffset_;
          finger_msg.poseProximalPhalange.pose.position.y =  (double)((bone.center().z)/1000) + yOffset_;
          finger_msg.poseProximalPhalange.pose.position.z =  (double)((bone.center().y)/1000) + zOffset_;
          //TODO fill in
          break;
        case Leap::Bone::TYPE_METACARPAL:
          finger_msg.poseMetacarpal.pose.position.x = -(double)((bone.center().x)/1000) + xOffset_;
          finger_msg.poseMetacarpal.pose.position.y =  (double)((bone.center().z)/1000) + yOffset_;
          finger_msg.poseMetacarpal.pose.position.z =  (double)((bone.center().y)/1000) + zOffset_;
          //TODO fill in
          break;
      }
    }//end for loop
    current_hand_msg_.poseFingers.push_back(finger_msg);
}

void LeapController::publishHandPose(){
  hand_pose_publisher_.publish(current_hand_msg_);
  resetMessageInfo();
}

void LeapController::resetMessageInfo(){
  current_hand_msg_ = leap_controller_capstone::HandPoseStamped();
}

void LeapController::onInit(const Leap::Controller& controller) {
    ROS_INFO("Leap Controller: Initialized");
}

void LeapController::onConnect(const Leap::Controller& controller) {
    ROS_INFO("Leap Controller: Connected");
}

void LeapController::onDisconnect(const Leap::Controller& controller) {
    // Note: not dispatched when running in a debugger.
    ROS_INFO("Leap Controller: Disconnected");
}

void LeapController::onExit(const Leap::Controller& controller) {
    ROS_INFO("Leap Controller: Exited");
}

void LeapController::onFocusGained(const Leap::Controller& controller) {
    ROS_INFO("Leap Controller: Focus Gained");
}

void LeapController::onFocusLost(const Leap::Controller& controller) {
    ROS_INFO("Leap Controller: Focus Lost");
}

void LeapController::onDeviceChange(const Leap::Controller& controller) {
    ROS_INFO("Leap Controller: Device Changed");
    const Leap::DeviceList devices = controller.devices();

    for (int i = 0; i < devices.count(); ++i) {
        ROS_INFO("Leap Controller: id: [%s]", devices[i].toString().c_str());
        ROS_INFO("  isStreaming:  [%s]",(devices[i].isStreaming() ? "true" : "false"));
    }
}

void LeapController::onServiceConnect(const Leap::Controller& controller) {
    ROS_INFO("Leap Controller: Service Connected");
}

void LeapController::onServiceDisconnect(const Leap::Controller& controller) {
    ROS_INFO("Leap Controller: Service Disconnected");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "leap_controller_node");
    ros::NodeHandle node_handle;

    // Create the LeapController instance
    LeapController listener = LeapController(node_handle);
    Leap::Controller controller;

    // Have the sample listener receive events from the controller
    controller.addListener(listener);

    //Essentially, it runs everything as a background multi-thread.
    if (argc > 1 && strcmp(argv[1], "--bg") == 0)
        controller.setPolicy(Leap::Controller::POLICY_BACKGROUND_FRAMES);

    // process the data
    while(ros::ok()){
      ros::spinOnce();
    }

    // Remove the sample listener when done
    ROS_INFO(" Shutting down LeapMotion listener ... ");
    controller.removeListener(listener);

    return 0;
}
