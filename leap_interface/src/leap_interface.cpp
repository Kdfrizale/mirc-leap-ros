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

#include <leap_interface/leap_interface.h>
#include <geometry_msgs/PoseStamped.h>
#include "tf/LinearMath/Matrix3x3.h"
#include "tf/transform_datatypes.h"

std::ostream& operator<<(std::ostream& os, const tf::Quaternion &quat)
{
  os << "[" << quat.x() << ", " << quat.y() << ", " << quat.z() << ", " << quat.w() << "]";
  return os;
}
std::ostream& operator<<(std::ostream& os, const tf::Vector3 &vec)
{
  os << "(" << vec.x() << ", " << vec.y() << ", " << vec.z()  << ")";
  return os;
}


LeapController::LeapController(ros::NodeHandle &nh):nh_(nh){
  std::string output_pose_topic_name;
  nh_.param<std::string>("hand_pose_topic",output_pose_topic_name, "/leap_hand_pose");
  ROS_INFO("Publishing Leap Hand Poses to topic [%s]", output_pose_topic_name.c_str());
  hand_pose_publisher_ = nh_.advertise<leap_interface::HandPoseStamped>(output_pose_topic_name,10);

  nh_.param<std::string>("pose_stamped_topic",output_pose_topic_name, "/leap_pose");
  if (output_pose_topic_name.length() > 0) {
    ROS_INFO("Publishing simple pose stamped to topic [%s]", output_pose_topic_name.c_str());
    pose_stamped_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>(output_pose_topic_name,10);
  }


  std::string hand_to_sense_tmp;
  nh_.param<std::string>("hand_to_sense",hand_to_sense_tmp, "first");
  hand_to_sense_ = convertToHandSenseEnum(hand_to_sense_tmp);

  std::vector<std::string> fingers_to_track_tmp;
  nh_.getParam("fingers_to_track_list",fingers_to_track_tmp);
  for (std::string finger : fingers_to_track_tmp){
    fingers_to_track_.push_back(convertToFingerType(finger));
  }

  nh_.param<std::string>("base_frame",base_frame_, "/root");
  nh_.param<std::string>("palm_frame",palm_frame_, "/palm");
  nh_.param<bool>("use_stabilized_position",use_stabilized_position_,false);
  if (use_stabilized_position_)
  {
    ROS_INFO("Using stabilized palm position from Leap");
  }
  double xOffset, yOffset, zOffset, rollOffset, pitchOffset, yawOffset;
  nh_.param<double>("x_offset_position",xOffset, 0.0);
  nh_.param<double>("y_offset_position",yOffset, 0.0);
  nh_.param<double>("z_offset_position",zOffset, 0.0);
  nh_.param<double>("roll_offset_orientation",  rollOffset,  0.0); // angle about x-axis ( "roll" in ZYX form)
  nh_.param<double>("pitch_offset_orientation", pitchOffset, 0.0); // angle around the y-axis ( "pitch" in ZYX form)
  nh_.param<double>("yaw_offset_orientation",   yawOffset,   0.0); // angle aroud the z-axis ( "yaw" in ZYX form)
  tf::Quaternion tfQuat = tf::createQuaternionFromRPY( rollOffset, pitchOffset, yawOffset); // ROS notation
  tfQuat.normalize();

  // Pose in Leap frame
  tfLeapInBase_ = tf::Transform(tfQuat, tf::Vector3(xOffset, yOffset, zOffset));
  std::cout << " tfLeapInBase_ = " << tfLeapInBase_.getRotation() << "  " << tfLeapInBase_.getOrigin() << std::endl;

  bool broadcast_tf = true;
  nh_.param<bool>("broadcast_tf",  broadcast_tf, true); // angle aroud the z-axis ( "yaw" in ZYX form)
  if (broadcast_tf){
    ROS_INFO(" Set up the tf broadcaster ...");
    tf_broadcaster_.reset(new tf::TransformBroadcaster() );
    tf_palm_.stamp_ = ros::Time::now();
    tf_palm_.frame_id_       = base_frame_;
    tf_palm_.child_frame_id_ = palm_frame_;

    // Can only broadcast fingers if also doing palm
    nh_.param<bool>("broadcast_fingers_tf",  broadcast_fingers_tf_, false);
  }
  else {
    broadcast_fingers_tf_ = false;
  }
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
  } else {
      ROS_INFO_THROTTLE(1.0," Not tracking - empty hands array ...");
  }//end if
}

void LeapController::processHand(const Leap::Hand& aHand){
  current_hand_msg_.name = aHand.isLeft() ? "left" : "right";
  current_hand_msg_.header.stamp = ros::Time::now();
  current_hand_msg_.header.frame_id = base_frame_;
  //current_hand_msg_.posePalm.header.frame_id = base_frame_;

  // Get hand orientation in Leap frame
  //float pitch = aHand.direction().pitch(); // angle about x-axis ( "roll" in ZYX form)
  //float yaw   = aHand.direction().yaw();   // angle around the y-axis ( "pitch" in ZYX form)
  //float roll  = aHand.palmNormal().roll(); // angle aroud the z-axis ( "yaw" in ZYX form)
  //tf::Quaternion tfQuat  = tf::createQuaternionFromRPY(roll,pitch, yaw);//pitch, yaw, roll); // using Leap names in ROS ZYX order
  //tfQuat.normalize();
  tf::Quaternion tfQuat;
  convertLeapBasisToQuat(tfQuat, aHand.basis(), aHand.isLeft());

  // Pose in Leap frame
  tf::Vector3 palmPosition(aHand.palmPosition().x/1000., aHand.palmPosition().y/1000., aHand.palmPosition().z/1000);
  if (use_stabilized_position_){
    palmPosition = tf::Vector3(aHand.stabilizedPalmPosition().x/1000., aHand.stabilizedPalmPosition().y/1000., aHand.stabilizedPalmPosition().z/1000);
  }
  tf::Transform tfPalmInLeap(tfQuat,palmPosition);

  // Pose in base frame
  tf::Transform tfPalmInBase = tfLeapInBase_*tfPalmInLeap;
  //std::cout << " tfPalmInLeap = " << tfPalmInLeap.getRotation() << "  " << tfPalmInLeap.getOrigin() << std::endl;

  tfQuat = tfPalmInBase.getRotation();
  tf::Vector3 tfOrigin = tfPalmInBase.getOrigin();
  //std::cout << " tfPalmInBase = " << tfQuat << "   " << tfOrigin << std::endl;
  current_hand_msg_.posePalm.position.x    = tfOrigin.x(); // original had this negated?
  current_hand_msg_.posePalm.position.y    = tfOrigin.y();
  current_hand_msg_.posePalm.position.z    = tfOrigin.z();
  current_hand_msg_.posePalm.orientation.x = tfQuat.x();
  current_hand_msg_.posePalm.orientation.y = tfQuat.y();
  current_hand_msg_.posePalm.orientation.z = tfQuat.z();
  current_hand_msg_.posePalm.orientation.w = tfQuat.w();

  if (tf_broadcaster_){
    tf_palm_.stamp_ = ros::Time::now();
    tf_palm_.setData(tfPalmInBase);
    tf_transforms_.clear();
    if (broadcast_fingers_tf_)
      tf_transforms_.reserve(1 + 4*fingers_to_track_.size());

    tf_transforms_.push_back(tf_palm_);
  }

  if(!fingers_to_track_.empty()){
    const Leap::FingerList fingers = aHand.fingers();
    for (Leap::FingerList::const_iterator fl = fingers.begin(); fl != fingers.end(); ++fl) {
        const Leap::Finger finger = *fl;
        if(std::find(fingers_to_track_.begin(),fingers_to_track_.end(),finger.type()) != fingers_to_track_.end()){
          processFinger(finger,aHand.isLeft());
        }
      }
  }
  publishHandPose();

  if (tf_broadcaster_){
    tf_broadcaster_->sendTransform(tf_transforms_);
  }
}

void LeapController::convertLeapBasisToQuat(tf::Quaternion& tfQuat, const Leap::Matrix basis, const bool isLeftHand)
{

  /**
   * The orientation of the hand as a basis matrix.
   *
   * The basis is defined as follows:
   * **xAxis** Positive in the direction of the pinky
   * **yAxis** Positive above the hand
   * **zAxis** Positive in the direction of the wrist
   *
   * Note: Since the left hand is a mirror of the right hand, the
   * basis matrix will be left-handed for left hands.
   */

 /**
  * The orientation of a finger as a basis
  * **xBasis** Perpendicular to the longitudinal axis of the
  *   bone; exits the sides of the finger.
  *
  * **yBasis or up vector** Perpendicular to the longitudinal
  *   axis of the bone; exits the top and bottom of the finger. More positive
  *   in the upward direction.
  *
  * **zBasis** Aligned with the longitudinal axis of the bone.
  *   More positive toward the base of the finger.
  *
  * The bases provided for the right hand use the right-hand rule; those for
  * the left hand use the left-hand rule. Thus, the positive direction of the
  * x-basis is to the right for the right hand and to the left for the left
  * hand. You can change from right-hand to left-hand rule by multiplying the
  * z basis vector by -1. ??? why not x-basis ???
  */
  Leap::Vector xBasis = basis.xBasis;
  if (isLeftHand)
  {
    xBasis *= -1.0;
  }

  // This works for Leap frame with z-axis from palm to wrist and y-axis out the top of palm
  tf::Matrix3x3 rotation(xBasis.x, basis.yBasis.x, basis.zBasis.x,
                         xBasis.y, basis.yBasis.y, basis.zBasis.y,
                         xBasis.z, basis.yBasis.z, basis.zBasis.z);

  // Re-define with x-axis to fingers, and z-axis positive out the back of palm
  //    Commented code didn't work; the Frame looked OK at first, but rotations are off
  //tf::Matrix3x3 rotation(-basis.zBasis.x, -xBasis.x, basis.yBasis.x,
  //                       -basis.zBasis.y, -xBasis.y, basis.yBasis.y,
  //                       -basis.zBasis.z, -xBasis.z, basis.yBasis.z  );
  rotation.getRotation(tfQuat);
}

void LeapController::transformLeapFingerPose(geometry_msgs::Pose& fingerPose, const Leap::Vector position, const Leap::Matrix basis,
                                            const bool isLeftHand, const std::string& name) {


  // Pose in Leap frame
  tf::Quaternion tfQuat;
  convertLeapBasisToQuat(tfQuat, basis, isLeftHand);
  tf::Vector3   fingerPosition(position.x/1000., position.y/1000., position.z/1000);
  tf::Transform tfFingerInLeap(tfQuat,fingerPosition);

  // Pose in base frame
  tf::Transform tfFingerInBase = tfLeapInBase_*tfFingerInLeap;
  tfQuat = tfFingerInBase.getRotation();
  tf::Vector3 tfOrigin = tfFingerInBase.getOrigin();
  fingerPose.position.x    = tfOrigin.x();
  fingerPose.position.y    = tfOrigin.y();
  fingerPose.position.z    = tfOrigin.z();
  fingerPose.orientation.x = tfQuat.x();
  fingerPose.orientation.y = tfQuat.y();
  fingerPose.orientation.z = tfQuat.z();
  fingerPose.orientation.w = tfQuat.w();

  if (broadcast_fingers_tf_){
    tf_transforms_.push_back(tf::StampedTransform(tfFingerInBase, tf_palm_.stamp_, tf_palm_.frame_id_, name));
  }
}

void LeapController::processFinger(const Leap::Finger& aFinger, bool isLeftHand){
  leap_interface::FingerPose finger_msg;
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
    default:
      ROS_WARN(" Unknown finger type %d",aFinger.type());
      finger_msg.name = "unknown";
  }

  for (int boneEnum = 0; boneEnum < 4; ++boneEnum) {
      Leap::Bone::Type boneType = static_cast<Leap::Bone::Type>(boneEnum);
      Leap::Bone bone = aFinger.bone(boneType);
      std::string bone_name = isLeftHand ? "l":"r";
      bone_name  += finger_msg.name.at(0);
      switch (bone.type()) {
        case Leap::Bone::TYPE_DISTAL:
          transformLeapFingerPose(finger_msg.poseDistalPhalange,       bone.nextJoint(), bone.basis(), isLeftHand, bone_name+"_tip");//Used nextJoint to get the tip
          break;
        case Leap::Bone::TYPE_INTERMEDIATE:
          transformLeapFingerPose(finger_msg.poseIntermediatePhalange, bone.center(), bone.basis(), isLeftHand, bone_name+"_inter");
          break;
        case Leap::Bone::TYPE_PROXIMAL:
          transformLeapFingerPose(finger_msg.poseProximalPhalange,     bone.center(), bone.basis(), isLeftHand, bone_name+"_proxi");
          break;
        case Leap::Bone::TYPE_METACARPAL:
          transformLeapFingerPose(finger_msg.poseMetacarpal,           bone.center(), bone.basis(), isLeftHand, bone_name+"_meta");
          break;
      }
    }//end for loop
    current_hand_msg_.poseFingers.push_back(finger_msg);


}

void LeapController::publishHandPose(){
  current_pose_stamped_msg_.header = current_hand_msg_.header;
  current_pose_stamped_msg_.pose   = current_hand_msg_.posePalm;

  hand_pose_publisher_.publish(current_hand_msg_);
  if (pose_stamped_publisher_) {
    pose_stamped_publisher_.publish(current_pose_stamped_msg_);
  }
  resetMessageInfo();
}

void LeapController::resetMessageInfo(){
  current_hand_msg_ = leap_interface::HandPoseStamped();
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
    ros::init(argc, argv, "leap_interface_node");
    ros::NodeHandle node_handle("~");

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
