/*
 Based off of the very elaborate sample provided by the Leap Motion Developement team
 with the necessary adjustments and added code to use for my controller's intents and purposes.
 All rights reserved to the their proper place.
 Author: Ricardo Flores
 */

#include <iostream>
#include <cstring>
#include "../ignore/Leap.h"
#include "ros/ros.h"
#include <leap_controller_capstone/HandPoseStamped.h>
#include <leap_controller_capstone/FingerPoseStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include "../ignore/tf/LinearMath/Matrix3x3.h"
#include <boost/shared_ptr.hpp>
#include "../ignore/tf/transform_datatypes.h"

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
    LeapController::HandToSenseEnum hand_to_sense_;
    std::vector<Leap::Finger::Type> fingers_to_track_;
    std::string relative_frame_;

    double xOffset_;
    double yOffset_;
    double zOffset_;

    leap_controller_capstone::HandPoseStamped current_hand_msg_;
    Leap::Frame current_frame_;

    void processFrame();
    void processHand(const Leap::Hand& aHand);
    void processFinger(const Leap::Finger& aFinger);
    void publishHandPose();
    void resetMessageInfo();
    LeapController::HandToSenseEnum convertToHandSenseEnum(std::string const& aString);
    Leap::Finger::Type convertToFingerType(std::string const& aString);
};

LeapController::LeapController(ros::NodeHandle &nh):nh_(nh){
  std::string output_pose_topic_name;
  nh_.param<std::string>("leap_output_pose_topic",output_pose_topic_name, "/leap_hand_pose");
  ROS_INFO("Publishing Leap Hand Poses to topic [%s]", output_pose_topic_name.c_str());
  hand_pose_publisher_ = nh_.advertise<leap_controller_capstone::HandPoseStamped>(output_pose_topic_name,10);

  std::string hand_to_sense_tmp;
  nh_.param<std::string>("leap_controller_node/hand_to_sense",hand_to_sense_tmp, "first");
  hand_to_sense_ = convertToHandSenseEnum(hand_to_sense_tmp);

  std::vector<std::string> fingers_to_track_tmp;
  nh_.getParam("leap_controller_node/fingers_to_track_list",fingers_to_track_tmp);
  for (std::string finger : fingers_to_track_tmp){
    fingers_to_track_.push_back(convertToFingerType(finger));
  }

  nh_.param<std::string>("leap_controller_node/relative_frame",relative_frame_, "/root");

  nh_.param<double>("leap_controller_node/x_offset_position",xOffset_, 0.0);
  nh_.param<double>("leap_controller_node/y_offset_position",yOffset_, 0.0);
  nh_.param<double>("leap_controller_node/z_offset_position",zOffset_, 0.0);
}

LeapController::HandToSenseEnum LeapController::convertToHandSenseEnum(std::string const& aString){
  if (aString == "first") return FIRST_HAND;
  else if (aString == "left") return LEFT_HAND;
  else if (aString == "right") return RIGHT_HAND;
  else if (aString == "both") return BOTH_HANDS;
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
  current_hand_msg_.header.frame_id = relative_frame_;
  current_hand_msg_.posePalm.header.frame_id = relative_frame_;
  current_hand_msg_.posePalm.pose.position.x = -(aHand.palmPosition().x/1000) + xOffset_;
  current_hand_msg_.posePalm.pose.position.y = (aHand.palmPosition().z/1000) + yOffset_;
  current_hand_msg_.posePalm.pose.position.z = (aHand.palmPosition().y/1000) + zOffset_;

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
          finger_msg.poseDistalPhalange.pose.position.y = (double)((bone.nextJoint().z)/1000) + yOffset_;
          finger_msg.poseDistalPhalange.pose.position.z = (double)((bone.nextJoint().y)/1000) + zOffset_;
          //TODO fill in the rest for position and orientation
          //translate to ROS Space here
          //how to do correct orientation?
            //look at both old arm_mimic_capstone code and old leap code
          break;
        case Leap::Bone::TYPE_INTERMEDIATE:
          finger_msg.poseIntermediatePhalange.pose.position.x = -(double)((bone.center().x)/1000) + xOffset_;
          finger_msg.poseIntermediatePhalange.pose.position.y = (double)((bone.center().z)/1000) + yOffset_;
          finger_msg.poseIntermediatePhalange.pose.position.z = (double)((bone.center().y)/1000) + zOffset_;
          //TODO fill in
          break;
        case Leap::Bone::TYPE_PROXIMAL:
          finger_msg.poseProximalPhalange.pose.position.x = -(double)((bone.center().x)/1000) + xOffset_;
          finger_msg.poseProximalPhalange.pose.position.y = (double)((bone.center().z)/1000) + yOffset_;
          finger_msg.poseProximalPhalange.pose.position.z = (double)((bone.center().y)/1000) + zOffset_;
          //TODO fill in
          break;
        case Leap::Bone::TYPE_METACARPAL:
          finger_msg.poseMetacarpal.pose.position.x = -(double)((bone.center().x)/1000) + xOffset_;
          finger_msg.poseMetacarpal.pose.position.y = (double)((bone.center().z)/1000) + yOffset_;
          finger_msg.poseMetacarpal.pose.position.z = (double)((bone.center().y)/1000) + zOffset_;
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

    LeapController listener = LeapController(node_handle);
    Leap::Controller controller;
    // Have the sample listener receive events from the controller
    controller.addListener(listener);
    //Essentially, it runs everything as a background multi-thread.
    if (argc > 1 && strcmp(argv[1], "--bg") == 0)
        controller.setPolicy(Leap::Controller::POLICY_BACKGROUND_FRAMES);


    bool should_operate = true;
    while(ros::ok()  && should_operate){
      ros::spinOnce();
    }

    // Remove the sample listener when done
    controller.removeListener(listener);

    return 0;
}
