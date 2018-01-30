/*
 Based off of the very elaborate sample provided by the Leap Motion Developement team
 with the necessary adjustments and added code to use for my controller's intents and purposes.
 All rights reserved to the their proper place.
 Author: Ricardo Flores
 */

#include <iostream>
#include <cstring>
#include "../include/Leap.h"
#include "ros/ros.h"
#include <leap_controller_capstone/HandPoseStamped.h>
#include <leap_controller_capstone/FingerPoseStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include "../include/tf/LinearMath/Matrix3x3.h"
#include <boost/shared_ptr.hpp>
#include "../include/tf/transform_datatypes.h"

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
private:
    ros::NodeHandle nh_;
    ros::Publisher hand_pose_publisher_;
    std::string hand_to_sense_;
    std::vector<std::string> fingers_to_track_;

    double xOffset_;
    double yOffset_;
    double zOffset_;

    geometry_msgs::PoseStamped sensedPosePalm_;//Center of the Palm Pose
    std::vector<leap_controller_capstone::FingerPoseStamped> sensedPoseFingers_;//Finger Poses

    Leap::Frame current_frame_;

    void processFrame();
    void processHand(const Leap::Hand& aHand);
    void processFinger(const Leap::Finger& aFinger);
    void translateToRosCoordinate();
    void publishHandPose();

};

const std::string fingerNames[] = {"Thumb", "Index", "Middle", "Ring", "Pinky"};
const std::string boneNames[] = {"Metacarpal", "Proximal", "Middle", "Distal"};
const std::string stateNames[] = {"STATE_INVALID", "STATE_START", "STATE_UPDATE", "STATE_END"};

LeapController::LeapController(ros::NodeHandle &nh):nh_(nh){
  std::string output_pose_topic_name;
  nh_.getParam("leap_output_pose_topic",output_pose_topic_name);
  ROS_INFO("Publishing Leap Hand Poses to topic [%s]", output_pose_topic_name.c_str());
  hand_pose_publisher_ = nh_.advertise<leap_controller_capstone::HandPoseStamped>(output_pose_topic_name,10);

  nh_.param<std::string>("hand_to_sense",hand_to_sense_, "first");
  //TODO add param logic to get an array of bone names to track
  //for now hard code
  fingers_to_track_.push_back("index");

  //TODO add param to get X,Y,Z, offset
  //for now hard code
  xOffset_ = 0.0;
  yOffset_ = 0.0;
  zOffset_ = 0.0;

}

void LeapController::onFrame(const Leap::Controller& controller){
  current_frame_ = controller.frame();
  processFrame();

}

void LeapController::processFrame(){
  enum HandToSense {first, second, left, right, both};
  //TODO determine which hand should be processed based on ROS param
  //Then process that hand
  Leap::HandList hands = current_frame_.hands();
  for (Leap::HandList::const_iterator hl = hands.begin(); hl != hands.end(); ++hl) {
      const Leap::Hand hand = *hl;
      processHand(hand);
    }
}

void LeapController::processHand(const Leap::Hand& aHand){
  const Leap::Vector normal = aHand.palmNormal();
  const Leap::Vector direction = aHand.direction();
  //TODO add logic code to get important information on the palm here
  //Also set HandPoseStamped.name = "left" or "right"

  const Leap::FingerList fingers = aHand.fingers();
  for (Leap::FingerList::const_iterator fl = fingers.begin(); fl != fingers.end(); ++fl) {
      const Leap::Finger finger = *fl;
      processFinger(finger);
    }
    publishHandPose();
}

void LeapController::processFinger(const Leap::Finger& aFinger){
  leap_controller_capstone::FingerPoseStamped finger_msg;
  switch(aFinger.type()){
    case Leap::Finger::TYPE_THUMB:
      finger_msg.name = "thumb";
    case Leap::Finger::TYPE_INDEX:
      finger_msg.name = "index";
    case Leap::Finger::TYPE_MIDDLE:
      finger_msg.name = "middle";
    case Leap::Finger::TYPE_RING:
      finger_msg.name = "ring";
    case Leap::Finger::TYPE_PINKY:
      finger_msg.name = "pinky";
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
          //Should it translate to ROS Space here?
            // e.g. divide by 1000, negate the axis, add a user-set offset?

        case Leap::Bone::TYPE_INTERMEDIATE:
          finger_msg.poseIntermediatePhalange.pose.position.x = -(double)((bone.center().x)/1000) + xOffset_;
          //TODO fill in

        case Leap::Bone::TYPE_PROXIMAL:
          finger_msg.poseProximalPhalange.pose.position.x = -(double)((bone.center().x)/1000) + xOffset_;
          //TODO fill in

        case Leap::Bone::TYPE_METACARPAL:
          finger_msg.poseMetacarpal.pose.position.x = -(double)((bone.center().x)/1000) + xOffset_;
          //TODO fill in
      }
    }//end for loop
    sensedPoseFingers_.push_back(finger_msg);
}

void LeapController::publishHandPose(){
  leap_controller_capstone::HandPoseStamped msg;
  msg.posePalm = sensedPosePalm_;
  msg.poseFingers = sensedPoseFingers_;
  hand_pose_publisher_.publish(msg);
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
