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

    geometry_msgs::PoseStamped sensedPosePalm_;//Center of the Palm Pose
    std::vector<geometry_msgs::PoseStamped> sensedPoseFingerTips_;//Finger Tip Poses
    std::vector<geometry_msgs::PoseStamped> sensedPoseMisc_;//Anything Extra

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
    publishHandPose();
}

void LeapController::processHand(const Leap::Hand& aHand){
  const Leap::Vector normal = aHand.palmNormal();
  const Leap::Vector direction = aHand.direction();
  //TODO add logic code to get important information on the palm here

  const Leap::FingerList fingers = aHand.fingers();
  for (Leap::FingerList::const_iterator fl = fingers.begin(); fl != fingers.end(); ++fl) {
      const Leap::Finger finger = *fl;
      processFinger(finger);
    }

}

void LeapController::processFinger(const Leap::Finger& aFinger){
  //TODO add logic code to get all the information for the finger
}

void LeapController::publishHandPose(){
  leap_controller_capstone::HandPoseStamped msg;
  msg.posePalm = sensedPosePalm_;
  msg.poseFingerTips = sensedPoseFingerTips_;
  msg.poseMisc = sensedPoseMisc_;
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
