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
#include <arm_mimic_capstone/HandStampedPose.h>
#include <geometry_msgs/PoseStamped.h>
#include "../include/tf/LinearMath/Matrix3x3.h"
#include <boost/shared_ptr.hpp>

using namespace Leap;



class SampleListener : public Listener {
public:
    virtual void onInit(const Controller&);
    virtual void onConnect(const Controller&);
    virtual void onDisconnect(const Controller&);
    virtual void onExit(const Controller&);
    virtual void onFrame(const Controller&);
    virtual void onFocusGained(const Controller&);
    virtual void onFocusLost(const Controller&);
    virtual void onDeviceChange(const Controller&);
    virtual void onServiceConnect(const Controller&);
    virtual void onServiceDisconnect(const Controller&);

    //boost::shared_ptr <ros::NodeHandle> h;
    //ros::NodeHandle * h;
    // ros::Publisher publish;
private:
};

const std::string fingerNames[] = {"Thumb", "Index", "Middle", "Ring", "Pinky"};
const std::string boneNames[] = {"Metacarpal", "Proximal", "Middle", "Distal"};
const std::string stateNames[] = {"STATE_INVALID", "STATE_START", "STATE_UPDATE", "STATE_END"};
ros::Publisher publish;

void SampleListener::onInit(const Controller& controller) {
    std::cout << "Initialized" << std::endl;
    //h = boost::make_shared<ros::NodeHandle>("myNodeHandle");
    //h = ros::NodeHandle("myNodeHandle");
    //publish = h->advertise<arm_mimic_capstone::HandStampedPose>("/handPoseTopic", 1);

}

void SampleListener::onConnect(const Controller& controller) {
    std::cout << "Connected" << std::endl;

//Added this in to stop the device from recording frames before the user has indicated to start recording. (Hopefully it works. WIP)

//    controller.enableGesture(Gesture::TYPE_CIRCLE);
//    controller.enableGesture(Gesture::TYPE_KEY_TAP);
//    controller.enableGesture(Gesture::TYPE_SCREEN_TAP);
//    controller.enableGesture(Gesture::TYPE_SWIPE);
}

void SampleListener::onDisconnect(const Controller& controller) {
    // Note: not dispatched when running in a debugger.
    std::cout << "Disconnected" << std::endl;
}

void SampleListener::onExit(const Controller& controller) {
    std::cout << "Exited" << std::endl;
}

void SampleListener::onFrame(const Controller& controller) {
    // Get the most recent frame and report some basic information
    const Frame frame = controller.frame();
    std::cout << "Frame id: " << frame.id()
//                << ", timestamp: " << frame.timestamp()
//                << ", hands: " << frame.hands().count()
//    << ", extended fingers: " << frame.fingers().extended().count()
//                << ", tools: " << frame.tools().count()
//    << ", gestures: " << frame.gestures().count() << std::endl;
    << std::endl;

    HandList hands = frame.hands();
    //Struct of the 3 points that I need to send via ROS MSG
    geometry_msgs::PoseStamped sensedPoseTip1;//Thumb Finger tip
    geometry_msgs::PoseStamped sensedPoseTip2;//Index Finger tip
    geometry_msgs::PoseStamped sensedPosePalm;//wrist

    for (HandList::const_iterator hl = hands.begin(); hl != hands.end(); ++hl) {
        // Get the first hand
        const Hand hand = *hl;
        if(hand == frame.hands() [0]){
            std::string handType = hand.isLeft() ? "Left hand" : "Right hand";
            std::cout << std::string(2, ' ') << handType << ", id: " << hand.id()
            << ", palm position: " << hand.palmPosition() << std::endl;

            // Get the hand's normal vector and direction
            const Vector normal = hand.palmNormal();
            const Vector direction = hand.direction();

            // Calculate the hand's pitch, roll, and yaw angles
            std::cout << std::string(2, ' ') <<  "pitch: " << direction.pitch() * RAD_TO_DEG << " degrees, "
            << "roll: " << normal.roll() * RAD_TO_DEG << " degrees, "
            << "yaw: " << direction.yaw() * RAD_TO_DEG << " degrees" << std::endl;

            // Get the Arm bone
            Arm arm = hand.arm();
            std::cout << std::string(2, ' ') <<  "Arm direction: " << arm.direction()
            << " wrist position: " << arm.wristPosition()
            << " elbow position: " << arm.elbowPosition() << std::endl;

            // Get fingers
            const FingerList fingers = hand.fingers();
            for (FingerList::const_iterator fl = fingers.begin(); fl != fingers.end(); ++fl) {
                const Finger finger = *fl;

//FIRST HAND ONLY
//Added the if statement to only print Thumb and Index finger.
                if(fingerNames[finger.type()] == fingerNames[0] || fingerNames[finger.type()] == fingerNames[1]
                   || fingerNames[finger.type()] == fingerNames[2]){
                    std::cout << std::string(4, ' ') <<  fingerNames[finger.type()]
                    << " finger, id: " << finger.id()
                    << ", length: " << finger.length()
                    << "mm, width: " << finger.width() << std::endl;
                }

//The Start and End points of each bone is the frame coordinates of where the bones begin and end
//Finger Tip Thumb = Thumb Distal Start (x,y,z)
//Finger Tip Index = Index Distal Start (x,y,z)
                //ONLY SEND ONE OF THE BELOW POINTS
//Middle Finger End = Middle Proximal End (x,y,z)(Yaw, Roll, Pitch)
//Center of Palm = (Middle Metacarpal Start + End)/2 (x,y,z) (Yaw, Roll, Pitch)
//Center End of Hand/Wrist = Middle Metacarpal End (x,y,z) (Yaw, Roll, Pitch)

            // Get finger bones
                for (int b = 0; b < 4; ++b) {
                    Bone::Type boneType = static_cast<Bone::Type>(b);
                    Bone bone = finger.bone(boneType);

//Added code below to only print the Proximal, Metacarpal, and Distal Bone.
                    if(fingerNames[finger.type()] == fingerNames[3]){
                        if(boneNames[boneType] == boneNames[0]){
                            //Center of Palm (Active)
                            std::cout << std::string(6, ' ') <<  "Center of Palm: ("
                            << ((bone.prevJoint().x + bone.nextJoint().x)/2) << ", "
                            << ((bone.prevJoint().y + bone.nextJoint().y)/2) << ", "
                            << ((bone.prevJoint().z + bone.nextJoint().z)/2) << ")"
                            << " (" << bone.prevJoint().yaw()
                            << ", " << bone.prevJoint().roll()
                            << ", " << bone.prevJoint().pitch() << ")" << std::endl
                            //Center End of Hand (Inactive)
                            << std::string(6, ' ') << "Center End of Hand: "
                            << bone.nextJoint() << " (" << bone.prevJoint().yaw()
                            << ", " << bone.prevJoint().roll()
                            << ", " << bone.prevJoint().pitch() << ")" << std::endl;

//Send the data via ROS MSG in a created struct
                            //arm_mimic_capstone::HandStampedPose msg;

                            //Copied from Kyle Frizzell's myDummyPub publisher
                            //Values in meters. COVERT FROM MILLIMETERS(Leap) TO METERS
                            //geometry_msgs::PoseStamped sensedPosePalm;
                            sensedPosePalm.header.frame_id = "m1n6s200_link_base";
                            double palmX = (bone.center().z/1000);
                            double palmY = (bone.center().x/1000);
                            double palmZ = (bone.center().y/1000);
                            sensedPosePalm.pose.position.x = palmX;
                            sensedPosePalm.pose.position.y = palmY;
                            sensedPosePalm.pose.position.z = palmZ;
                            double yaw = (bone.prevJoint().yaw());
                            double roll = (bone.prevJoint().roll());
                            double pitch = (bone.prevJoint().pitch());

                            //Example of how to do x,y,z,w thing. implement with leap number system.
                            // tf::Matrix3x3 obs_mat;
                            // obs_mat.setEulerYPR(Yaw,Pitch,Roll);i
                            //
                            // tf::Quaternion q_tf;
                            // obs_mat.getRotation(q_tf);
                            // INS_msg.orientation.x = q_tf.getX();
                            // INS_msg.orientation.y = q_tf.getY();
                            // INS_msg.orientation.z = q_tf.getZ();
                            // INS_msg.orientation.w = q_tf.getW();
                            tf::Matrix3x3 space;
                            space.setEulerYPR(yaw,pitch,roll);
                            tf::Quaternion quater;
                            space.getRotation(quater);

                            sensedPosePalm.pose.orientation.x = quater.getX();
                            sensedPosePalm.pose.orientation.y = quater.getY();
                            sensedPosePalm.pose.orientation.z = quater.getZ();
                            sensedPosePalm.pose.orientation.w = quater.getW();
                            ROS_INFO("DIS IS ROLL [%f]", sensedPosePalm.pose.orientation.w);
                        }else if(boneNames[boneType] == boneNames[1]){
                            //End of Middle Finger (Inactive)
                            std::cout << std::string(6, ' ') <<  "End of Middle Finger: "
                            << bone.nextJoint() << " (" << bone.prevJoint().yaw()
                            << ", " << bone.prevJoint().roll()
                            << ", " << bone.prevJoint().pitch() << ")" << std::endl;
                        }
                    }
                    if(fingerNames[finger.type()] == fingerNames[0]){
                      std::cout << std::string(6, ' ') << "Finger Tip: " << bone.prevJoint() << std::endl;
                        if(boneNames[boneType] == boneNames[3]){
                          //geometry_msgs::PoseStamped sensedPoseTip1;
                              sensedPoseTip1.header.frame_id = "m1n6s200_link_base";
                              sensedPoseTip1.pose.position.x = (double)((bone.prevJoint().z)/1000);
                              sensedPoseTip1.pose.position.y = (double)((bone.prevJoint().x)/1000);
                              sensedPoseTip1.pose.position.z = (double)((bone.prevJoint().y)/1000);
                        }
                    }
                    if(fingerNames[finger.type()] == fingerNames[1]){
                      std::cout << std::string(6, ' ') << "Finger Tip: " << bone.prevJoint() << std::endl;
                        if(boneNames[boneType] == boneNames[3]){
                            //geometry_msgs::PoseStamped sensedPoseTip2;//Index Finger tip
                            sensedPoseTip2.header.frame_id = "m1n6s200_link_base";
                            sensedPoseTip2.pose.position.x = (double)((bone.prevJoint().z)/1000);
                            sensedPoseTip2.pose.position.y = (double)((bone.prevJoint().x)/1000);
                            sensedPoseTip2.pose.position.z = (double)((bone.prevJoint().y)/1000);
                        }
                    }

                    // if(fingerNames[finger.type()] == fingerNames[0] || fingerNames[finger.type()] == fingerNames[1]){
                    //     if(boneNames[boneType] == boneNames[3]){
                    //         std::cout << std::string(6, ' ') << "Finger Tip: " << bone.prevJoint() << std::endl;
                    //         if(fingerNames[finger.type()] == fingerNames[0]){//Thumb Finger MSG
                    //           // std::cout << "Trialz" << bone.prevJoint() << std::endl;
                    //           std::cout << (double)((bone.prevJoint().x)/1000);
                    //           ROS_INFO("hey bitch [%f]", (double)((bone.prevJoint().x)/1000));
                    //           geometry_msgs::PoseStamped sensedPoseTip1;
                    //           sensedPoseTip1.header.frame_id = "m1n6s200_link_base";
                    //           sensedPoseTip1.pose.position.x = (double)((bone.prevJoint().x)/1000);
                    //           sensedPoseTip1.pose.position.y = (double)((bone.prevJoint().y)/1000);
                    //           sensedPoseTip1.pose.position.z = (double)((bone.prevJoint().z)/1000);
                    //         }else if(fingerNames[finger.type()] == fingerNames[1]){
                    //           // std::cout << "This is The Index!" << std::endl;//Index Finger MSG
                    //           geometry_msgs::PoseStamped sensedPoseTip2;//Index Finger tip
                    //           sensedPoseTip2.header.frame_id = "m1n6s200_link_base";
                    //           sensedPoseTip2.pose.position.x = (double)((bone.prevJoint().x)/1000);
                    //           sensedPoseTip2.pose.position.y = (double)((bone.prevJoint().y)/1000);
                    //           sensedPoseTip2.pose.position.z = (double)((bone.prevJoint().z)/1000);
                    //         }
                        // }
                    // }
                }
            }
        }
        arm_mimic_capstone::HandStampedPose thread;
        thread.poseTip2 = sensedPoseTip2;
        thread.poseTip1 = sensedPoseTip1;
        thread.posePalm = sensedPosePalm;
        publish.publish(thread);
    }
}

void SampleListener::onFocusGained(const Controller& controller) {
    std::cout << "Focus Gained" << std::endl;
}

void SampleListener::onFocusLost(const Controller& controller) {
    std::cout << "Focus Lost" << std::endl;
}

void SampleListener::onDeviceChange(const Controller& controller) {
    std::cout << "Device Changed" << std::endl;
    const DeviceList devices = controller.devices();

    for (int i = 0; i < devices.count(); ++i) {
        std::cout << "id: " << devices[i].toString() << std::endl;
        std::cout << "  isStreaming: " << (devices[i].isStreaming() ? "true" : "false") << std::endl;
    }
}

void SampleListener::onServiceConnect(const Controller& controller) {
    std::cout << "Service Connected" << std::endl;
}

void SampleListener::onServiceDisconnect(const Controller& controller) {
    std::cout << "Service Disconnected" << std::endl;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "leap_controller_node");
    ros::NodeHandle h;
    publish = h.advertise<arm_mimic_capstone::HandStampedPose>("/handPoseTopic", 1);
    //ros::NodeHandle h;
    std::cout << "Touch Trump's Hair to start (Enter).."<< std::endl;
    std::cin.get();
    // Create a sample listener and controller
    SampleListener listener;
    Controller controller;

    // Have the sample listener receive events from the controller
    controller.addListener(listener);

    if (argc > 1 && strcmp(argv[1], "--bg") == 0)
        controller.setPolicy(Leap::Controller::POLICY_BACKGROUND_FRAMES);

//Think of a way to stop Leap from recording Frames until "Enter" button is pressed.

    // Keep this process running until Enter is pressed
    std::cout << "Press Enter to quit..." << std::endl;
    std::cin.get();

    // Remove the sample listener when done
    controller.removeListener(listener);

    return 0;
}
