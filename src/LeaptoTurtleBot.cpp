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
// #include <arm_mimic_capstone/HandStampedPose.h>
#include <geometry_msgs/PoseStamped.h>
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
private:
};

const std::string fingerNames[] = {"Thumb", "Index", "Middle", "Ring", "Pinky"};
const std::string boneNames[] = {"Metacarpal", "Proximal", "Middle", "Distal"};
const std::string stateNames[] = {"STATE_INVALID", "STATE_START", "STATE_UPDATE", "STATE_END"};
ros::Publisher publish;

void SampleListener::onInit(const Controller& controller) {
    std::cout << "Initialized" << std::endl;
}

void SampleListener::onConnect(const Controller& controller) {
    std::cout << "Connected" << std::endl;
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
    std::cout << "Frame id: " << frame.id() << std::endl;

    //All hands recorded on the frame
    HandList hands = frame.hands();

    //Struct of the 1 point that I need to send via ROS MSG
//    geometry_msgs::PoseStamped sensedPoseTip1;//Center of Hand

    for (HandList::const_iterator hl = hands.begin(); hl != hands.end(); ++hl) {
        // Get the first hand
        const Hand hand = *hl;

        //FIRST HAND ONLY
        if(hand == frame.hands() [0]){
            std::cout << "palm position (x,z): (" << hand.palmPosition().x << ", " << hand.palmPosition().z << ")" << std::endl;

                            //Values in meters (mm/1000).
//                            sensedPosePalm.header.frame_id = "m1n6s200_link_base";
//                            sensedPosePalm.pose.position.x = -(hand.palmPosition().x/1000);
//                            sensedPosePalm.pose.position.y = (hand.palmPosition().z/1000);
                }
            }
        }
        //Puts everything in one nice package and sends the data
//        arm_mimic_capstone::HandStampedPose thread;
//        thread.posePalm = sensedPosePalm;
//        publish.publish(thread);
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

    //Defines where the data is to be sent to.
//    ros::init(argc, argv, "leap_controller_node");
    ros::NodeHandle h;
//    publish = h.advertise<arm_mimic_capstone::HandStampedPose>("/handPoseTopic", 1);

    //Heh. Funny.
    //Prevents anything from happening until the user is ready.
    std::cout << "Type in the answer to life, the universe, and everything to start (   Enter).."<< std::endl;
    std::cin.get();

    // Create a sample listener and controller
    SampleListener listener;
    Controller controller;

    // Have the sample listener receive events from the controller
    controller.addListener(listener);

    //Essentially, it runs everything as a background multi-thread.
    if (argc > 1 && strcmp(argv[1], "--bg") == 0)
        controller.setPolicy(Leap::Controller::POLICY_BACKGROUND_FRAMES);

    // Keep this process running until Enter is pressed
    std::cout << "Press Enter to quit..." << std::endl;
    std::cin.get();

    // Remove the sample listener when done
    controller.removeListener(listener);

    return 0;
}
