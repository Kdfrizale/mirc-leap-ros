/*
 Based off of the very elaborate sample provided by the Leap Motion Developement team
 with the necessary adjustments and added code to use for my controller's intents and purposes.
 All rights reserved to the their proper place.

 Author: Ricardo Flores
*/

#include <iostream>
#include <cstring>
#include "../include/Leap.h"

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

void SampleListener::onInit(const Controller& controller) {
    std::cout << "Initialized" << std::endl;
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
    for (HandList::const_iterator hl = hands.begin(); hl != hands.end(); ++hl) {

        // Get the first hand
        const Hand hand = *hl;
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

//Added the if statement to only print Thumb and Index finger.
            if(fingerNames[finger.type()] == fingerNames[0] || fingerNames[finger.type()] == fingerNames[1]){
                std::cout << std::string(4, ' ') <<  fingerNames[finger.type()]
                << " finger, id: " << finger.id()
                << ", length: " << finger.length()

                << "mm, width: " << finger.width() << std::endl;
            }

//The Start and End points of each bone is the frame coordinates of where the bones begin and end
//Finger Tip Thumb = Thumb Distal Start (x,y,z)
//Finger Tip Index = Index Distal Start (x,y,z)
//Finger Joint Thumb = Thumb Middle Start or Thumb Distal End (x,y,z)
//Finger Joint Index = Index Middle Start or Thumb Distal End (x,y,z)
//Finger End Thumb = Thumb Proximal Start or Thumb Middle End (x,y,z)
//Finger End Index = Index Proximal End (Maybe another alternative if the bone length is too interfering)

            // Get finger bones
            for (int b = 0; b < 4; ++b) {
                Bone::Type boneType = static_cast<Bone::Type>(b);
                Bone bone = finger.bone(boneType);

//Added another if statement to only print the Proximal, Middle, and Distal Bone.
                if(fingerNames[finger.type()] == fingerNames[0] || fingerNames[finger.type()] == fingerNames[1]){
                    if(boneNames[boneType] == boneNames[3] || boneNames[boneType] == boneNames[2] || boneNames[boneType] == boneNames[1]){
                        std::cout << std::string(6, ' ') <<  boneNames[boneType]
                        << " bone, start: " << bone.prevJoint()
                        << ", end: " << bone.nextJoint()
                        << ", direction: " << bone.direction() << std::endl;
                    }
                }
            }
        }
    }

    // Get tools
//    const ToolList tools = frame.tools();
//    for (ToolList::const_iterator tl = tools.begin(); tl != tools.end(); ++tl) {
//        const Tool tool = *tl;
//        std::cout << std::string(2, ' ') <<  "Tool, id: " << tool.id()
//        << ", position: " << tool.tipPosition()
//        << ", direction: " << tool.direction() << std::endl;
//    }
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
