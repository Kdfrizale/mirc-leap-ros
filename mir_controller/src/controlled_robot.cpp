/*********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018
 * Capable Humanitarian Robotics and Intelligent Systems Lab (CHRISLab)
 * Christopher Newport University
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
#include <mir_controller/controlled_robot.h>

ControlledRobot::ControlledRobot(ros::NodeHandle &nh):nh_(nh){
  std::string input_pose_topic_name;
  nh_.getParam("leap_output_pose_topic",input_pose_topic_name);
  ROS_INFO("Listening to topic [%s] for input pose", input_pose_topic_name.c_str());
  sub_leap_hand_ = nh_.subscribe(input_pose_topic_name,1,&ControlledRobot::updatePoseValues, this);

  receivedNewPose_ = false;
}

ControlledRobot::~ControlledRobot(){
}

bool ControlledRobot::calculateMove(){
  }

bool ControlledRobot::executeMove(){
}

//Sets the desired poseTargets to the received input poses
void ControlledRobot::updatePoseValues(const leap_interface::HandPoseStamped::ConstPtr& msg){
  //ROS_DEBUG_THROTTLE(1,"Received Input. Now processing...");
  //Record poses received from the ROS Topic
  handPoseStamped_ = msg;
  receivedNewPose_ = true;
}

//Begin executing callback functions for subscriptions
void ControlledRobot::beginListening(){
  while (ros::ok()){
    ROS_DEBUG_THROTTLE(1,"Waiting for input...");
    ros::spinOnce();
    if(receivedNewPose_){
      receivedNewPose_ = false;
      if(this->calculateMove()){
        this->executeMove();
      }
    }
  }
}
