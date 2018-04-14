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
#include <rover.h>

Rover::Rover(ros::NodeHandle &nh):ControlledRobot(nh){
  std::string output_topic_name;
  nh_.getParam("output_publish_topic",output_topic_name);
  move_publisher_ = nh_.advertise<geometry_msgs::TwistStamped>(output_topic_name,10);
  ROS_INFO("Publishing TwistStamped move to topic [%s]", output_topic_name.c_str());
}

Rover::~Rover(){
}

//Calculate require TwistStamped Message
bool Rover::calculateMove(){
  //Multiply the speed and turning rate by a scale coefficient based on size of rover
  goal_.twist.linear.x  = sensedPosePalm_.position.x * 3;
  goal_.twist.angular.z = sensedPosePalm_.position.y * 4.5;
  return true;
}

void Rover::setDefaultMove(){
  goal_.twist.linear.x = 0;
  goal_.twist.linear.y = 0;
  goal_.twist.linear.z = 0;
  goal_.twist.angular.x = 0;
  goal_.twist.angular.y = 0;
  goal_.twist.angular.z = 0;
}

bool Rover::executeMove(){
  move_publisher_.publish(goal_);
  return true;
}

//Begin executing callback functions for subscriptions
void Rover::beginListening(){
  while (ros::ok()){
    ROS_INFO_THROTTLE(1,"Waiting for input...");
    ros::spinOnce();
    if(receivedNewPose_){
      receivedNewPose_ = false;
      this->calculateMove();
    }
    else{
      //No Input Detected
      this->setDefaultMove();
    }
    this->executeMove();
    ros::WallDuration(0.1).sleep();
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "rover_node");
  ros::NodeHandle node("~");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ROS_INFO("Rover Node has started");

  Rover myRover = Rover(node);
  myRover.beginListening();

  ros::spin();
  return 0;
}
