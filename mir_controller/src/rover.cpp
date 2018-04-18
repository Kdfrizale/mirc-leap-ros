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
#include <mir_controller/rover.h>

Rover::Rover(ros::NodeHandle &nh):ControlledRobot(nh){
  std::string output_topic_name;
  nh_.getParam("output_publish_topic",output_topic_name);
  move_publisher_ = nh_.advertise<geometry_msgs::TwistStamped>(output_topic_name,10);
  ROS_INFO("Publishing TwistStamped move to topic [%s]", output_topic_name.c_str());

  nh_.param<double>("fwd_vel_scaling",     fwd_vel_scaling_,  3.0);
  nh_.param<double>("rev_vel_scaling",     rev_vel_scaling_,  1.5);
  nh_.param<double>("rotation_vel_scaling",rotation_scaling_, 4.5);

  // For a limited zone around origin, set the output to zero within deadband
  // between deadband and transition use f= 1/2 * ((x-deadband)/(transition-deadband))^2
  // beyond transition use f = 1/2 + x - transition
  nh_.param<double>("vel_deadband",    vel_deadband_,   0.01);
  nh_.param<double>("vel_transition",  vel_transition_, 0.03);
  nh_.param<double>("rot_deadband",    rot_deadband_,   0.01);
  nh_.param<double>("rot_transition_", rot_transition_, 0.03);
  nh_.param<double>("max_tracking_height", max_tracking_height_, 0.25);

  nh_.param<double>("vel_smoothing",   vel_smoothing_,  0.0);
  nh_.param<double>("rot_smoothing",   rot_smoothing_,  0.0);

  if (vel_smoothing_ > 0.9999){
    ROS_ERROR("Invalid vel_smoothing constant %f for [%s]", vel_smoothing_, output_topic_name.c_str());
    exit(-1);
  }
  if (rot_smoothing_ > 0.9999){
    ROS_ERROR("Invalid rot_smoothing constant %f for [%s]", rot_smoothing_, output_topic_name.c_str());
    exit(-1);
  }
}

Rover::~Rover(){
}

//Calculate require TwistStamped Message
bool Rover::calculateMove(){
  twist_.header.stamp = handPoseStamped_->header.stamp;
  if (handPoseStamped_->posePalm.position.z > max_tracking_height_)
  {
    ROS_INFO_THROTTLE(1.0, "Hand is too high -tracking is not allowed!");
    twist_.twist.linear.x = 0;
    twist_.twist.linear.y = 0;
    twist_.twist.linear.z = 0;
    twist_.twist.angular.x = 0;
    twist_.twist.angular.y = 0;
    twist_.twist.angular.z = 0;
    return true;
  }
  //Multiply the speed and turning rate by a scaling coefficient based on size of rover
  // This calculation provides a deadband around origin where the output is zero, and a transition zone
  if (handPoseStamped_->posePalm.position.x >= 0.0) {
    // Separate forward and reverse velocity scaling
    twist_.twist.linear.x  = command_calculation(handPoseStamped_->posePalm.position.x, vel_deadband_, vel_transition_,
                               fwd_vel_scaling_, twist_.twist.linear.x, vel_smoothing_);
  } else {
    twist_.twist.linear.x  = command_calculation(handPoseStamped_->posePalm.position.x, vel_deadband_, vel_transition_,
                              rev_vel_scaling_, twist_.twist.linear.x, vel_smoothing_);
  }

  twist_.twist.angular.z = command_calculation(handPoseStamped_->posePalm.position.y, rot_deadband_, rot_transition_,
                             rotation_scaling_, twist_.twist.angular.z, rot_smoothing_);
  return true;
}

void Rover::setDefaultMove(){
  twist_.header.stamp = ros::Time::now();
  twist_.twist.linear.x = 0;
  twist_.twist.linear.y = 0;
  twist_.twist.linear.z = 0;
  twist_.twist.angular.x = 0;
  twist_.twist.angular.y = 0;
  twist_.twist.angular.z = 0;
}

bool Rover::executeMove(){
  move_publisher_.publish(twist_);
  return true;
}

//Begin executing callback functions for subscriptions
void Rover::beginListening(){
  while (ros::ok()){
    //ROS_DEBUG_THROTTLE(1,"Waiting for input...");
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
