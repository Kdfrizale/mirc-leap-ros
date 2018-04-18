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
#include <robotic_arm.h>

RoboticArm::RoboticArm(ros::NodeHandle &nh):ControlledRobot(nh){
  //TODO make these parameters
  std::string tempParam;
  nh_.getParam("moveit_planning_group_arm_name",tempParam);
  group_ = new moveit::planning_interface::MoveGroupInterface(tempParam);
  group_->setPlanningTime(0.05);

  nh_.getParam("moveit_planning_group_gripper_name",tempParam);
  gripper_group_ = new moveit::planning_interface::MoveGroupInterface(tempParam);
  gripper_group_->setPlanningTime(0.05);

  nh_.getParam("moveit_gripper_close_pose_name",tempParam);
  closedJointValues_ = gripper_group_->getNamedTargetValues(tempParam);

  nh_.getParam("moveit_gripper_open_pose_name",tempParam);
  openedJointValues_ = gripper_group_->getNamedTargetValues(tempParam);
}

RoboticArm::~RoboticArm(){
    delete group_;
    delete gripper_group_;
}

//Sets the desired poseTargets to the received input poses
void RoboticArm::updatePoseValues(const leap_interface::HandPoseStamped::ConstPtr& msg){
  ROS_INFO_THROTTLE(1,"Received Input. Now processing...");
  //Record poses received from the ROS Topic
  sensedPosePalm_ = msg->posePalm;
  auto index_finger = std::find_if(msg->poseFingers.begin(),msg->poseFingers.end(), [](const leap_interface::FingerPose& finger)
                                                                    {return finger.name == "index";});
  auto thumb_finger = std::find_if(msg->poseFingers.begin(),msg->poseFingers.end(), [](const leap_interface::FingerPose& finger)
                                                                    {return finger.name == "thumb";});
  sensedPoseTip2_ = index_finger->poseProximalPhalange;
  sensedPoseTip1_ = thumb_finger->poseDistalPhalange;
  receivedNewPose_ = true;
}

bool RoboticArm::calculateMove(){
  //Calculate require joint values for fingers
  double ratioOpen = std::min(1.0,calculateDistanceBetweenPoints(sensedPoseTip1_,sensedPoseTip2_)/DISTANCE_BETWEEN_USER_FINGERS_FULL_OPEN);//the 0.02 is for skin and tissue between bones
  std::map<std::string, double> jointValueGoal = closedJointValues_;
  ROS_WARN("The distance between fingers is: [%f]", calculateDistanceBetweenPoints(sensedPoseTip1_,sensedPoseTip2_));

  std::map<std::string, double>::iterator it_jv = jointValueGoal.begin();
  std::map<std::string, double>::iterator it_open = openedJointValues_.begin();
  std::map<std::string, double>::iterator it_close = closedJointValues_.begin();
  while (it_jv != jointValueGoal.end() || it_open != openedJointValues_.end() || it_close != closedJointValues_.end()){
    //Set jointValueGoal to be the ratio between the robot's MoveIt defined Open and Close preset Poses
    it_jv->second = calculateValueBetweenRange(it_open->second, it_close->second, 1-ratioOpen);
    it_jv++;
    it_open++;
    it_close++;
  }
  group_->setPoseTarget(sensedPosePalm_);
  //group_->setPoseTarget(group_->getRandomPose());
  gripper_group_->setJointValueTarget(jointValueGoal);
  return (group_->plan(planArm_) && gripper_group_->plan(planGripper_));
}

bool RoboticArm::executeMove(){
  //printPlanInfo(planArm_);
  planArm_.trajectory_.joint_trajectory.header.stamp = ros::Time::now() + ros::Duration(8.1);
  return (group_->execute(planArm_) && gripper_group_->execute(planGripper_));
}

void RoboticArm::printPlanInfo(moveit::planning_interface::MoveGroupInterface::Plan aPlan){
  //ROS_INFO("ROS TIME: [%f]", ros::Time::now().toSec());
  ROS_INFO("PLAN INFO: the planning time is [%f]", aPlan.planning_time_);
  ROS_INFO("PLAN INFO: the time stamp is [%f]", aPlan.trajectory_.joint_trajectory.header.stamp.sec);
  for (std::string name : aPlan.trajectory_.joint_trajectory.joint_names){
    ROS_INFO("PLAN INFO: Joint Name: [%s]", name.c_str());
  }
  for (trajectory_msgs::JointTrajectoryPoint aPoint : aPlan.trajectory_.joint_trajectory.points){
    ROS_INFO("PLAN INFO: Time from start for a point: [%f]", aPoint.time_from_start.toSec());
  }

}

int main(int argc, char** argv){
  ros::init(argc, argv, "robotic_arm_node");
  ros::NodeHandle node("~");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ROS_INFO("RoboticArm Node has started");

  ros::WallDuration(10.0).sleep();//allow for Rviz/MoveIt to start before continuing
  RoboticArm myArm = RoboticArm(node);
  myArm.beginListening();

  ros::spin();
  return 0;
}
