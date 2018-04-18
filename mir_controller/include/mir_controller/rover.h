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
#include <geometry_msgs/TwistStamped.h>

class Rover:public ControlledRobot {
public:
  Rover(ros::NodeHandle &nh);
  ~Rover();
  void beginListening();

private:
  ros::Publisher move_publisher_;
  geometry_msgs::TwistStamped twist_;

  double fwd_vel_scaling_;
  double rev_vel_scaling_;
  double rotation_scaling_;

  // For a limited zone around origin, set the output to zero within deadband
  // between deadband and transition use f= 1/2 * ((x-deadband)/(transition-deadband))^2
  // beyond transition use f = 1/2 + x - transition
  double vel_deadband_;
  double vel_transition_;
  double rot_deadband_;
  double rot_transition_;

  // Smoothing constant (0.0 use raw calculation, < 1.0 blend prior)
  double vel_smoothing_;
  double rot_smoothing_;

  inline double command_calculation(const double& x, const double& deadband, const double& transition,
                                    const double& scaling, const double& prior, const double& smoothing)
  {
    if (fabs(x) <= deadband) return 0.0;
    double val;
    if (fabs(x) < transition)
    {
      double tmp = fabs(x) - deadband;
      double tmp2 = tmp/(transition-deadband);
      val = 0.5*tmp2*tmp2;
    } else {
      // transition zone ends with a slope of 1. and value of 1/2
      val = 0.5 + fabs(x) - transition;
    }

    // Apply the scaling factor
    val *= scaling;

    // retain the original sign of the x input
    if (x < 0.0) val *= -1.0;

    if (smoothing <= 0.0) return val; // use raw value

    return prior*smoothing + (1.0-smoothing)*val;

  }

  void setDefaultMove();
  bool calculateMove();
  bool executeMove();
};
