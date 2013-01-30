/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2011, PAL Robotics, S.L.
 *  All rights reserved.
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
 *   * Neither the name of PAL Robotics, S.L. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
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
 */

/** \author Adolfo Rodriguez Tsouroukdissian. */

#ifndef PAL_CONTROL_JOINT_SPLINE_TRAJECTORY_CONTROLLER_ROS_H_
#define PAL_CONTROL_JOINT_SPLINE_TRAJECTORY_CONTROLLER_ROS_H_

// C++ standard
#include <string>

// Boost
#include <boost/scoped_ptr.hpp>

// OrocosRTT
#include <rtt/TaskContext.hpp>
#include <rtt/Method.hpp>
#include <rtt/Ports.hpp>
#include <rtt/Property.hpp>

// ROS
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <pr2_controllers_msgs/QueryTrajectoryState.h>
#include <actionlib/server/action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>

namespace pal
{

namespace control
{

class JointTrajectoryActionControllerRos : public RTT::TaskContext
{
public:
  JointTrajectoryActionControllerRos(const std::string& name);
  virtual ~JointTrajectoryActionControllerRos();

protected:
  virtual bool configureHook();
  virtual bool startHook();
  virtual void updateHook();
  virtual void stopHook();

private:
  typedef trajectory_msgs::JointTrajectoryConstPtr JointTrajectoryConstPtr;
  typedef pr2_controllers_msgs::JointTrajectoryControllerState JointTrajectoryControllerState;
  typedef pr2_controllers_msgs::JointTrajectoryControllerStateConstPtr JointTrajectoryControllerStateConstPtr;
  typedef pr2_controllers_msgs::QueryTrajectoryState::Request  QueryStateRequest;
  typedef pr2_controllers_msgs::QueryTrajectoryState::Response QueryStateResponse;

  // Action typedefs for the original PR2 specific joint trajectory action
  typedef actionlib::ActionServer<pr2_controllers_msgs::JointTrajectoryAction> JTAS;
  typedef JTAS::GoalHandle GoalHandle;

  // Action typedefs for the new follow joint trajectory action
  typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> FJTAS;
  typedef FJTAS::GoalHandle GoalHandleFollow;

  typedef std::vector<std::string> GetJointNamesMethodType(void);
  typedef bool QueryStateMethodType(QueryStateRequest&, QueryStateResponse&);
  typedef void UpdateCommandMethodType(JointTrajectoryConstPtr);
  typedef void JTASGoalType(GoalHandle, ros::NodeHandle);
  typedef void JTASCancelType(GoalHandle);
  typedef void FJTASGoalType(GoalHandleFollow, ros::NodeHandle);
  typedef void FJTASCancelType(GoalHandleFollow);

  ros::NodeHandle nh_;
  ros::CallbackQueue callbackQueue_;

  RTT::Property<std::string> commandTopicName_;         ///< Name of input ROS topic containing the trajectory command.
  RTT::Property<std::string> controllerStateTopicName_; ///< Name of output ROS topic containing the controller state.
  RTT::Property<std::string> queryStateServiceName_;    ///< Name of ROS service for querying trajectory state.
  RTT::Property<std::string> jointTrajectoryActionName_;       ///< Name of ROS JointTrajectoryAction server.
  RTT::Property<std::string> followJointTrajectoryActionName_; ///< Name of ROS FollowJointTrajectoryAction server.

  /// Query list of controller joint names.
  RTT::Method<GetJointNamesMethodType> getJointNamesMethod_;

  /// Query where the controller setpoint will be at a desired time.
  bool queryState(QueryStateRequest &req, QueryStateResponse &resp);
  RTT::Method<QueryStateMethodType> queryStateMethod_;
  ros::ServiceServer queryStateService_;

  /// Process command containing trajectory to follow.
  void commandCallback(const JointTrajectoryConstPtr &msg);
  RTT::Method<UpdateCommandMethodType> updateCommandMethod_;
  ros::Subscriber commandSubscriber_;

  ros::Publisher controllerStatePublisher_;

  // Action servers and supporting infrastructure
  boost::scoped_ptr<JTAS>  actionServer_;
  boost::scoped_ptr<FJTAS> actionServerFollow_;

  void goalCB(GoalHandle gh);
  void cancelCB(GoalHandle gh);
  void goalCBFollow(GoalHandleFollow gh);
  void cancelCBFollow(GoalHandleFollow gh);

  RTT::Method<JTASGoalType>    goalCBMethod_;
  RTT::Method<JTASCancelType>  cancelCBMethod_;
  RTT::Method<FJTASGoalType>   goalCBFollowMethod_;
  RTT::Method<FJTASCancelType> cancelCBFollowMethod_;

  RTT::ReadBufferPort<JointTrajectoryControllerStateConstPtr> controllerState_; ///< Feedback for client.

  JointTrajectoryControllerStateConstPtr currentControllerState_;

  bool initOrocosInterface();
  bool initRosInterface();
};

inline bool JointTrajectoryActionControllerRos::queryState(QueryStateRequest &req, QueryStateResponse &resp)
{
  if (!queryStateMethod_.ready()) {return false;}
  return queryStateMethod_(req, resp);
}

inline void JointTrajectoryActionControllerRos::commandCallback(const JointTrajectoryConstPtr &msg)
{
  if (isConfigured() && updateCommandMethod_.ready())
  {
    updateCommandMethod_(msg);
  }
}

inline void JointTrajectoryActionControllerRos::goalCB(GoalHandle gh)
{
  if (isConfigured() && goalCBMethod_.ready())
  {
    goalCBMethod_(gh, nh_);
  }
}

inline void JointTrajectoryActionControllerRos::cancelCB(GoalHandle gh)
{
  if (isConfigured() && cancelCBMethod_.ready())
  {
    cancelCBMethod_(gh);
  }
}

inline void JointTrajectoryActionControllerRos::goalCBFollow(GoalHandleFollow gh)
{
  if (isConfigured() && goalCBFollowMethod_.ready())
  {
    goalCBFollowMethod_(gh, nh_);
  }
}

inline void JointTrajectoryActionControllerRos::cancelCBFollow(GoalHandleFollow gh)
{
  if (isConfigured() && cancelCBFollowMethod_.ready())
  {
    cancelCBFollowMethod_(gh);
  }
}

} // control

} // pal

#endif // PAL_CONTROL_JOINT_SPLINE_TRAJECTORY_CONTROLLER_ROS_H_