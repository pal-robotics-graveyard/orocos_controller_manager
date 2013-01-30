/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  Copyright (c) 2012, PAL Robotics, S.L.
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

/** \author Stuart Glaser, Adolfo Rodriguez Tsouroukdissian. */

#ifndef PAL_CONTROL_JOINT_SPLINE_TRAJECTORY_CONTROLLER_H_
#define PAL_CONTROL_JOINT_SPLINE_TRAJECTORY_CONTROLLER_H_

// C++ standard
#include <cassert>
#include <map>
#include <string>
#include <vector>

// Boost
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>

// OrocosRTT
#include <rtt/DataObjectInterfaces.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Method.hpp>
#include <rtt/Ports.hpp>
#include <rtt/Property.hpp>

// ROS
#include <ros/ros.h>
#include <actionlib/server/action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <pr2_controllers_msgs/QueryTrajectoryState.h>

// Project
#include "joint_trajectory_action_controller_utils.h" // TODO: Move to separate ROS package
#include "JointControllerState.h"

namespace pal
{

namespace control
{

using namespace ::controller; // TODO: Remove!
using namespace ::controller::internal; // TODO: Remove!

// TODO: Could we do away with the hold trajectory?
// TODO: Doc ROS namespace selection (property, otherwise component name)
/// \brief Controller that executes joint-space trajectories on a set of joints.
/// OrocosRTT and pr2_controller_manager-independent port of similarly named controller present in the
/// robot_mechanism_controllers ROS package.
/// \sa{http://www.ros.org/wiki/robot_mechanism_controllers/JointTrajectoryActionController}.
class JointTrajectoryActionController : public RTT::TaskContext
{
public:
  JointTrajectoryActionController(const std::string& name);
  virtual ~JointTrajectoryActionController();

protected:
  virtual bool configureHook();
  virtual bool startHook();
  virtual void updateHook();
  virtual void stopHook();
  virtual void errorHook();

private:
  typedef trajectory_msgs::JointTrajectoryConstPtr JointTrajectoryConstPtr;
  typedef pr2_controllers_msgs::JointTrajectoryControllerState JointTrajectoryControllerState;
  typedef pr2_controllers_msgs::JointTrajectoryControllerStatePtr JointTrajectoryControllerStatePtr;
  typedef pr2_controllers_msgs::JointTrajectoryControllerStateConstPtr JointTrajectoryControllerStateConstPtr;
  typedef pr2_controllers_msgs::QueryTrajectoryState::Request  QueryStateRequest;
  typedef pr2_controllers_msgs::QueryTrajectoryState::Response QueryStateResponse;

  typedef void UpdateCommandMethodType(JointTrajectoryConstPtr);
  typedef bool QueryStateMethodType(QueryStateRequest&, QueryStateResponse&);

  typedef std::vector<std::string> JointNamesType;

  unsigned int getJointDim() const; ///< \pre jointNames_ has been initialized (done in configureHook).

  /// Query list of controller joint names.
  JointNamesType getJointNamesMethod();
  RTT::Method<JointNamesType (void)> getJointNamesMethod_;

  /// \brief Update trajectory command.
  /// Runs on caller thread.
  /// \param msg New trajectory message.
  /// \note This method is not realtime safe, as it allocates resources.
  /// \pre Component must be running.
  void updateCommand(JointTrajectoryConstPtr msg);
  RTT::Method<UpdateCommandMethodType> updateCommandMethod_;

  /// \brief Update trajectory command. Mechanisms for passing the trajectory into realtime.
  /// Generalization of void updateCommand(JointTrajectoryConstPtr msg) to work with ROS action goal handles as well.
  void commandTrajectory(const trajectory_msgs::JointTrajectory::ConstPtr &traj,
                         boost::shared_ptr<RTGoalHandle> gh = boost::shared_ptr<RTGoalHandle>((RTGoalHandle*)NULL),
                         boost::shared_ptr<RTGoalHandleFollow> gh_follow = boost::shared_ptr<RTGoalHandleFollow>((RTGoalHandleFollow*)NULL));

  /// Query where the controller setpoint will be at a desired time.
  /// \pre Component must be running.
  bool queryState(QueryStateRequest& req, QueryStateResponse& resp);
  RTT::Method<QueryStateMethodType> queryStateMethod_;

  // Callbacks related to action servers. They all run on the caller thread.
  void goalCB(GoalHandle gh, ros::NodeHandle nh); ///< Process JointTrajectoryAction goal callback
  RTT::Method<void (GoalHandle, ros::NodeHandle)> goalCBMethod_;
  void cancelCB(GoalHandle gh); ///< Process JointTrajectoryAction cancel callback
  RTT::Method<void (GoalHandle)> cancelCBMethod_;
  void goalCBFollow(GoalHandleFollow gh, ros::NodeHandle nh); ///< Process FollowJointTrajectoryAction goal callback
  RTT::Method<void (GoalHandleFollow, ros::NodeHandle nh)> goalCBFollowMethod_;
  void cancelCBFollow(GoalHandleFollow gh); ///< Process FollowJointTrajectoryAction cancel callback
  RTT::Method<void (GoalHandleFollow)> cancelCBFollowMethod_;

  RTT::WriteBufferPort<JointTrajectoryControllerStateConstPtr> controllerState_; ///< Feedback for client.
  RTT::ReadDataPort<ros::Time> currentTimePort_; ///< Timestamp of the last control cycle.
  RTT::ReadDataPort<JointControllerState> currentStatePort_; ///< Joint state (id, pos, vel, acc) of the last control cycle.
  RTT::WriteDataPort<JointControllerState> referenceStatePort_; ///< Joint state (id, pos, vel, acc) for the next control cycle.
  JointControllerState currentState_;
  JointControllerState referenceState_;
  JointControllerState stateError_;

  JointTrajectoryControllerStatePtr currentControllerState_;

  /// List of joint names odered according to MotorActuator's indices.
  /// \note Value is set at configure-time and remains constant thereafter, hence requires no locks for concurrent reads
  ///  when component is running.
  JointNamesType jointNames_;

  unsigned int loopCount_; ///< Used to publish controller state at a lower frequency

  boost::shared_ptr<RTGoalHandle> rt_active_goal_;
  boost::shared_ptr<RTGoalHandleFollow> rt_active_goal_follow_;
  ros::Timer goal_handle_timer_;

  boost::scoped_ptr<TrajectorySplicer> traj_splicer_;

  void publishControllerState(); ///< Publish controller feedback

  RTT::DataObjectLockFree<SpecifiedTrajectoryPtr> traj_;

  /// \param traj Trajectory, a sequence of spline segments.
  /// \param time Time stamp.
  /// \return Index of segment active at \e time, or -1 if traj is empty or if traj starts after \e time.
  int findActiveTrajectorySegment(const SpecifiedTrajectory& traj, const ros::Time& time);

  /// \brief Create a "hold current position" trajectory.
  /// \note This method is not realtime safe, as it allocates resources.
  /// \pre Component must be configured.
  void setHoldTrajectory();

  void preemptActiveGoal();
};

inline unsigned int JointTrajectoryActionController::getJointDim() const
{
  assert(!jointNames_.empty()); // Precondition
  return jointNames_.size();
}

inline int
JointTrajectoryActionController::findActiveTrajectorySegment(const SpecifiedTrajectory& traj, const ros::Time& time)
{
  int seg = -1;
  // NOTE: Not particularly realtime friendly
  while (seg + 1 < (int)traj.size() && traj[seg+1].start_time < time.toSec()) {++seg;}
  return seg;
}

inline void JointTrajectoryActionController::setHoldTrajectory()
{
  assert(isConfigured()); // Precondition
  ros::Time time = currentTimePort_.Get();
  const unsigned int jointDim = getJointDim();
  SpecifiedTrajectoryPtr holdPtr(new SpecifiedTrajectory(1));
  SpecifiedTrajectory& hold = *holdPtr;
  hold[0].start_time = time.toSec() - 0.001; // NOTE: Magic number
  hold[0].duration = 0.0;
  hold[0].splines.resize(jointDim);
  for (size_t i = 0; i < jointDim; ++i) {hold[0].splines[i].coef[0] = currentState_.position[i];}
  traj_.Set(holdPtr);
}

inline JointTrajectoryActionController::JointNamesType JointTrajectoryActionController::getJointNamesMethod()
{
  assert(isConfigured()); // Precondition
  return jointNames_;
}

inline void JointTrajectoryActionController::updateCommand(JointTrajectoryConstPtr msg)
{
  preemptActiveGoal();
  commandTrajectory(msg);
}

} // control

} // pal

#endif // PAL_CONTROL_JOINT_SPLINE_TRAJECTORY_CONTROLLER_H_