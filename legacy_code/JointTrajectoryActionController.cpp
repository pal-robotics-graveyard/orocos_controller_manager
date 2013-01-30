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

// C++ standard
#include <algorithm>

// Boost
#include <boost/foreach.hpp>

// OrocosRTT
#include <rtt/Property.hpp>

// OrocosOCL
#include <ocl/ComponentLoader.hpp>

// ROS
#include <urdf/model.h>

// Project
#include <common/qa/logger.h>
#include "JointTrajectoryActionController.h"

using namespace pal::control;
using std::size_t;

JointTrajectoryActionController::JointTrajectoryActionController(const std::string& name)
  : RTT::TaskContext(name, PreOperational),
    getJointNamesMethod_("getJointNames", &JointTrajectoryActionController::getJointNamesMethod, this),
    updateCommandMethod_("updateCommand", &JointTrajectoryActionController::updateCommand, this),
    queryStateMethod_("queryState", &JointTrajectoryActionController::queryState, this),
    goalCBMethod_("goal", &JointTrajectoryActionController::goalCB, this),
    cancelCBMethod_("cancel", &JointTrajectoryActionController::cancelCB, this),
    goalCBFollowMethod_("goalFollow", &JointTrajectoryActionController::goalCBFollow, this),
    cancelCBFollowMethod_("cancelFollow", &JointTrajectoryActionController::cancelCBFollow, this),
    controllerState_("controllerState", 1),
    currentTimePort_("currentTime"),
    currentStatePort_("currentState"),
    referenceStatePort_("referenceState"),
    currentControllerState_(new JointTrajectoryControllerState()),
    loopCount_(0),
    traj_("")
{
  // Add the method objects to the method interface
  methods()->addMethod(&getJointNamesMethod_);  // Do not expose to scripting
  methods()->addMethod(&updateCommandMethod_);  // Do not expose to scripting
  methods()->addMethod(&queryStateMethod_);     // Do not expose to scripting
  methods()->addMethod(&goalCBMethod_);         // Do not expose to scripting
  methods()->addMethod(&cancelCBMethod_);       // Do not expose to scripting
  methods()->addMethod(&goalCBFollowMethod_);   // Do not expose to scripting
  methods()->addMethod(&cancelCBFollowMethod_); // Do not expose to scripting

  // Add the port objects to the data flow interface
  ports()->addPort(&controllerState_);
  ports()->addPort(&currentTimePort_);
  ports()->addPort(&currentStatePort_);
  ports()->addPort(&referenceStatePort_);
}

JointTrajectoryActionController::~JointTrajectoryActionController() {}

bool JointTrajectoryActionController::configureHook()
{
  // ROS node handle for the controller namespace
  ros::NodeHandle nh(getName());

  // Initialize joint names vector
  if (!::controller::internal::getJointNames(nh, jointNames_))
  {
    PAL_ERROR("Failed to configure component: Could not get list of controlled joints.");
    return false;
  }
  const unsigned int jointDim = getJointDim();

  // Default trajectory and goal tolerances
  std::vector<JointTolerance> default_trajectory_tolerance(jointDim);
  std::vector<JointTolerance> default_goal_tolerance(jointDim);
  double default_goal_time_tolerance;

  getDefaultTolerances(nh,
                       jointNames_,
                       default_trajectory_tolerance,
                       default_goal_tolerance,
                       default_goal_time_tolerance);

  // Trajectory splicer
  traj_splicer_.reset(new TrajectorySplicer(jointNames_,
                                            default_trajectory_tolerance,
                                            default_goal_tolerance,
                                            default_goal_time_tolerance));

  // Preallocate resources
  currentState_   = JointControllerState(jointDim);
  referenceState_ = JointControllerState(jointDim);
  stateError_     = JointControllerState(jointDim);

  currentControllerState_->joint_names = jointNames_;
  currentControllerState_->desired.positions.resize(jointDim);
  currentControllerState_->desired.velocities.resize(jointDim);
  currentControllerState_->desired.accelerations.resize(jointDim);
  currentControllerState_->actual.positions.resize(jointDim);
  currentControllerState_->actual.velocities.resize(jointDim);
  currentControllerState_->error.positions.resize(jointDim);
  currentControllerState_->error.velocities.resize(jointDim);
  controllerState_.Set(currentControllerState_);

  return true;
}

bool JointTrajectoryActionController::startHook()
{
  // Check that component is configured
  if (!isConfigured())
  {
    PAL_ERROR("Failed to start component: Component has not yet been configured.");
    return false;
  }

  // Start peer providing ROS interface
  // TODO: Make less fragile!
  RTT::TaskContext* ros_interface = getPeer(getName() + "_ros");
  if (!ros_interface)
  {
    PAL_ERROR_STREAM("Failed to configure component: Could not find peer providing ROS interface for \"" << getName() << "\".");
    return false;
  }
  ros_interface->start();

  // Check port connections
  BOOST_FOREACH(RTT::PortInterface* port, ports()->getPorts())
  {
    if (!port->connected())
    {
      PAL_ERROR_STREAM("Failed to start component: Port \"" << port->getName() << "\" not connected.");
      return false;
    }
  }

  // Check that last control timestep is nonzero
  if (currentTimePort_.Get().isZero())
  {
    PAL_ERROR("Failed to start component: Not receiving clock updates from joint controller backend.");
    return false;
  }

  // State update and resource preallocation
  currentStatePort_.Get(currentState_);
  referenceState_ = currentState_;
  referenceStatePort_.Set(referenceState_); // TODO: Initialize without pushing to readers?: fix when migration to RTT 2.x takes place

  // Hold current position
  setHoldTrajectory();

  return true;
}

void JointTrajectoryActionController::updateHook()
{
  // State update
  currentStatePort_.Get(currentState_);
  ros::Time time = currentTimePort_.Get();

  // Fetch specified trajectory
  SpecifiedTrajectoryPtr trajPtr = traj_.Get();
  assert(trajPtr);
  const SpecifiedTrajectory& traj = *trajPtr;
  if (traj.empty())
  {
    PAL_ERROR("No segments in the trajectory"); // TODO: Migrate to RT-logger
    return;
  }

  // Trajectory sampling
  const int seg = sampleTrajectory(traj,
                                   time.toSec(),
                                   referenceState_.position,
                                   referenceState_.velocity,
                                   referenceState_.acceleration);
  if (seg == -1)
  {
    PAL_ERROR("No earlier segments. First segment starts at %.3lf (now = %.3lf)", traj[0].start_time, time.toSec()); // TODO: Migrate to RT-logger
    return;
  }

  // Trajectory following
  referenceStatePort_.Set(referenceState_);

  // Determine if the goal has failed or succeeded
  const unsigned int jointDim = getJointDim();
  for (size_t j = 0; j < jointDim; ++j)
  {
    stateError_.position[j]     = currentState_.position[j]     - referenceState_.position[j];
    stateError_.velocity[j]     = currentState_.velocity[j]     - referenceState_.velocity[j];
    stateError_.acceleration[j] = currentState_.acceleration[j] - referenceState_.acceleration[j];
  }
  ::controller::internal::checkTolerances(rt_active_goal_,
                                          rt_active_goal_follow_,
                                          stateError_.position,
                                          stateError_.velocity,
                                          traj,
                                          time,
                                          seg);

  // Controller state
  publishControllerState();
}

void JointTrajectoryActionController::stopHook()
{
  // Stop peer providing ROS interface
  // TODO: Make less fragile!
  RTT::TaskContext* ros_interface = getPeer(getName() + "_ros");
  if (ros_interface)
  {
    ros_interface->stop();
  }
  else
  {
    PAL_ERROR_STREAM("Unexpected error: Could not find and stop peer providing ROS interface for \"" << getName() << "\".");
  }
}

void JointTrajectoryActionController::errorHook()
{
  // TODO
}

bool JointTrajectoryActionController::queryState(QueryStateRequest& req, QueryStateResponse& resp)
{
  if (!isRunning()) {return false;}

  // Fetch specified trajectory
  SpecifiedTrajectoryPtr trajPtr = traj_.Get(); // TODO: Encapsulate?
  assert(trajPtr);
  const SpecifiedTrajectory& traj = *trajPtr;

  // Resize response vectors to appropriate size
  const unsigned int jointDim = getJointDim();
  resp.name = jointNames_;
  resp.position.resize(jointDim);
  resp.velocity.resize(jointDim);
  resp.acceleration.resize(jointDim);

  // Trajectory sampling
  const int seg = sampleTrajectory(traj, req.time.toSec(), resp.position, resp.velocity, resp.acceleration);
  return seg != -1;
}

void JointTrajectoryActionController::publishControllerState()
{
  if (loopCount_ % 10 == 0) // NOTE: Magic number
  {
    const unsigned int jointDim = getJointDim();
    currentControllerState_->header.stamp          = currentTimePort_.Get();
    currentControllerState_->desired.positions     = referenceState_.position;
    currentControllerState_->desired.velocities    = referenceState_.velocity;
    currentControllerState_->desired.accelerations = referenceState_.acceleration;
    currentControllerState_->actual.positions      = currentState_.position;
    currentControllerState_->actual.velocities     = currentState_.velocity;
    currentControllerState_->error.positions       = stateError_.position;
    currentControllerState_->error.velocities      = stateError_.velocity;
    controllerState_.Push(currentControllerState_);
    loopCount_ = 0;
  }

  ++loopCount_;
}

void JointTrajectoryActionController::commandTrajectory(const trajectory_msgs::JointTrajectory::ConstPtr &msg,
                                                        boost::shared_ptr<RTGoalHandle> gh,
                                                        boost::shared_ptr<RTGoalHandleFollow> gh_follow)
{
  // NOTE: We use some underscore_notation here as we're keeping code from the original implementation
  // Also, the original implementation checks for wrapping joints, which we don't have now, so that part has been
  // omitted.

  // Precondition: At most one goal handle can be valid
  assert(!gh || !gh_follow);

  if (!isRunning()) // Precondition
  {
    PAL_ERROR_THROTTLE(2.0, "Cannot process trajectory commands. Controller not yet running.");
    return;
  }

  ros::Time time = currentTimePort_.Get() + ros::Duration(0.01);
  PAL_DEBUG("Figuring out new trajectory at %.3lf, with data from %.3lf", time.toSec(), msg->header.stamp.toSec());

  // Hold current position if trajectory is empty
  if (msg->points.empty())
  {
    PAL_INFO("Empty trajectory command, stopping.");
    setHoldTrajectory();
    return;
  }

  // Currently followed trajectory
  SpecifiedTrajectoryPtr prev_traj_ptr = traj_.Get();
  assert(prev_traj_ptr);

  // Trajectoty splicing: Keep previous trajectory until the message header time, and from then on replace it with the
  // new one specified in the message
  SpecifiedTrajectoryPtr new_traj_ptr = traj_splicer_->splice(*prev_traj_ptr,
                                                              time,
                                                              msg,
                                                              gh,
                                                              gh_follow);

  if (!new_traj_ptr) {return;}

  // Commits the new trajectory
  traj_.Set(new_traj_ptr);
}

void JointTrajectoryActionController::goalCB(GoalHandle gh, ros::NodeHandle nh)
{
  // Ensures that the joints in the goal match the joints we are commanding.
  if (!setsEqual(jointNames_, gh.getGoal()->trajectory.joint_names))
  {
    PAL_ERROR("Joints on incoming goal don't match our joints");
    gh.setRejected();
    return;
  }

  preemptActiveGoal();

  gh.setAccepted();
  boost::shared_ptr<RTGoalHandle> rt_gh(new RTGoalHandle(gh));

  // Sends the trajectory along to the controller
  goal_handle_timer_ = nh.createTimer(ros::Duration(0.05), &RTGoalHandle::runNonRT, rt_gh); // NOTE: Magic value
  commandTrajectory(share_member(gh.getGoal(), gh.getGoal()->trajectory), rt_gh);
  rt_active_goal_ = rt_gh;
  goal_handle_timer_.start();
}

void JointTrajectoryActionController::cancelCB(GoalHandle gh)
{
  boost::shared_ptr<RTGoalHandle> current_active_goal(rt_active_goal_);
  if (current_active_goal && current_active_goal->gh_ == gh)
  {
    rt_active_goal_.reset();

    trajectory_msgs::JointTrajectory::Ptr empty(new trajectory_msgs::JointTrajectory);
    empty->joint_names.resize(jointNames_.size());
    for (size_t j = 0; j < jointNames_.size(); ++j) {empty->joint_names[j] = jointNames_[j];}
    commandTrajectory(empty);

    // Marks the current goal as canceled.
    current_active_goal->gh_.setCanceled();
  }
}

void JointTrajectoryActionController::goalCBFollow(GoalHandleFollow gh, ros::NodeHandle nh)
{
  // Ensures that the joints in the goal match the joints we are commanding.
  if (!setsEqual(jointNames_, gh.getGoal()->trajectory.joint_names))
  {
    ROS_ERROR("Joints on incoming goal don't match our joints");
    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
    gh.setRejected(result);
    return;
  }
  preemptActiveGoal();

  gh.setAccepted();
  boost::shared_ptr<RTGoalHandleFollow> rt_gh(new RTGoalHandleFollow(gh));

  // Sends the trajectory along to the controller
  goal_handle_timer_ = nh.createTimer(ros::Duration(0.05), &RTGoalHandleFollow::runNonRT, rt_gh); // NOTE: Magic value
  commandTrajectory(share_member(gh.getGoal(), gh.getGoal()->trajectory),
                    boost::shared_ptr<RTGoalHandle>((RTGoalHandle*)NULL),
                    rt_gh);
  rt_active_goal_follow_ = rt_gh;
  goal_handle_timer_.start();
}

void JointTrajectoryActionController::cancelCBFollow(GoalHandleFollow gh)
{
  boost::shared_ptr<RTGoalHandleFollow> current_active_goal(rt_active_goal_follow_);
  if (current_active_goal && current_active_goal->gh_ == gh)
  {
    rt_active_goal_follow_.reset();

    trajectory_msgs::JointTrajectory::Ptr empty(new trajectory_msgs::JointTrajectory);
    empty->joint_names.resize(jointNames_.size());
    for (size_t j = 0; j < jointNames_.size(); ++j)
      empty->joint_names[j] = jointNames_[j];
    commandTrajectory(empty);

    // Marks the current goal as canceled.
    current_active_goal->gh_.setCanceled();
  }
}

void JointTrajectoryActionController::preemptActiveGoal()
{
  boost::shared_ptr<RTGoalHandle> current_active_goal(rt_active_goal_);
  boost::shared_ptr<RTGoalHandleFollow> current_active_goal_follow(rt_active_goal_follow_);

  // Cancels the currently active goal.
  if (current_active_goal)
  {
    // Marks the current goal as canceled.
    rt_active_goal_.reset();
    current_active_goal->gh_.setCanceled();
  }
  if (current_active_goal_follow)
  {
    rt_active_goal_follow_.reset();
    current_active_goal_follow->gh_.setCanceled();
  }
}
