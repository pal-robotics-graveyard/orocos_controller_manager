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

// C++ standard
#include <cassert>

// Project
#include <common/qa/logger.h>
#include "JointTrajectoryActionControllerRos.h"

using namespace pal::control;

JointTrajectoryActionControllerRos::JointTrajectoryActionControllerRos(const std::string& name)
  : RTT::TaskContext(name, PreOperational),
    callbackQueue_(),
    commandTopicName_("commandTopic", "Name of input ROS topic containing the trajectory command.", "command"),
    controllerStateTopicName_("controllerStateTopic", "Name of output ROS topic containing the controller state.", "state"),
    queryStateServiceName_("queryStateService", "Name of ROS service for querying trajectory state.", "query_state"),
    jointTrajectoryActionName_("jointTrajectoryAction", "Name of ROS jointTrajectoryAction server.", "joint_trajectory_action"),
    followJointTrajectoryActionName_("followJointTrajectoryAction", "Name of ROS FollowJointTrajectoryAction server.", "follow_joint_trajectory"),
    getJointNamesMethod_(),  // Will point to a peer's method
    queryStateMethod_(),     // Will point to a peer's method
    queryStateService_(),
    updateCommandMethod_(),  // Will point to a peer's method
    goalCBMethod_(),         // Will point to a peer's method
    cancelCBMethod_(),       // Will point to a peer's method
    goalCBFollowMethod_(),   // Will point to a peer's method
    cancelCBFollowMethod_(), // Will point to a peer's method
    controllerState_("controllerState"),
    currentControllerState_()
{
  // Add the property/attribute objects to the property/attributes interface
  properties()->addProperty(&commandTopicName_);
  properties()->addProperty(&controllerStateTopicName_);
  properties()->addProperty(&queryStateServiceName_);
  properties()->addProperty(&jointTrajectoryActionName_);
  properties()->addProperty(&followJointTrajectoryActionName_);

  // Add the port objects to the data flow interface
  ports()->addPort(&controllerState_);
}

JointTrajectoryActionControllerRos::~JointTrajectoryActionControllerRos() {}

bool JointTrajectoryActionControllerRos::configureHook()
{
  // Check preconditions
  if( !(engine()->getActivity()->isPeriodic()) )
  {
    PAL_ERROR("Failed to configure component: Not running in a periodic activity.");
    return false;
  }

  return initOrocosInterface();
}

bool JointTrajectoryActionControllerRos::startHook()
{
  return initRosInterface();
}

void JointTrajectoryActionControllerRos::updateHook()
{
  // Process command requests, state queries
  callbackQueue_.callAvailable();

  // Publish controller state
  if (!controllerState_.empty())
  {
    controllerState_.Pop(currentControllerState_);
    controllerStatePublisher_.publish(currentControllerState_);
  }
}

void JointTrajectoryActionControllerRos::stopHook()
{
  queryStateService_.shutdown();
  commandSubscriber_.shutdown();
  controllerStatePublisher_.shutdown();
  actionServer_.reset();
  actionServerFollow_.reset();
}

bool JointTrajectoryActionControllerRos::initOrocosInterface()
{
  RTT::TaskContext::PeerList peerList = getPeerList();
  if (1 != peerList.size())
  {
    PAL_ERROR("Failed to configure component. Should have only one peer.");
    return false;
  }

  RTT::TaskContext* peer = getPeer(peerList.front());
  assert(peer);

  getJointNamesMethod_  = peer->methods()->getMethod<GetJointNamesMethodType>("getJointNames");
  queryStateMethod_     = peer->methods()->getMethod<QueryStateMethodType>("queryState");
  updateCommandMethod_  = peer->methods()->getMethod<UpdateCommandMethodType>("updateCommand");
  goalCBMethod_         = peer->methods()->getMethod<JTASGoalType>("goal");
  cancelCBMethod_       = peer->methods()->getMethod<JTASCancelType>("cancel");
  goalCBFollowMethod_   = peer->methods()->getMethod<FJTASGoalType>("goalFollow");
  cancelCBFollowMethod_ = peer->methods()->getMethod<FJTASCancelType>("cancelFollow");

  if (!getJointNamesMethod_.ready() ||
      !queryStateMethod_.ready()    ||
      !updateCommandMethod_.ready() ||
      !goalCBMethod_.ready()        ||
      !cancelCBMethod_.ready()      ||
      !goalCBFollowMethod_.ready()  ||
      !cancelCBFollowMethod_.ready())
  {
    PAL_ERROR("Failed to configure component. Peer does not conform to the expected interface.");
    return false;
  }

  return true;
}

bool JointTrajectoryActionControllerRos::initRosInterface()
{
  // Get ROS namespace from peer
  RTT::TaskContext* peer = getPeer(getPeerList().front());
  assert(peer);

  // Initialize ROS node handle
  nh_ = ros::NodeHandle(peer->getName());

  if (!nh_.ok())
  {
    PAL_ERROR("Failed to configure component: ROS node handle not OK.");
    return false;
  }

  // Setup ROS callback queue
  nh_.setCallbackQueue(&callbackQueue_);

  // Advertise ROS services
  queryStateService_ = nh_.advertiseService(queryStateServiceName_.get(), &JointTrajectoryActionControllerRos::queryState, this);

  // ROS topic subscribers
  commandSubscriber_ = nh_.subscribe(commandTopicName_.get(), 1, &JointTrajectoryActionControllerRos::commandCallback, this);

  // ROS topic publishers
  controllerStatePublisher_ = nh_.advertise<JointTrajectoryControllerState>(controllerStateTopicName_.get(), 1);

  // Action servers
  actionServer_.reset(new JTAS(nh_, jointTrajectoryActionName_.get(),
                               boost::bind(&JointTrajectoryActionControllerRos::goalCB, this, _1),
                               boost::bind(&JointTrajectoryActionControllerRos::cancelCB, this, _1),
                               false));
  actionServerFollow_.reset(new FJTAS(nh_, followJointTrajectoryActionName_.get(),
                                      boost::bind(&JointTrajectoryActionControllerRos::goalCBFollow, this, _1),
                                      boost::bind(&JointTrajectoryActionControllerRos::cancelCBFollow, this, _1),
                                      false));
  actionServer_->start();
  actionServerFollow_->start();

  return true;
}