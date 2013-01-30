/*
*  JointControllerManagerRos.cpp
*  Copyright (c) 2012 PAL Robotics sl. All Rights Reserved.
*  Author: Adolfo Rodriguez Tsouroukdissian
*/

// C++ standard headers
#include <cassert>

// Project headers
#include <common/qa/logger.h>
#include "JointControllerManagerRos.h"

namespace pal
{

namespace control
{

JointControllerManagerRos::JointControllerManagerRos(const std::string& name)
  : RTT::TaskContext(name, PreOperational),
    callbackQueue_(),
    loadControllerMethod_("load_controller"),                  // Will point to a peer's method
    unloadControllerMethod_("unload_controller"),              // Will point to a peer's method
    switchControllerMethod_("switch_controller"),              // Will point to a peer's method
    listLoadedControllersMethod_("list_loaded_controllers"),   // Will point to a peer's method
    listRunningControllersMethod_("list_running_controllers"), // Will point to a peer's method
    loadControllerService_(),
    unloadControllerService_(),
    switchControllerService_(),
    listControllersService_() {}

JointControllerManagerRos::~JointControllerManagerRos() {}

bool JointControllerManagerRos::configureHook()
{
  // Check preconditions
  if( !(engine()->getActivity()->isPeriodic()) )
  {
    PAL_ERROR("Failed to configure joint controller manager ROS interface: Not running in a periodic activity.");
    return false;
  }

  return initOrocosInterface();
}

bool JointControllerManagerRos::initOrocosInterface()
{
  RTT::TaskContext::PeerList peerList = getPeerList();
  if (1 != peerList.size())
  {
    PAL_ERROR("Failed to configure joint controller manager ROS interface. Should have only one peer.");
    return false;
  }

  RTT::TaskContext* peer = getPeer(peerList.front());
  assert(peer);

  loadControllerMethod_         = peer->methods()->getMethod<loadControllerType>(loadControllerMethod_.getName());
  unloadControllerMethod_       = peer->methods()->getMethod<unloadControllerType>(unloadControllerMethod_.getName());
  switchControllerMethod_       = peer->methods()->getMethod<switchControllerType>(switchControllerMethod_.getName());
  listLoadedControllersMethod_  = peer->methods()->getMethod<listControllersType>(listLoadedControllersMethod_.getName());
  listRunningControllersMethod_ = peer->methods()->getMethod<listControllersType>(listRunningControllersMethod_.getName());

  if (!loadControllerMethod_.ready()        ||
      !unloadControllerMethod_.ready()      ||
      !switchControllerMethod_.ready()      ||
      !listLoadedControllersMethod_.ready() ||
      !listRunningControllersMethod_.ready())
  {
    PAL_ERROR("Failed to configure joint controller manager ROS interface. Peer does not conform to the expected interface.");
    return false;
  }

  return true;
}

bool JointControllerManagerRos::initRosInterface()
{
  // Get ROS namespace from peer
  RTT::TaskContext* peer = getPeer(getPeerList().front());
  assert(peer);

  // Initialize ROS node handle
  nh_ = ros::NodeHandle(peer->getName());

  if (!nh_.ok())
  {
    PAL_ERROR("Failed to configure joint controller manager ROS interface: ROS node handle not OK.");
    return false;
  }

  // Setup ROS callback queue
  nh_.setCallbackQueue(&callbackQueue_);

  // Advertise ROS services
  loadControllerService_   = nh_.advertiseService(loadControllerMethod_.getName(),   &JointControllerManagerRos::loadController,   this);
  unloadControllerService_ = nh_.advertiseService(unloadControllerMethod_.getName(), &JointControllerManagerRos::unloadController, this);
  switchControllerService_ = nh_.advertiseService(switchControllerMethod_.getName(), &JointControllerManagerRos::switchController, this);
  listControllersService_  = nh_.advertiseService("list_controllers",                &JointControllerManagerRos::listControllers,  this);

  return true;
}

} // control

} // pal