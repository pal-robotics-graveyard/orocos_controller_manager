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
#include <cassert>
#include <cstddef>
#include <sstream>

// OrocosRTT
#include <rtt/SlaveActivity.hpp>

// Project
#include <common/qa/logger.h>
#include "HandJointInterface.h"
#include "MoCoJointInterface.h"
#include "JointControllerManager.h"
#include "joint_trajectory_action_controller_utils.h"

namespace pal
{

namespace control
{

using namespace ::controller::internal;
using std::size_t;
using std::string;
using std::vector;
using boost::shared_ptr;

JointControllerManager::JointControllerManager(const string& name)
  : RTT::TaskContext(name, PreOperational),
    loadControllerMethod_("load_controller",     &JointControllerManager::loadController,   this),
    unloadControllerMethod_("unload_controller", &JointControllerManager::unloadController, this),
    switchControllerMethod_("switch_controller", &JointControllerManager::switchController, this),
    listLoadedControllersMethod_("list_loaded_controllers", &JointControllerManager::listLoadedControllers, this),
    listRunningControllersMethod_("list_running_controllers", &JointControllerManager::listRunningControllers, this),
    getSystemTimeMethod_(), // Will point to a peer's method
    current_time_port_("currentTime", ros::Time(0.0)),
    loaded_controllers_("loaded_controllers"),
    running_controllers_("running_controllers"),
    joint_iface_()
{
  // Add the method objects to the method interface
  methods()->addMethod(&loadControllerMethod_,         "Load controller",
                                                       "name", "Name of controller to load");
  methods()->addMethod(&unloadControllerMethod_,       "Unload controller",
                                                       "name", "Name of controller to unload");
  methods()->addMethod(&switchControllerMethod_,       "Switch controllers",
                                                       "start_controllers", "list of controller names to start",
                                                       "stop_controllers",  "list of controller names to stop",
                                                       "strictness", "can be set to STRICT or BEST_EFFORT");
  methods()->addMethod(&listLoadedControllersMethod_,  "List names of loaded controllers");
  methods()->addMethod(&listRunningControllersMethod_, "List names of running controllers");

  // Add the port objects to the data flow interface
  ports()->addPort(&current_time_port_);

  // Initialize empty loaded and running controller lists
  shared_ptr<ControllerDataPtrList> empty_loaded_controllers(new ControllerDataPtrList());
  shared_ptr<ControllerDataPtrList> empty_running_controllers(new ControllerDataPtrList());
  loaded_controllers_.Set(empty_loaded_controllers);
  running_controllers_.Set(empty_running_controllers);

  // TODO: Implement this with a plugin mechanism/factory pattern. This is a dirty HACK.
  if (getName().find("hand") != string::npos)
  {
    joint_iface_.reset(new ReemHandsJointInterface(this));
  }
  else
  {
    joint_iface_.reset(new MoCoJointInterface(this));
  }
}

JointControllerManager::~JointControllerManager() {}

bool JointControllerManager::configureHook()
{
  // TODO: Is it possible to change the configure logic so prerequisites are verified first, and resources are allocated later?
  cleanupHook();

  // Configure joint interface
  if (!joint_iface_->configure())
  {
    PAL_ERROR("Failed to configure joint controller manager: Could not configure low-level joint access.");
    return false;
  }

  // Configure time source
  if (!configureTimeSource())
  {
    PAL_ERROR("Failed to configure joint controller manager: Could not find peer providing SystemTimeService Orocos interface.");
    return false;
  }

  // Auto-load controllers
  if (!autoLoadComponents())
  {
    return false;
  }

  // Preallocate resources
  const unsigned int joint_dim = joint_iface_->size();
  current_state_   = JointControllerState(joint_dim);
  reference_state_ = JointControllerState(joint_dim);

  return true;
}

bool JointControllerManager::startHook()
{
  // Check that component is configured
  if (!isConfigured())
  {
    PAL_ERROR("Failed to start joint controller manager: Component has not yet been configured.");
    return false;
  }

  // Start joint interface
  if (!joint_iface_->start())
  {
    PAL_ERROR("Failed to start joint controller manager: Could not start low level joint access.");
    return false;
  }

  // Update current time
  updateCurrentTime();

  // Update current state
  if (!updateCurrentState())
  {
    PAL_ERROR("Failed to start joint controller manager: Could not update current state.");
    return false;
  }

  // Auto-start controllers
  if (!autoStartComponents())
  {
    return false;
  }

  return true;
}

void JointControllerManager::updateHook()
{
  // Get data needed by controllers
  updateCurrentTime();
  if (!updateCurrentState()) {error();}

  // Update running controllers
  BOOST_FOREACH(const ControllerDataPtr controller_data, getRunningControllers())
  {
    controller_data->controller->getActivity()->execute();
  }

  // Send control commands
  if (!updateReferenceState()) {error();}
}

void JointControllerManager::stopHook()
{
  // Stop running controllers
  stopController(getControllerNames(getRunningControllers()), BEST_EFFORT);

  // Stop joint interface
  if (!joint_iface_->stop())
  {
    PAL_ERROR("Failed to successfully stop component: Could not stop low level joint access.");
  }
}

void JointControllerManager::cleanupHook()
{
  // Unload controllers
  BOOST_FOREACH(const string& name, getControllerNames(getLoadedControllers()))
  {
    unloadController(name);
  }
}

void JointControllerManager::errorHook()
{
  // TODO: Reimplement!

  // Recover if able to read current position and send it to actuators
  if (updateCurrentState() && updateReferenceState())
  {
    PAL_INFO("Recovering from runtime error state."); // TODO: Migrate to RT-logger
    recovered();
  }
}

bool JointControllerManager::loadController(const string& name)
{
  // This scope is a critical section
  boost::mutex::scoped_lock lock(mutex_);

  typedef vector<string> getJointNamesType(void);

  // Bail out if the controller is already loaded
  BOOST_FOREACH(const ControllerDataPtr controller_data, getLoadedControllers())
  {
    assert(controller_data && controller_data->controller);
    if (name == controller_data->controller->getName())
    {
      PAL_ERROR_STREAM("Controller \"" << name << "\" has already been loaded.");
      return false;
    }
  }

  // Get peer
  RTT::TaskContext* peer = getPeer(name);
  if (!peer)
  {
    PAL_ERROR_STREAM("Could not load controller \"" << name <<
                     "\" because it does not exist as a peer of the joint controller manager."); // TODO: Move away from peers and towards plugins?
    return false;
  }

  // Validate that peer conforms to controller interface
  using RTT::PortInterface;
  PortInterface* peer_current_state_port = peer->ports()->getPort("currentState");
  const bool peer_current_state_port_ok =
  peer_current_state_port                                              && // Port is valid,
  peer_current_state_port->getPortType() != PortInterface::WritePort   && // can perform reads,
  peer_current_state_port->getConnectionModel() == PortInterface::Data;   // and is unbuffered

  PortInterface* peer_reference_state_port = peer->ports()->getPort("referenceState");
  const bool peer_reference_state_port_ok =
  peer_reference_state_port                                              && // Port is valid,
  peer_reference_state_port->getPortType() != PortInterface::ReadPort    && // can perform writes,
  peer_reference_state_port->getConnectionModel() == PortInterface::Data;   // and is unbuffered

  const string joint_names_method_str = "getJointNames";
  if (!peer_current_state_port_ok   ||
      !peer_reference_state_port_ok ||
      !peer->methods()->hasMethod(joint_names_method_str))
  {
    PAL_ERROR_STREAM("Could not load controller \"" << name <<
                     "\"because it does not conform to the expected controller interface.");
    return false; // Peer does not conform to controller interface
  }
  RTT::Method<getJointNamesType> joint_names_method = peer->methods()->getMethod<getJointNamesType>(joint_names_method_str);
  assert(joint_names_method.ready());

  // Unexpected situation: Running peer
  if (peer->isRunning())
  {
    PAL_ERROR_STREAM("Could not load controller \"" << name << "\" because it is (unexpectedly) running!.");
    return false;
  }

  // (Re)set the peer's activity to a slave activity taking this component's period and priority
  RTT::ActivityInterface* slave = new RTT::SlaveActivity(getActivity());
  peer->setActivity(slave); // Pass ownership of slave

  // (Re)configure peers
  if (peer->isConfigured())
  {
    peer->cleanup();
  }
  if (!peer->configure())
  {
    PAL_ERROR_STREAM("Could not load controller \"" << name << "\"because it failed to configure.");
    return false;
  }

  // Start constructing controller data structure for this peer
  ControllerDataPtr controller_data(new ControllerData());
  controller_data->controller = peer;

  // Get list of joints exposed by peer
  controller_data->joint_names      = joint_names_method();
  const size_t controller_joint_dim = controller_data->joint_names.size();
  assert(!controller_data->joint_names.empty());

  // Check that peer joints exist in this component
  const int last_mapped_id = getLookupVector(controller_data->joint_names,
                                             joint_iface_->getJointNames(),
                                             controller_data->lookup);
  if (last_mapped_id < static_cast<int>(controller_joint_dim))
  {
    PAL_ERROR_STREAM("Could not load controller \"" << name << "\": Could not find joint "<<
                     controller_data->joint_names[last_mapped_id] << " in joint controller manager.");
    return false;
  }

  // Preallocate resources
  controller_data->current_state   = JointControllerState(controller_joint_dim);
  controller_data->reference_state = JointControllerState(controller_joint_dim);

  // Initialize ports
  const string current_state_port_name = name + "-" + peer_current_state_port->getName();
  controller_data->current_state_port.reset(new RTT::WriteDataPort<JointControllerState>(current_state_port_name,
                                                                                         controller_data->current_state));
  const string reference_state_port_name = name + "-" + peer_reference_state_port->getName();
  controller_data->reference_state_port.reset(new RTT::ReadDataPort<JointControllerState>(reference_state_port_name));

  // Add ports to this component's interface
  ports()->addPort(controller_data->current_state_port.get());
  ports()->addPort(controller_data->reference_state_port.get());

  // Connect ports
  if (!controller_data->current_state_port->connectTo(peer_current_state_port) ||
      !controller_data->reference_state_port->connectTo(peer_reference_state_port))
  {
    PAL_ERROR_STREAM("Could not load controller \"" << name << "\": Could not connect ports.");
    return false;
  }

  // Copy currently running controllers to new instance
  shared_ptr<ControllerDataPtrList> new_loaded_controllers_ptr(new ControllerDataPtrList);
  *new_loaded_controllers_ptr = *(loaded_controllers_.Get());
  assert(new_loaded_controllers_ptr);

  // Atomically update list of loaded controllers
  new_loaded_controllers_ptr->push_back(controller_data);
  loaded_controllers_.Set(new_loaded_controllers_ptr);
  PAL_INFO_STREAM("Successfully loaded controller \"" << name << "\".");

  return true;
}

bool JointControllerManager::unloadController(const string& name)
{
  // This scope is a critical section
  boost::mutex::scoped_lock lock(mutex_);

  // Bail out if the controller is running
  BOOST_FOREACH(const ControllerDataPtr controller_data, getRunningControllers())
  {
    assert(controller_data && controller_data->controller);
    if (name == controller_data->controller->getName())
    {
      PAL_ERROR_STREAM("Could not unload controller \"" << name << "\" because it's still running.");
      return false;
    }
  }

  // Copy currently running controllers to new instance
  shared_ptr<ControllerDataPtrList> new_loaded_controllers_ptr(new ControllerDataPtrList);
  *new_loaded_controllers_ptr = *(loaded_controllers_.Get());
  assert(new_loaded_controllers_ptr);

  // Find controller to unload in list of loaded controllers
  for (ControllerDataPtrList::iterator it = new_loaded_controllers_ptr->begin(); it != new_loaded_controllers_ptr->end(); ++it)
  {
    const ControllerData& controller_data = *(*it);
    if (name == controller_data.controller->getName())
    {
      // Remove ports from this component's interface
      ports()->removePort(controller_data.current_state_port->getName());
      ports()->removePort(controller_data.reference_state_port->getName());

      // Cleanup controller
      controller_data.controller->cleanup();

      // Update list of loaded controllers
      new_loaded_controllers_ptr->erase(it);
      loaded_controllers_.Set(new_loaded_controllers_ptr);

      PAL_INFO_STREAM("Successfully unloaded controller \"" << name << "\".");
      return true;
    }
  }
  PAL_ERROR_STREAM("Could not unload controller \"" << name << "\" because it has not been loaded.");
  return false;
}

bool JointControllerManager::switchController(const vector<string>& start_controllers,
                                              const vector<string>& stop_controllers,
                                              ControllerSwitchStrictness strictness)
{
  // This scope is a critical section
  boost::mutex::scoped_lock lock(mutex_);

  // Get currently loaded and running controllers
  const ControllerDataPtrList& loaded_controllers  = getLoadedControllers();
  const ControllerDataPtrList& running_controllers = getRunningControllers();

  // Populate list of controllers to stop
  vector<string> stop_controllers_checked = preprocessStopControllers(stop_controllers, strictness);
  if (STRICT == strictness && !stop_controllers.empty() && stop_controllers_checked.empty())
  {
    return false; // Not all controllers in stop request meet STRICT stopping criteria
  }

  // Populate list of controllers to start
  vector<string> start_controllers_checked = preprocessStartControllers(start_controllers, strictness);
  if (STRICT == strictness && !start_controllers.empty() && start_controllers_checked.empty())
  {
    return false; // Not all controllers in start request meet STRICT starting criteria
  }

  // Copy currently running controllers to new instance
  shared_ptr<ControllerDataPtrList> new_running_controllers_ptr(new ControllerDataPtrList);
  *new_running_controllers_ptr = running_controllers;
  assert(new_running_controllers_ptr);

  // Stop controllers
  BOOST_FOREACH(const string& name, stop_controllers_checked)
  {
    ControllerDataPtrList::iterator it;
    for (it = new_running_controllers_ptr->begin(); it != new_running_controllers_ptr->end(); ++it)
    {
      ControllerDataPtr running_controller_data = *it;
      RTT::TaskContext* controller = running_controller_data->controller;
      assert(controller);
      if (name == controller->getName())
      {
        // Stop controller
        if (controller->getActivity()->stop())
        {
          // Update list of running controllers
          new_running_controllers_ptr->erase(it);
          PAL_INFO_STREAM("Successfully stopped controller \"" << name << "\".");
          break;
        }
        else
        {
          PAL_FATAL_STREAM("Controller \"" << name << "\" failed to stop. This should never happen.");
          // TODO: Take action here, other than log an error?
          return false;
        }
      }
    }
  }

  // Start controllers
  BOOST_FOREACH(const string& name, start_controllers_checked)
  {
    BOOST_FOREACH(ControllerDataPtr loaded_controller_data, loaded_controllers)
    {
      if (name == loaded_controller_data->controller->getName())
      {
        // Update controller current joint state
        for (std::size_t i = 0; i < loaded_controller_data->lookup.size(); ++i)
        {
          const int id_lookup = loaded_controller_data->lookup[i];
          loaded_controller_data->current_state.position[i]     = current_state_.position[id_lookup];
          loaded_controller_data->current_state.velocity[i]     = current_state_.velocity[id_lookup];
          loaded_controller_data->current_state.acceleration[i] = current_state_.acceleration[id_lookup];
        }
        loaded_controller_data->current_state_port->Set(loaded_controller_data->current_state);

        // Start controller
        if (loaded_controller_data->controller->getActivity()->start())
        {
          // Update list of running controllers
          new_running_controllers_ptr->push_back(loaded_controller_data);
          PAL_INFO_STREAM("Successfully started controller \"" << name << "\".");
        }
        else
        {
          PAL_FATAL_STREAM("Controller \"" << name << "\" failed to start. This should never happen.");
          // TODO: Take action here, other than log an error?
          return false;
        }
      }
    }
  }

  // Atomically update list of running controllers
  running_controllers_.Set(new_running_controllers_ptr);

  return true;
}

vector<string> JointControllerManager::preprocessStartControllers(const vector<string>& start_controllers,
                                                                  ControllerSwitchStrictness strictness) const
{
  const ControllerDataPtrList& loaded_controllers  = getLoadedControllers();
  const ControllerDataPtrList& running_controllers = getRunningControllers();

  vector<string> start_controllers_checked;
  BOOST_FOREACH(const string& name, start_controllers)
  {
    // Ignore duplicates
    if (start_controllers_checked.end() != std::find(start_controllers_checked.begin(),
                                                     start_controllers_checked.end(),
                                                     name))
    {
      PAL_DEBUG_STREAM("Skipping duplicate entry \"" << name << "\" in list of controllers to start.");
      continue;
    }

    // Check if controller to start is loaded
    ControllerDataPtr controller_data = getControllerData(loaded_controllers, name);
    if (!controller_data)
    {
      const string msg = "Cannot start controller \"" + name + "\" because it is not loaded.";
      if (STRICT == strictness)
      {
        PAL_ERROR_STREAM(msg);
        return vector<string>();
      }
      else
      {
        PAL_DEBUG_STREAM(msg);
        continue;
      }
    }

    // Check if controller to start is not running
    if (getControllerData(running_controllers, name))
    {
      const string msg = "Cannot start controller \"" + name + "\" because it is already running.";
      if (STRICT == strictness)
      {
        PAL_ERROR_STREAM(msg);
        return vector<string>();
      }
      else
      {
        PAL_DEBUG_STREAM(msg);
        continue;
      }
    }

    // Check if controller to start shares joints with already running controllers
    bool joints_available_running = true;
    BOOST_FOREACH(ControllerDataPtr running_controller_data, running_controllers)
    {
      if (!isDifferentJoints(controller_data, running_controller_data))
      {
        const string msg = "Cannot start controller \"" + name +
        "\" because some of its joints are in use by the already running controller \"" +
        running_controller_data->controller->getName() + "\".";
        if (STRICT == strictness)
        {
          PAL_ERROR_STREAM(msg);
          return vector<string>();
        }
        else
        {
          PAL_DEBUG_STREAM(msg);
          joints_available_running = false;
          break;
        }
      }
    }
    if (!joints_available_running)
    {
      continue;
    }

    // Check if controller to start shares joints with controllers already selected for starting
    bool joints_available_start = true;
    BOOST_FOREACH(const string& to_start_controller_name, start_controllers_checked)
    {
      ControllerDataPtr to_start_controller_data = getControllerData(loaded_controllers, to_start_controller_name);
      if (!isDifferentJoints(controller_data, to_start_controller_data))
      {
        const string msg = "Cannot start controller \"" + name +
        "\" because some of its joints are in use by another controller scheduled for start \"" +
        to_start_controller_name + "\".";
        if (STRICT == strictness)
        {
          PAL_ERROR_STREAM(msg);
          return vector<string>();
        }
        else
        {
          PAL_DEBUG_STREAM(msg);
          joints_available_start = false;
          break;
        }
      }
    }
    if (!joints_available_start)
    {
      continue;
    }

    // Controller meets all requirements for starting
    start_controllers_checked.push_back(name);
    PAL_DEBUG_STREAM("Found controller \"" << name << "\" that needs to be started in list of loaded controllers.");
  }
  return start_controllers_checked;
}

vector<string> JointControllerManager::preprocessStopControllers(const vector<string>& stop_controllers,
                                                                 ControllerSwitchStrictness strictness) const
{
  vector<string> stop_controllers_checked;
  const ControllerDataPtrList& running_controllers = getRunningControllers();
  BOOST_FOREACH(const string& name, stop_controllers)
  {
    // Ignore duplicates
    if (stop_controllers_checked.end() != std::find(stop_controllers_checked.begin(),
                                                    stop_controllers_checked.end(),
                                                    name))
    {
      PAL_DEBUG_STREAM("Skipping duplicate entry \"" << name << "\" in list of controllers to stop.");
      continue;
    }

    // Check if controller to stop is running
    if (!getControllerData(running_controllers, name))
    {
      const string msg = "Cannot stop controller \"" + name + "\" because it is not running.";
      if (STRICT == strictness)
      {
        PAL_ERROR_STREAM(msg);
        return vector<string>();
      }
      else
      {
        PAL_DEBUG_STREAM(msg);
        continue;
      }
    }

    // Controller meets all requirements for starting
    stop_controllers_checked.push_back(name);
    PAL_DEBUG_STREAM("Found controller \"" << name << "\" that needs to be stopped in list of running controllers.");
  }

  return stop_controllers_checked;
}

bool JointControllerManager::autoLoadComponents()
{
  // Populate list of controllers to auto-load
  vector<string> auto_load_controllers;
  RTT::Property<string>* auto_load_controllers_prop = properties()->getProperty<string>("autoLoad");
  if (auto_load_controllers_prop)
  {
    std::stringstream stream;
    stream << auto_load_controllers_prop->get();
    string word;
    while (stream >> word)
    {
      auto_load_controllers.push_back(word);
    }
  }

  // Auto-load controllers (if any)
  BOOST_FOREACH(const string& controller_name, auto_load_controllers)
  {
    if (!loadController(controller_name))
    {
      return false;
    }
  }
  return true;
}

bool JointControllerManager::autoStartComponents()
{
  // Populate list of controllers to auto-start
  vector<string> auto_start_controllers;
  RTT::Property<string>* auto_start_controllers_prop = properties()->getProperty<string>("autoStart");
  if (auto_start_controllers_prop)
  {
    std::stringstream stream;
    stream << auto_start_controllers_prop->get();
    string word;
    while (stream >> word)
    {
      auto_start_controllers.push_back(word);
    }
  }

  // Auto-start controllers (if any)
  return startController(auto_start_controllers, STRICT);
}

bool JointControllerManager::configureTimeSource()
{
  bool found_peer = false;
  const RTT::TaskContext::PeerList peer_list = getPeerList();
  BOOST_FOREACH(const string& peer_name, peer_list)
  {
    RTT::TaskContext* peer = getPeer(peer_name);
    assert(peer);
    const string get_system_time_method_name = "getSystemTime";
    if (peer->methods()->hasMethod(get_system_time_method_name))
    {
      getSystemTimeMethod_ = peer->methods()->getMethod<GetSystemTimeType>("getSystemTime");
      assert(getSystemTimeMethod_.ready());
      found_peer = true;
      break;
    }
  }
  return found_peer;
}

} // control

} // pal
