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

#ifndef PAL_CONTROL__JOINT_CONTROLLER_MANAGER_H__
#define PAL_CONTROL__JOINT_CONTROLLER_MANAGER_H__

// C++ standard
#include <algorithm>
#include <string>
#include <vector>

// Boost
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

// OrocosRTT
#include <rtt/TaskContext.hpp>
#include <rtt/Method.hpp>
#include <rtt/Ports.hpp>

// ROS
#include <ros/ros.h>

// Project
#include "JointControllerState.h"
#include "JointInterface.h"

namespace pal
{

namespace control
{

// TODO: Move this class to a separate file?
struct ControllerData
{
  RTT::TaskContext* controller;
  boost::shared_ptr< RTT::WriteDataPort<JointControllerState> > current_state_port;
  boost::shared_ptr< RTT::ReadDataPort<JointControllerState> >  reference_state_port;
  JointControllerState current_state;
  JointControllerState reference_state;
  std::vector<int> lookup;
  std::vector<std::string> joint_names;
};
typedef boost::shared_ptr<ControllerData> ControllerDataPtr;
typedef std::vector<ControllerDataPtr> ControllerDataPtrList;

// NOTE: This is an artifact of using an array of structures (AoS)
std::vector<std::string> getControllerNames(const ControllerDataPtrList& list);

/// \brief Get data associated to controller \e name.
/// \param list List of controller data structures.
/// \param name Name of the controller whose data is desired.
/// \return Shared pointer to controller data structure.
/// Will be valid if a controller named \e name is contained in \e list, or null if the controller name is not found.
ControllerDataPtr getControllerData(const ControllerDataPtrList& list, const std::string& name);

/// \brief Check if the joints of two controllers are different (the intersection of the two joint sets is empty).
bool isDifferentJoints(ControllerDataPtr controller1_data, ControllerDataPtr controller2_data);

inline std::vector<std::string> getControllerNames(const ControllerDataPtrList& list)
{
  std::vector<std::string> name_list;
  BOOST_FOREACH(const ControllerDataPtr controller_data_ptr, list)
  {
    RTT::TaskContext* controller = controller_data_ptr->controller;
    assert(controller);
    name_list.push_back(controller->getName());
  }
  return name_list;
}

inline ControllerDataPtr getControllerData(const ControllerDataPtrList& list, const std::string& name)
{
  BOOST_FOREACH(const ControllerDataPtr controller_data_ptr, list)
  {
    if (name == controller_data_ptr->controller->getName()) {return controller_data_ptr;}
  }
  return ControllerDataPtr();
}

inline bool isDifferentJoints(ControllerDataPtr controller1_data, ControllerDataPtr controller2_data)
{
  BOOST_FOREACH(int joint_id, controller1_data->lookup)
  {
    if (std::find(controller2_data->lookup.begin(),
                  controller2_data->lookup.end(),
                  joint_id) != controller2_data->lookup.end())
    {
      return false;
    }
  }
  return true;
}


/// \brief TODO
// TODO: Attribute credit to pr2_controller_manager implementation.
class JointControllerManager : public RTT::TaskContext
{
public:
  JointControllerManager(const std::string& name);
  virtual ~JointControllerManager();

  enum ControllerSwitchStrictness
  {
    BEST_EFFORT = 1,
    STRICT      = 2
  };

protected:
  virtual bool configureHook();
  virtual bool startHook();
  virtual void updateHook();
  virtual void stopHook();
  virtual void cleanupHook();
  virtual void errorHook();

private:
  RTT::Method<bool (const std::string&)> loadControllerMethod_;
  RTT::Method<bool (const std::string&)> unloadControllerMethod_;
  RTT::Method<bool (const std::vector<std::string>&,
                    const std::vector<std::string>&,
                    ControllerSwitchStrictness)> switchControllerMethod_;
  RTT::Method<std::vector<std::string> ()> listLoadedControllersMethod_;
  RTT::Method<std::vector<std::string> ()> listRunningControllersMethod_;

  typedef long long nsecs;
  typedef nsecs GetSystemTimeType();
  /// Converts RTT::TimeService timestamps to system time. Provided by peer.
  RTT::Method<GetSystemTimeType> getSystemTimeMethod_;

  /// \brief Get current joint state from the joint driver (hardware/simulation) and send it to the running controllers.
  bool updateCurrentState();

  /// \brief Get reference joint state from the running controllers and send it to the joint driver (hardware/simulation).
  bool updateReferenceState();

  /// \param name Name of controller to load.
  /// \return True if successful (controller found and properly configured), false otherwise (including if the
  /// controller was already loaded).
  bool loadController(const std::string& name);

  /// \param name Name of controller to unload.
  /// \return True if successful, false otherwise (including if the controller was already unloaded).
  /// \pre The controller to unload must be \e loaded and \e stopped.
  bool unloadController(const std::string& name);

  /// \brief Start/stop sets of controllers.
  /// Joint resources are not shared, so when a controller is started, it is given \e exclusive access to the joints it
  /// controls (i.e. session start). Joint resources are released upon stopping a controller (i.e. session close).
  /// This means that a controller will fail to start if one of the joints it controls is in use by an already running
  /// controller, or by another controller scheduled to start appearing earlier in start_controllers.
  /// \param start_controllers Names of controllers to start. Leave empty if no controllers are to be started.
  /// \param stop_controllers  Names of controllers to stop. Leave empty if no controllers are to be stopped.
  /// \param strictness  STRICT means that switching will fail if anything goes wrong (an invalid controller name, a
  /// controller that failed to start, etc. ).
  /// BEST_EFFORT means that even when something goes wrong with one controller, the service will still try to
  /// start/stop the remaining controllers.
  /// \return True if successful, false otherwise. Note that calling this method with different \e strictness parameter
  /// values might influence the success of the switch operation.
  /// \note Controllers to start must be \e loaded and \e stopped, while controllers to stop must be \e running,
  /// otherwise the operation will fail.
  /// \sa Convenience methods startController(), stopController()
  bool switchController(const std::vector<std::string>& start_controllers,
                        const std::vector<std::string>& stop_controllers,
                        ControllerSwitchStrictness strictness = STRICT);

  /// \param start_controllers Names of controllers to start.
  /// \return True if successful, false otherwise.
  /// \sa switchController()
  bool startController(const std::vector<std::string>& start_controllers,
                       ControllerSwitchStrictness strictness = STRICT);

  /// \param stop_controllers Names of controllers to stop.
  /// \return True if successful, false otherwise.
  /// \sa switchController()
  bool stopController(const std::vector<std::string>& stop_controllers,
                      ControllerSwitchStrictness strictness = STRICT);

  /// \return List of controllers that meet criteria for starting, i.e. are loaded, not running, and do not share joints
  /// with: a) currently running controllers b) other controllers pending to start.
  /// \note An empty list will be returned if STRICT is specified and at least one controller does not meet the criteria.
  std::vector<std::string> preprocessStartControllers(const std::vector<std::string>& start_controllers,
                                                      ControllerSwitchStrictness strictness) const;

  /// \return List of controllers that meet criteria for stopping, i.e. that are actually running.
  /// \note An empty list will be returned if STRICT is specified and at least one controller does not meet the criteria.
  std::vector<std::string> preprocessStopControllers(const std::vector<std::string>& stop_controllers,
                                                     ControllerSwitchStrictness strictness) const;

  /// \brief Fetch list of controllers to auto-load and do just that.
  bool autoLoadComponents();

  /// \brief Fetch list of controllers to auto-start and do just that.
  bool autoStartComponents();

  /// \brief Convenience method for accessing the loaded controllers from the lock-free data structure.
  const ControllerDataPtrList& getLoadedControllers() const;

  /// \brief Convenience method for accessing the loaded controllers from the lock-free data structure.
  ControllerDataPtrList& getLoadedControllers();

  /// \brief Convenience method for accessing the running controllers from the lock-free data structure.
  const ControllerDataPtrList& getRunningControllers() const;

  /// \brief Convenience method for accessing the running controllers from the lock-free data structure.
  ControllerDataPtrList& getRunningControllers();

  /// \brief Convenience method for accessing the names of the loaded controllers.
  std::vector<std::string> listLoadedControllers();

  /// \brief Convenience method for accessing the names of the running controllers.
  std::vector<std::string> listRunningControllers();

  void updateCurrentTime();

  /// Configure system time service
  bool configureTimeSource();

  /// Query list of managed joint names.
  std::vector<std::string> getJointNamesMethod();

  /// Timestamp of the last control cycle.
  RTT::WriteDataPort<ros::Time>  current_time_port_;

  /// This encapsulation (smart pointer to lock-free data) is required to allow fast and concurrent controller access
  /// (load, unload, start, stop) without compromising the control cycle determinism.
  typedef RTT::DataObjectLockFree<boost::shared_ptr<ControllerDataPtrList> > LockFreeControllerList;

  LockFreeControllerList loaded_controllers_;  ///< List of loaded controllers.
  LockFreeControllerList running_controllers_; ///< List of running controllers.

  /// Prevents concurrent calls to (un)load and switch. Although loaded and running controller lists are contained
  /// in lock-free data structures, a lock is required to define the scope of certain critical sections.
  /// E.g, when switching controllers, the code between getting the current running controllers and re-setting
  /// them should be executed in exclusivity.
  boost::mutex mutex_;

  boost::shared_ptr<JointInterface> joint_iface_; ///< Interface to control command sink (hardware/simulation).

  JointControllerState current_state_;   ///< Current robot state. Includes all joints in robot.
  JointControllerState reference_state_; ///< Current robot state. Includes all joints in robot.
};

inline bool JointControllerManager::startController(const std::vector<std::string>& start_controllers,
                                                    ControllerSwitchStrictness strictness)
{
  return switchController(start_controllers, std::vector<std::string>(), strictness);
}

inline bool JointControllerManager::stopController(const std::vector<std::string>& stop_controllers,
                                                   ControllerSwitchStrictness strictness)
{
  return switchController(std::vector<std::string>(), stop_controllers, strictness);
}

inline const ControllerDataPtrList& JointControllerManager::getLoadedControllers() const
{
  return *(loaded_controllers_.Get());
}

inline ControllerDataPtrList& JointControllerManager::getLoadedControllers()
{
  return *(loaded_controllers_.Get());
}

inline const ControllerDataPtrList& JointControllerManager::getRunningControllers() const
{
  return *(running_controllers_.Get());
}

inline ControllerDataPtrList& JointControllerManager::getRunningControllers()
{
  return *(running_controllers_.Get());
}

inline bool JointControllerManager::updateCurrentState()
{
  // Get complete robot state
  if (!joint_iface_->getCurrentState(current_state_))
  {
    return false;
  }

  // Update joint state of every controller
  BOOST_FOREACH(ControllerDataPtr controller_data, getRunningControllers())
  {
    for (std::size_t i = 0; i < controller_data->lookup.size(); ++i)
    {
      const int id_lookup = controller_data->lookup[i];
      controller_data->current_state.position[i]     = current_state_.position[id_lookup];
      controller_data->current_state.velocity[i]     = current_state_.velocity[id_lookup];
      controller_data->current_state.acceleration[i] = current_state_.acceleration[id_lookup];
    }
    controller_data->current_state_port->Set(controller_data->current_state);
  }
  return true;
}

inline bool JointControllerManager::updateReferenceState()
{
  // Update reference joint state from every controller
  BOOST_FOREACH(ControllerDataPtr controller_data, getRunningControllers())
  {
    controller_data->reference_state_port->Get(controller_data->reference_state);
    for (std::size_t i = 0; i < controller_data->lookup.size(); ++i)
    {
      const int id_lookup = controller_data->lookup[i];
      reference_state_.position[id_lookup]     = controller_data->reference_state.position[i];
      reference_state_.velocity[id_lookup]     = controller_data->reference_state.velocity[i];
      reference_state_.acceleration[id_lookup] = controller_data->reference_state.acceleration[i];
    }
  }
  return joint_iface_->setReferenceState(reference_state_);
}

inline std::vector<std::string> JointControllerManager::listLoadedControllers()
{
  std::vector<std::string> loaded_controller_names;
  BOOST_FOREACH(const ControllerDataPtr controller_data, getLoadedControllers())
  {
    assert(controller_data && controller_data->controller);
    loaded_controller_names.push_back(controller_data->controller->getName());
  }
  return loaded_controller_names;
}

inline std::vector<std::string> JointControllerManager::listRunningControllers()
{
  std::vector<std::string> running_controller_names;
  BOOST_FOREACH(const ControllerDataPtr controller_data, getRunningControllers())
  {
    assert(controller_data && controller_data->controller);
    running_controller_names.push_back(controller_data->controller->getName());
  }
  return running_controller_names;
}

inline void JointControllerManager::updateCurrentTime()
{
  ros::Time time;
  time.fromNSec(getSystemTimeMethod_());
  current_time_port_.Set(time);
}

inline std::vector<std::string> JointControllerManager::getJointNamesMethod()
{
  assert(isConfigured()); // Precondition
  return joint_iface_->getJointNames();
}

} // control

} // pal

#endif // PAL_CONTROL__JOINT_CONTROLLER_MANAGER_H__
