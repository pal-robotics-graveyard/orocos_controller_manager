/*
*  JointControllerManagerRos.h
*  Copyright (c) 2012 PAL Robotics sl. All Rights Reserved.
*  Author: Adolfo Rodriguez Tsouroukdissian
*/

#ifndef PAL_CONTROL_JOINT_CONTROLLER_MANAGER_ROS_H_
#define PAL_CONTROL_JOINT_CONTROLLER_MANAGER_ROS_H_

// C++ standard
#include <string>

// OrocosRTT
#include <rtt/TaskContext.hpp>
#include <rtt/Method.hpp>

// ROS
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <pr2_mechanism_msgs/LoadController.h>
#include <pr2_mechanism_msgs/UnloadController.h>
#include <pr2_mechanism_msgs/SwitchController.h>
#include <pr2_mechanism_msgs/ListControllers.h>

// Project
#include "JointControllerManager.h"

namespace pal
{

namespace control
{

class JointControllerManagerRos : public RTT::TaskContext
{
public:
  JointControllerManagerRos(const std::string& name);
  virtual ~JointControllerManagerRos();

protected:
  virtual bool configureHook();
  virtual bool startHook();
  virtual void updateHook();
  virtual void stopHook();

private:
  typedef bool loadControllerType(const std::string&);
  typedef bool unloadControllerType(const std::string&);
  typedef bool switchControllerType(const std::vector<std::string>&,
                                    const std::vector<std::string>&,
                                    JointControllerManager::ControllerSwitchStrictness);
  typedef std::vector<std::string> listControllersType();

  bool loadController(pr2_mechanism_msgs::LoadController::Request&  req,
                      pr2_mechanism_msgs::LoadController::Response& resp);

  bool unloadController(pr2_mechanism_msgs::UnloadController::Request&  req,
                        pr2_mechanism_msgs::UnloadController::Response& resp);

  bool switchController(pr2_mechanism_msgs::SwitchController::Request&  req,
                        pr2_mechanism_msgs::SwitchController::Response& resp);

  bool listControllers(pr2_mechanism_msgs::ListControllers::Request&  req,
                       pr2_mechanism_msgs::ListControllers::Response& resp);

  ros::NodeHandle nh_;
  ros::CallbackQueue callbackQueue_;

  RTT::Method<loadControllerType>   loadControllerMethod_;
  RTT::Method<unloadControllerType> unloadControllerMethod_;
  RTT::Method<switchControllerType> switchControllerMethod_;
  RTT::Method<listControllersType>  listLoadedControllersMethod_;
  RTT::Method<listControllersType>  listRunningControllersMethod_;

  ros::ServiceServer loadControllerService_;
  ros::ServiceServer unloadControllerService_;
  ros::ServiceServer switchControllerService_;
  ros::ServiceServer listControllersService_;

  bool initOrocosInterface();
  bool initRosInterface();
  void finiRosInterface();
};

inline bool JointControllerManagerRos::startHook()
{
  return initRosInterface();
}

inline void JointControllerManagerRos::updateHook()
{
  callbackQueue_.callAvailable();
}

inline void JointControllerManagerRos::stopHook()
{
  finiRosInterface();
}

inline bool JointControllerManagerRos::loadController(pr2_mechanism_msgs::LoadController::Request&  req,
                                                      pr2_mechanism_msgs::LoadController::Response& resp)
{
  if (isConfigured() && loadControllerMethod_.ready())
  {
    resp.ok = loadControllerMethod_(req.name);
    return true;
  }
  else
  {
    return false;
  }
}

inline bool JointControllerManagerRos::unloadController(pr2_mechanism_msgs::UnloadController::Request&  req,
                                                        pr2_mechanism_msgs::UnloadController::Response& resp)
{
  if (isConfigured() && unloadControllerMethod_.ready())
  {
    resp.ok = unloadControllerMethod_(req.name);
    return true;
  }
  else
  {
    return false;
  }
}

inline bool JointControllerManagerRos::switchController(pr2_mechanism_msgs::SwitchController::Request&  req,
                                                        pr2_mechanism_msgs::SwitchController::Response& resp)
{
  if (isConfigured() && switchControllerMethod_.ready())
  {
    JointControllerManager::ControllerSwitchStrictness strictness = (req.STRICT == req.strictness) ?
    JointControllerManager::STRICT : JointControllerManager::BEST_EFFORT;

    resp.ok = switchControllerMethod_(req.start_controllers,
                                      req.stop_controllers,
                                      strictness);
    return true;
  }
  else
  {
    return false;
  }
}

inline bool JointControllerManagerRos::listControllers(pr2_mechanism_msgs::ListControllers::Request&  req,
                                                       pr2_mechanism_msgs::ListControllers::Response& resp)
{
  if (isConfigured() && listLoadedControllersMethod_.ready() && listRunningControllersMethod_.ready())
  {
    resp.controllers = listLoadedControllersMethod_();
    const std::vector<std::string> running_controllers = listRunningControllersMethod_();

    resp.state.clear();
    BOOST_FOREACH(const std::string& loaded_controller, resp.controllers)
    {
      const std::string state =
      (std::find(running_controllers.begin(), running_controllers.end(), loaded_controller) != running_controllers.end()) ?
      "running" : "stopped";
      resp.state.push_back(state);
    }
    return true;
  }
  else
  {
    return false;
  }
}

inline void JointControllerManagerRos::finiRosInterface()
{
  loadControllerService_.shutdown();
  unloadControllerService_.shutdown();
  switchControllerService_.shutdown();
  listControllersService_.shutdown();
}


} // control

} // pal

#endif // PAL_CONTROL_JOINT_CONTROLLER_MANAGER_ROS_H_
