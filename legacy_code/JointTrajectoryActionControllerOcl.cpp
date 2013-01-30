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


// OrocosOCL headers
#include <ocl/ComponentLoader.hpp>

// Project headers
#include "JointTrajectoryActionController.h"
#include "JointTrajectoryActionControllerRos.h"
#include "SystemTimeService.h"
#include "JointControllerManager.h"
#include "JointControllerManagerRos.h"
#include "DummyHandController.h"

// Create shared library
ORO_CREATE_COMPONENT_TYPE();

// Register multiple components
ORO_LIST_COMPONENT_TYPE(pal::control::JointTrajectoryActionController);
ORO_LIST_COMPONENT_TYPE(pal::control::JointTrajectoryActionControllerRos);
ORO_LIST_COMPONENT_TYPE(pal::control::SystemTimeService);
ORO_LIST_COMPONENT_TYPE(pal::control::JointControllerManager);
ORO_LIST_COMPONENT_TYPE(pal::control::JointControllerManagerRos);
ORO_LIST_COMPONENT_TYPE(pal::dev::DummyHandController);