/*
 * Software License Agreement (Modified BSD License)
 *
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

/** \author Adolfo Rodriguez Tsouroukdissian. */

#ifndef PAL_CONTROL_JOINT_CONTROLLER_STATE_H__
#define PAL_CONTROL_JOINT_CONTROLLER_STATE_H__

// C++ standard
#include <cstddef>
#include <vector>

namespace pal
{

namespace control
{

/**
 * \brief Structure used by a joint controller to report current state, and by its clients to set the desired state.
 * Velocity and acceleration are optional entries, and should be empty if unused. The id and position entries must
 * always be populated. All non-empty vectors must have the same size.
 */
struct JointControllerState
{
  /**
   * Resource-allocating constructor.
   */
  JointControllerState(std::vector<double>::size_type size = 0)
    : position(std::vector<double>(size, 0.0)),
      velocity(std::vector<double>(size, 0.0)),
      acceleration(std::vector<double>(size, 0.0))
  {}

  std::vector<double> position;
  std::vector<double> velocity;
  std::vector<double> acceleration;
};

} // control

} // pal

#endif // PAL_CONTROL_JOINT_CONTROLLER_STATE_H__
 
