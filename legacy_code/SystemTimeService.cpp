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

// POSIX
#include <time.h>

// Project
#include <common/qa/logger.h>
#include "SystemTimeService.h"

using namespace pal::control;

SystemTimeService::SystemTimeService(const std::string& name)
  : RTT::TaskContext(name, PreOperational),
    timeService_(RTT::TimeService::Instance()),
    clockGettimeDuration_(0),
    offset_("", 0),
    toSystemTimeMethod_("toSystemTime", &SystemTimeService::toSystemTime, this),
    getSystemTimeMethod_("getSystemTime", &SystemTimeService::getSystemTime, this)
{
  // Add the method objects to the method interface
  methods()->addMethod(&toSystemTimeMethod_, "Convert a timestamp obtained with a realtime clock to the system clock reference.",
                                             "rtTime", "Realtime clock timestamp, in nanoseconds.");
  methods()->addMethod(&getSystemTimeMethod_, "Get current timestamp with a realtime clock, and convert it to the system clock reference.");
}

SystemTimeService::~SystemTimeService() {}

bool SystemTimeService::configureHook()
{
  // Measure duration of a call to clock_gettime(CLOCK_REALTIME, &ts)
  timespec ts;
  nsecs startTime = timeService_->getNSecs();
  for(unsigned int i = 0; i < MAX_ITER; ++i)
  {
    clock_gettime(CLOCK_REALTIME, &ts);
  }

  clockGettimeDuration_ = timeService_->getNSecs(startTime) / MAX_ITER;
  PAL_DEBUG_STREAM("clock_gettime(CLOCK_REALTIME, &ts) duration: " << clockGettimeDuration_ << " ns.");

  // Compute initial offset value (will be periodically refreshed in updateHook)
  updateOffset();
  return true;
}

bool SystemTimeService::startHook()
{
  const bool configured = isConfigured();
  if (!configured) {PAL_ERROR("Failed to start component. It has not been configured.");}
  return configured;
}
