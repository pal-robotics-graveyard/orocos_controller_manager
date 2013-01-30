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

#ifndef PAL_CONTROL_SYSTEM_TIME_SERVICE_H_
#define PAL_CONTROL_SYSTEM_TIME_SERVICE_H_

// C++ standard
#include <string>

// OrocosRTT
#include <rtt/DataObjectInterfaces.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Method.hpp>
#include <rtt/TimeService.hpp>

namespace pal
{

namespace control
{

// TODO: Test Xenomai 2.6.0 or later and get rid of this HACK, as they add:
// "CLOCK_HOST_REALTIME, a clock synchronized with Linux CLOCK_REALTIME clock, but safe to be read when in primary mode"

/// \brief Component that keeps track of the offset between the RTT::TimeService and the system clock.
/// This offset is the result of a fixed difference between the origins of the two time sources, and the possible drift
/// that may occur over time, eg. as a consequence of NTP synchronisation.
/// The time source used by the RTT::TimeService depends on the orocos target this component is compiled against.
/// The system time source is the posix clock_gettime function using the CLOCK_REALTIME clock.
///
/// Every time this component is triggered, it refreshes an estimate of the offset between the RTT::TimeService and the
/// system clock.
///
/// Use the toSystemTime(rtTime) method to convert a timestamp (in nanoseconds) of the realtime clock to its equivalent
/// in the system clock reference (in nanoseconds).

class SystemTimeService : public RTT::TaskContext
{
public:
  typedef RTT::TimeService::nsecs nsecs;
  typedef nsecs ToSystemTimeType(nsecs);
  typedef nsecs GetSystemTimeType();

  SystemTimeService(const std::string& name);
  virtual ~SystemTimeService();

protected:
  virtual bool configureHook();
  virtual bool startHook();
  virtual void updateHook() {updateOffset();}

private:
  const static unsigned int MAX_ITER = 1000;
  RTT::TimeService* timeService_;
  nsecs clockGettimeDuration_;
  RTT::DataObjectLockFree<nsecs> offset_;

  /// Convert a timestamp obtained with a realtime clock to the system clock reference.
  /// \param rtTime Realtime clock timestamp, in nanoseconds.
  /// \return System time equivalent of rtTime, in nanoseconds.
  nsecs toSystemTime(nsecs rtTime);
  RTT::Method<ToSystemTimeType> toSystemTimeMethod_;

  /// Get current timestamp with a realtime clock, and convert it to the system clock reference.
  /// \return Current system time, in nanoseconds.
  nsecs getSystemTime();
  RTT::Method<GetSystemTimeType> getSystemTimeMethod_;

  /// Update offset between the RTT::TimeService and the system clock.
  void updateOffset();
};

inline SystemTimeService::nsecs SystemTimeService::toSystemTime(nsecs rtTime)
{
  return rtTime + offset_.Get();
}

inline SystemTimeService::nsecs SystemTimeService::getSystemTime()
{
  return toSystemTime(timeService_->getNSecs());
}

inline void SystemTimeService::updateOffset()
{
  timespec systemTime;
  nsecs rtTime;
  clock_gettime(CLOCK_REALTIME, &systemTime);
  rtTime = timeService_->getNSecs();
  const nsecs systemTimeNs = systemTime.tv_sec * 1000000000ull + systemTime.tv_nsec;

  // clockGettimeDuration_ below compensates the fact that clock_gettime and timeService_->getNSecs() are
  // being called sequentially, and not in parallel. Testing in one platform revelas that this value is in the order
  // of ~750 +-25ns.
  offset_.Set(systemTimeNs + clockGettimeDuration_ - rtTime);
}

} // control

} // pal

#endif // PAL_CONTROL_SYSTEM_TIME_SERVICE_H_