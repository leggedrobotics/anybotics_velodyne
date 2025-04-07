// Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of {copyright_holder} nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/** \file
 *
 *  ROS driver nodelet for the Velodyne 3D LIDARs
 */

#include <string>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "any_velodyne_driver/driver.h"
#include "any_velodyne_msgs/VelodyneInterfaceStatus.h"

namespace velodyne_driver
{
class DriverNodelet : public nodelet::Nodelet
{
public:
  DriverNodelet() : running_(false)
  {
  }

  ~DriverNodelet()
  {
    // Support that nodelets are shut down smoothly. Explicit tear down of ROS infrastructure
    // ensures that nodelet threads leave ROS-time-dependent sleeps.
    // Request shutdown of the ROS node.
    ros::requestShutdown();
    // Shut down ROS time.
    ros::Time::shutdown();

    if (running_)
    {
      NODELET_INFO("shutting down driver thread");
      running_ = false;
      deviceThread_->join();
      NODELET_INFO("driver thread stopped");
    }
  }

private:
  virtual void onInit(void);
  virtual void devicePoll(void);

  volatile bool running_;  ///< device thread is running
  boost::shared_ptr<boost::thread> deviceThread_;
  boost::shared_ptr<VelodyneDriver> dvr_;  ///< driver implementation class

  void pubInterfaceStateTimerCb(const ros::WallTimerEvent& /*event*/);  // Publish the current state of the interface
  void convertToRosMsg(uint8_t& interface_status,
                       std::string& interface_status_msg);  // Converts State into
                                                            // any_velodyne_msgs::VelodyneInterfaceStatus
                                                            // compatible format
  void getDiagnosticsInterfaceState(diagnostic_updater::DiagnosticStatusWrapper& stat);  // ROS diagnostics updater task
                                                                                         // to get the sensor interface
                                                                                         // status
  any_velodyne_msgs::VelodyneInterfaceStatus getInterfaceStateROSMsg();  // Populates a ROS msg ready to publish to the
                                                                         // lidar/interface_status topic

  // Sensor interface state
  enum State
  {
    NONE,
    ERROR,
    STOPPED,
    DISCONNECTED,
    CONNECTED,
    STREAMING,
    DEGRADED
  };

  // Interface diagnostics
  std::atomic<State> interface_state_{ State::NONE };
  ros::WallTimer interface_callback_timer_;
  ros::Publisher interface_status_pub_;
  diagnostic_updater::Updater interface_diagnostics_updater_;
};

void DriverNodelet::onInit()
{
  // start the driver
  dvr_.reset(new VelodyneDriver(getNodeHandle(), getPrivateNodeHandle(), getName()));

  // set up ros diagnostics updater
  interface_diagnostics_updater_.setHardwareID("Velodyne Lidar VLP-16");
  interface_diagnostics_updater_.add("Interface status checker", this, &DriverNodelet::getDiagnosticsInterfaceState);
  interface_diagnostics_updater_.broadcast(0, "Starting diagnostics");

  // set up the interface status reporting
  interface_callback_timer_ = getNodeHandle().createWallTimer(
      ros::WallDuration(1.0), &DriverNodelet::pubInterfaceStateTimerCb, this, false, true);

  interface_status_pub_ =
      getNodeHandle().advertise<any_velodyne_msgs::VelodyneInterfaceStatus>("lidar/interface_status", 1);

  // spawn device poll thread
  running_ = true;
  deviceThread_ = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&DriverNodelet::devicePoll, this)));
}

/** @brief Device poll thread main loop. */
void DriverNodelet::devicePoll()
{
  while (ros::ok())
  {
    // poll device until end of file
    running_ = dvr_->poll();
    if (!running_)
    {
      ROS_ERROR_THROTTLE(1.0, "DriverNodelet::devicePoll - Failed to poll device.");
      interface_state_ = State::ERROR;
      dvr_->resetConnectionToInput(getPrivateNodeHandle());
    }
    else
    {
      interface_state_ = State::STREAMING;
    }
  }
  running_ = false;
}

/// \brief Timer to publish the sensor interface state every 1s
/// \param event ros timer event
void DriverNodelet::pubInterfaceStateTimerCb(const ros::WallTimerEvent& /*event*/)
{
  // Update diagnostics
  interface_diagnostics_updater_.update();

  // Publish machine-readable status to a dedicated topic
  interface_status_pub_.publish(getInterfaceStateROSMsg());
}

/// \brief Converts the current interface State into any_velodyne_msgs::VelodyneInterfaceStatus compatible format
/// \param interface_status Interface status level
/// \param interface_status_msg Additional feedback on the interface status
void DriverNodelet::convertToRosMsg(uint8_t& interface_status, std::string& interface_status_msg)
{
  switch (interface_state_.load())
  {
    case State::NONE:
      interface_status = any_velodyne_msgs::VelodyneInterfaceStatus::OK;
      interface_status_msg = "OK - Device communication not started yet";
      break;
    case State::ERROR:
      interface_status = any_velodyne_msgs::VelodyneInterfaceStatus::ERROR;
      interface_status_msg = "ERROR - Device communication error";
      break;
    case State::STOPPED:
      interface_status = any_velodyne_msgs::VelodyneInterfaceStatus::ERROR;
      interface_status_msg = "ERROR - Device communication stopped";
      break;
    case State::DISCONNECTED:
      interface_status = any_velodyne_msgs::VelodyneInterfaceStatus::ERROR;
      interface_status_msg = "ERROR - Device disconnected";
      break;
    case State::CONNECTED:
      interface_status = any_velodyne_msgs::VelodyneInterfaceStatus::OK;
      interface_status_msg = "OK - Device connected";
      break;
    case State::STREAMING:
      interface_status = any_velodyne_msgs::VelodyneInterfaceStatus::OK;
      interface_status_msg = "OK - Device streaming";
      break;
    case State::DEGRADED:
      interface_status = any_velodyne_msgs::VelodyneInterfaceStatus::WARN;
      interface_status_msg = "WARN - Device communication degraded";
      break;
    default:
      interface_status = any_velodyne_msgs::VelodyneInterfaceStatus::WARN;
      interface_status_msg = "WARN - Unknown device connection state";
      break;
  }
}

/// \brief Gets the current interface status information ready to publish to ROS diagnostics
/// \param stat
void DriverNodelet::getDiagnosticsInterfaceState(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  uint8_t interface_status_level{ diagnostic_msgs::DiagnosticStatus::ERROR };
  std::string interface_status_message{};
  convertToRosMsg(interface_status_level, interface_status_message);
  stat.summary(interface_status_level, interface_status_message);
}

/// \brief Gets the current interface status information ready to be published in a dedicated topic
/// \return any_velodyne_msgs::VelodyneInterfaceStatus message ready to be published
any_velodyne_msgs::VelodyneInterfaceStatus DriverNodelet::getInterfaceStateROSMsg()
{
  any_velodyne_msgs::VelodyneInterfaceStatus msg;
  uint8_t interface_status_level{ any_velodyne_msgs::VelodyneInterfaceStatus::OK };
  std::string interface_status_msg{};
  convertToRosMsg(interface_status_level, interface_status_msg);
  msg.interface_status = interface_status_level;
  msg.interface_status_message.data = interface_status_msg;
  return msg;
}

}  // namespace velodyne_driver

// Register this plugin with pluginlib.  Names must match nodelet_velodyne.xml.
//
// parameters are: class type, base class type
PLUGINLIB_EXPORT_CLASS(velodyne_driver::DriverNodelet, nodelet::Nodelet)
