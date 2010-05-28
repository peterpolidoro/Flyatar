///////////////////////////////////////////////////////////////////////////
//
// Copyright (C) 2009, 2010 Patrick Beeson, Jack O'Quin
//  ROS port of the Player 1394 camera driver.
//
// Copyright (C) 2004 Nate Koenig, Andrew Howard
//  Player driver for IEEE 1394 digital camera capture
//
// Copyright (C) 2000-2003 Damien Douxchamps, Dan Dennedy
//  Bayer filtering from libdc1394
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License as
// published by the Free Software Foundation; either version 2 of the
// License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
// 02111-1307, USA.
//
///////////////////////////////////////////////////////////////////////////

// $Id: camera1394.cpp 29743M 2010-05-28 18:39:03Z (local) $

#include <signal.h>
#include <ros/ros.h>
#include <boost/format.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <tf/transform_listener.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>
#include <driver_base/SensorLevels.h>
#include <driver_base/driver.h>

#include <sensor_msgs/image_encodings.h>
namespace enc = sensor_msgs::image_encodings;

#include "dev_camera1394.h"
#include "camera1394/Camera1394Config.h"

/** @file

    @brief ROS driver node for IEEE 1394 digital cameras.

    This is a ROS port of the Player driver for 1394 cameras, using
    libdc1394.  It provides a reliable driver with minimal dependencies,
    intended to fill a role in the ROS image pipeline similar to the other
    ROS camera drivers.

    The ROS image pipeline provides Bayer filtering at a higher level (in
    image_proc).  In some cases it is useful to run the driver without the
    entire image pipeline, so libdc1394 Bayer decoding is also provided.

    @par Advertises

    - @b camera/image_raw topic (sensor_msgs/Image) raw 2D camera images

    - @b camera/camera_info topic (sensor_msgs/CameraInfo) Calibration
    information for each image.

    @todo Make array of supported image encoding values, check parameter
    settings against that. Make enum type for dynamic reconfiguration.

*/

typedef driver_base::Driver Driver;
typedef driver_base::SensorLevels Levels;

/** Segfault signal handler */
void sigsegv_handler(int sig)
{
  signal(SIGSEGV, SIG_DFL);
  fprintf(stderr, "Segmentation fault, stopping camera driver.\n");
  ROS_ERROR("Segmentation fault, stopping camera.");
  ros::shutdown();                      // stop the main loop
}

class Camera1394Node
{
private:

  Driver::state_t state_;               // current driver state

  ros::NodeHandle privNH_;              // private node handle
  ros::NodeHandle camera_nh_;           // camera name space handle
  sensor_msgs::Image image_;
  sensor_msgs::CameraInfo cam_info_;

  /** 1394 camera device interface */
  camera1394::Camera1394 *dev_;

  /** dynamic parameter configuration */
  typedef camera1394::Camera1394Config Config;
  Config config_;

  /** camera calibration information */
  CameraInfoManager *cinfo_;

  /** image transport interfaces */
  image_transport::ImageTransport *it_;
  image_transport::CameraPublisher image_pub_;

public:

  Camera1394Node(): it_(0)
  {
    state_ = Driver::CLOSED;
    privNH_ = ros::NodeHandle("~");
    camera_nh_ = ros::NodeHandle("camera");
    cinfo_ = new CameraInfoManager(camera_nh_);
    dev_ = new camera1394::Camera1394();
  }

  ~Camera1394Node()
  {
    if (it_)
      delete it_;
    delete dev_;
    delete cinfo_;
  }

  /** Close camera device
   *
   *  postcondition: state_ is Driver::CLOSED
   */
  void closeCamera()
  {
    if (state_ != Driver::CLOSED)
      {
        ROS_INFO_STREAM("[" << config_.camera_name
                        << "] closing device, GUID: " << dev_->device_id_);
        dev_->close();
        state_ = Driver::CLOSED;
      }
  }

  /** Open the camera device.
   *
   * @return true, if successful
   *
   * if successful:
   *   state_ is Driver::OPENED
   *   device parameters loaded
   */
  bool openCamera()
  {
    bool success = true;
    try
      {
        ROS_INFO_STREAM("opening [" << config_.camera_name << "] "
                        << config_.video_mode
                        << ", fps: " << config_.fps
                        << ", iso_speed: " << config_.iso_speed
                        << ", GUID: " << config_.guid);
        if (dev_->open(config_.guid.c_str(), config_.video_mode.c_str(),
                       config_.fps, config_.iso_speed,
                       config_.bayer_pattern.c_str(),
                       config_.bayer_method.c_str())
            == 0)
          {
            ROS_INFO_STREAM("[" << config_.camera_name
                            << "] connected to device, GUID: "
                            << dev_->device_id_);
            state_ = Driver::OPENED;

            // set all the device parameters
            setDeviceParameters(config_);
          }

      }
    catch (camera1394::Exception& e)
      {
        ROS_FATAL_STREAM("[" << config_.camera_name
                         << "] exception opening device: " << e.what());
        success = false;
      }

    return success;
  }

  /** Publish camera stream topics
   *
   *  on entry: image_ contains latest camera frame
   */
  void publish()
  {
    image_.header.frame_id = config_.frame_id;

    // get current CameraInfo data
    cam_info_ = cinfo_->getCameraInfo();
    cam_info_.header.frame_id = config_.frame_id;
    cam_info_.header.stamp = image_.header.stamp;
    cam_info_.height = image_.height;
    cam_info_.width = image_.width;

    // Publish via image_transport
    image_pub_.publish(image_, cam_info_);
  }

  /** Read camera data.
   *
   * @return true if successful
   */
  bool read()
  {
    bool success = true;
    try
      {
        // Read data from the Camera
        ROS_DEBUG_STREAM("[" << config_.camera_name << "] reading data");
        dev_->readData(image_);
        ROS_DEBUG_STREAM("[" << config_.camera_name << "] read returned");

        if (config_.encoding != "")
          image_.encoding = config_.encoding; // override driver setting
      }
    catch (camera1394::Exception& e)
      {
        ROS_WARN_STREAM("[" << config_.camera_name
                        << "] Exception reading data: " << e.what());
        success = false;
      }
    return success;
  }

  /** Dynamic reconfigure callback
   *
   *  Called immediately when callback first defined. Called again
   *  when dynamic reconfigure starts or changes a parameter value.
   *
   *  @param newconfig new Config values
   *  @param level bit-wise OR of reconfiguration levels for all
   *               changed parameters (0xffffffff on initial call)
   **/
  void reconfig(Config &newconfig, uint32_t level)
  {
    ROS_INFO("dynamic reconfigure level 0x%x", level);

    // check parameter values
    if (newconfig.whitebalance == "")
      newconfig.whitebalance = "auto";  // TODO: change this to ""

    // resolve frame ID using tf_prefix parameter
    if (newconfig.frame_id == "")
      newconfig.frame_id = "camera";
    std::string tf_prefix = tf::getPrefixParam(privNH_);
    ROS_DEBUG_STREAM("tf_prefix: " << tf_prefix);
    newconfig.frame_id = tf::resolve(tf_prefix, newconfig.frame_id);

    if (newconfig.camera_name == "")
      {
        newconfig.camera_name = "camera";
      }

    if (config_.camera_name != newconfig.camera_name)
      {
        ROS_INFO_STREAM("camera name: "
                        << newconfig.camera_name);
        if (!cinfo_->setCameraName(newconfig.camera_name))
          {
            // new name not valid, use the old one
            ROS_WARN_STREAM("[" << config_.camera_name
                            << "] new name not valid: "
                            << newconfig.camera_name);
            newconfig.camera_name = config_.camera_name;
          }
      }

    if (config_.camera_info_url != newconfig.camera_info_url)
      {
        // set the new URL and load CameraInfo (if any) from it
        if (cinfo_->validateURL(newconfig.camera_info_url))
          {
            ROS_INFO_STREAM("camera_info_url: "
                            << newconfig.camera_info_url);
            cinfo_->loadCameraInfo(newconfig.camera_info_url);
          }
        else
          {
            // new URL not valid, use the old one
            newconfig.camera_info_url = config_.camera_info_url;
          }
      }

    if (state_ != Driver::CLOSED && (level & Levels::RECONFIGURE_CLOSE))
      {
        // must close the device before updating these parameters
        closeCamera();                  // state_ --> CLOSED
      }

    config_ = newconfig;                // update parameter values

    if (state_ == Driver::CLOSED)
      {
        openCamera();                   // open with new values
      }

    if (state_ != Driver::CLOSED)
      {
        // only set device parameters if open succeeded
        setDeviceParameters(config_);
      }

    ROS_DEBUG_STREAM("[" << newconfig.camera_name
                     << "] reconfigured: GUID " << newconfig.guid
                     << ", frame_id " << newconfig.frame_id
                     << ", camera_info_url " << newconfig.camera_info_url
                     << ", encoding " << newconfig.encoding);
  }

  /** Set device parameters.
   *
   *  Sends all device parameter values to the device, which must be
   *  opened already.
   *
   *  @param newconfig contains all parameter values
   */
  void setDeviceParameters(Config &newconfig)
  {
    // libdc1394 segfaults if you try to set these parameters when the
    // driver is not open
    ROS_ASSERT(state_ != Driver::CLOSED);

    if (dev_->setBrightness(newconfig.brightness) >= 0)
      {
        if (newconfig.brightness >= 0)
          ROS_INFO_STREAM("[" << config_.camera_name << "] Brightness set to "
                          << newconfig.brightness);
        else
          ROS_INFO_STREAM("[" << config_.camera_name << "] Auto Brightness set");
      }

    if (dev_->setExposure(newconfig.exposure) >= 0)
      {
        if (newconfig.exposure >= 0)
          ROS_INFO_STREAM("[" << config_.camera_name << "] Exposure set to "
                          << newconfig.exposure);
        else
          ROS_INFO_STREAM("[" << config_.camera_name << "] Auto Exposure set");
      }

    if (dev_->setGain(newconfig.gain) >= 0)
      {
        if (newconfig.gain >= 0)
          ROS_INFO_STREAM("[" << config_.camera_name << "] Gain set to "
                          << newconfig.gain);
        else
          ROS_INFO_STREAM("[" << config_.camera_name << "] Auto Gain set");
      }

    if (dev_->setShutter(newconfig.shutter) >= 0)
      {
        if (newconfig.shutter >= 0)
          ROS_INFO_STREAM("[" << config_.camera_name << "] Shutter set to "
                          << newconfig.shutter);
        else
          ROS_INFO_STREAM("[" << config_.camera_name << "] Auto Shutter set");
      }

    if (dev_->setWhiteBalance(newconfig.whitebalance.c_str()) >= 0)
      {
        if (newconfig.whitebalance != "auto")
          ROS_INFO_STREAM("[" << config_.camera_name << "] Whitebalance set to "
                          << newconfig.whitebalance);
        else
          ROS_INFO_STREAM("[" << config_.camera_name << "] Auto Whitebalance set");
      }
  }

  /** driver main spin loop */
  void spin(void)
  {
    // the bring up order is tricky
    ros::NodeHandle node;

    // define segmentation fault handler, sometimes libdc1394 craps out
    signal(SIGSEGV, &sigsegv_handler);

    // Define dynamic reconfigure callback, which gets called
    // immediately with level 0xffffffff.  The reconfig() method will
    // set initial parameter values, then open the device if it can.
    dynamic_reconfigure::Server<Config> srv;
    dynamic_reconfigure::Server<Config>::CallbackType f
      = boost::bind(&Camera1394Node::reconfig, this, _1, _2);
    srv.setCallback(f);

    // set up ROS interfaces in camera namespace
    it_ = new image_transport::ImageTransport(camera_nh_);
    image_pub_ = it_->advertiseCamera("image_raw", 1);

    while (node.ok())
      {
        if (state_ != Driver::CLOSED)
          {
            if (read())
              {
                publish();
              }
          }

        ros::spinOnce();
      }

    closeCamera();
  }

}; // end Camera1394Node class definition


/** Main entry point */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "camera1394_node");
  ros::NodeHandle node;
  Camera1394Node cm;

  cm.spin();

  return 0;
}
