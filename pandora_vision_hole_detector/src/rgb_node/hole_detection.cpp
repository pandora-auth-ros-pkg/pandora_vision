/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, P.A.N.D.O.R.A. Team.
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
*   * Neither the name of the P.A.N.D.O.R.A. Team nor the names of its
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
*
* Author: Despoina Paschalidou
*********************************************************************/
#include "rgb_node/hole_detection.h"

namespace pandora_vision
{
  /**
    @brief Constructor
  **/
  HoleDetection::HoleDetection(): _nh(), holeNowON(false)
  {
    //!< Get general parameters for image processing
    getGeneralParams();
    
    //!< Convert field of view from degrees to rads
    hfov = hfov * CV_PI / 180;
    vfov = vfov * CV_PI / 180;

    ratioX = hfov / frameWidth;
    ratioY = vfov / frameHeight;
    
    //!< Memory will allocated in the imageCallback
    _holeFrame= cv::Mat::zeros(frameWidth, frameHeight, CV_8UC3);
    
    //!< Subscribe to input image's topic
    //!< image_transport::ImageTransport it(_nh);
    _frameSubscriber = image_transport::ImageTransport(_nh).subscribe(
      imageTopic, 1, &HoleDetection::imageCallback, this);
    
    ROS_INFO("[hole_node] : Created Hole Detection instance");
  }
  
   /**
    @brief Destructor
  */
  HoleDetection::~HoleDetection()
  {
    ROS_DEBUG("[hole_node] : Destroying Hole Detection instance");
  }
  
  /**
    @brief Get parameters referring to the view and 
    frame characteristics
    @return void
  **/
  void HoleDetection::getGeneralParams()
  {
    packagePath = ros::package::getPath("pandora_vision_hole_detector");

    //!< Get the camera to be used by hole node;
    if (_nh.hasParam("camera_name")) 
    {
      _nh.getParam("camera_name", cameraName);
      ROS_DEBUG_STREAM("camera_name : " << cameraName);
    }
    else 
    {
      cameraName = "camera";
      ROS_DEBUG_STREAM("camera_name : " << cameraName);
    }

    //!< Get the Height parameter if available;
    if (_nh.hasParam("/" + cameraName + "/image_height")) 
    {
      _nh.getParam("/" + cameraName + "/image_height", frameHeight);
      ROS_DEBUG_STREAM("height : " << frameHeight);
    }
    else
    {
      frameHeight = RgbParameters::frameHeight;
      ROS_DEBUG_STREAM("height : " << frameHeight);
    }
    
    //!< Get the Width parameter if available;
    if (_nh.hasParam("/" + cameraName + "/image_width")) 
    {
      _nh.getParam("/" + cameraName + "/image_width", frameWidth);
      ROS_DEBUG_STREAM("width : " << frameWidth);
    }
    else 
    {
      frameWidth = RgbParameters::frameWidth;
      ROS_DEBUG_STREAM("width : " << frameWidth);
    }
    
    //!< Get the images's topic;
    if (_nh.hasParam("/" + cameraName + "/topic_name")) 
    {
      _nh.getParam("/" + cameraName + "/topic_name", imageTopic);
      ROS_DEBUG_STREAM("imageTopic : " << imageTopic);
    }
    else 
    {
      imageTopic = "/camera/rgb/image_color";
      ROS_DEBUG_STREAM("imageTopic : " << imageTopic);
    }
  
    //!< Get the images's frame_id;
    if (_nh.hasParam("/" + cameraName + "/camera_frame_id")) 
    {
      _nh.getParam("/" + cameraName + "/camera_frame_id", cameraFrameId);
      ROS_DEBUG_STREAM("camera_frame_id : " << cameraFrameId);
    }
    else 
    {
      cameraFrameId = "/camera";
      ROS_DEBUG_STREAM("camera_frame_id : " << cameraFrameId);
    }

    //!< Get the HFOV parameter if available;
    if (_nh.hasParam("/" + cameraName + "/hfov")) 
    {
      _nh.getParam("/" + cameraName + "/hfov", hfov);
      ROS_DEBUG_STREAM("HFOV : " << hfov);
    }
    else 
    {
      hfov = RgbParameters::hfov;
      ROS_DEBUG_STREAM("width : " << frameWidth);
    }
    
    //!< Get the VFOV parameter if available;
    if (_nh.hasParam("/" + cameraName + "/vfov")) 
    {
      _nh.getParam("/" + cameraName + "/vfov", vfov);
      ROS_DEBUG_STREAM("VFOV : " << vfov);
    }
    else
    {
      vfov = RgbParameters::vfov;
      ROS_DEBUG_STREAM("VFOV : " << vfov);
    }
  }
  
  /**
    @brief Function called when new ROS message appears, for camera
    @param msg [const sensor_msgs::ImageConstPtr&] The message
    @return void
  */
  void HoleDetection::imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr in_msg;
    in_msg = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    _holeFrame= in_msg->image.clone();
    _holeFrameTimestamp = msg->header.stamp;

    if (_holeFrame.empty() )
    {
      ROS_ERROR("[hole_node] : No more Frames");
      return;
    }
    
    holeCallback();
  }
  
  /**
    @brief This method uses a FaceDetector instance to detect all 
    present faces in a given frame
    @param timer [ros:TimerEvemt] the timer used to call 
    faceCallback
    @return void
  */
  void HoleDetection::holeCallback()
  {
    _holeDetector.findHoles(_holeFrame);
  } 
}// namespace pandora_vision
