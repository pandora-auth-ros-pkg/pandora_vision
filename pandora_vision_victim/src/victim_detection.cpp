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

#include "pandora_vision_victim/victim_detection.h"

namespace pandora_vision
{
/**
  @brief Constructor
**/
VictimDetection::VictimDetection() : _nh(), victimNowON(false)
{
  //!< Get general parameters for image processing
  getGeneralParams();

  //!< Convert field of view from degrees to rads
  hfov = hfov * CV_PI / 180;
  vfov = vfov * CV_PI / 180;

  ratioX = hfov / frameWidth;
  ratioY = vfov / frameHeight;

  //!< Memory will allocated in the imageCallback
  victimFrame = cv::Mat::zeros(frameWidth, frameHeight, CV_8UC3);

  //!< Subscribe to input image's topic
  //!< image_transport::ImageTransport it(_nh);
  _frameSubscriber = image_transport::ImageTransport(_nh).subscribe(
                       imageTopic, 1, &VictimDetection::imageCallback, this);

  //!< Initialize states - robot starts in STATE_OFF
  curState = state_manager_communications::robotModeMsg::MODE_OFF;
  prevState = state_manager_communications::robotModeMsg::MODE_OFF;

  clientInitialize();

  ROS_INFO("[victim_node] : Created Victim Detection instance");
}

/**
  @brief Destructor
*/
VictimDetection::~VictimDetection()
{
  ROS_DEBUG("[victim_node] : Destroying Victim Detection instance");
}

/**
 @brief Get parameters referring to the view and
 *frame characteristics
 @return void
**/
void VictimDetection::getGeneralParams()
{
  packagePath = ros::package::getPath("pandora_vision_victim");

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
    frameHeight = DEFAULT_HEIGHT;
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
    frameWidth = DEFAULT_WIDTH;
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
    imageTopic = "/camera_head/image_raw";
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
    hfov = HFOV;
    ROS_DEBUG_STREAM("HFOV : " << hfov);
  }

  //!< Get the VFOV parameter if available;
  if (_nh.hasParam("/" + cameraName + "/vfov"))
  {
    _nh.getParam("/" + cameraName + "/vfov", vfov);
    ROS_DEBUG_STREAM("VFOV : " << vfov);
  }
  else
  {
    vfov = VFOV;
    ROS_DEBUG_STREAM("VFOV : " << vfov);
  }

}



/**
 * @brief Function called when new ROS message appears, for camera
 * @param msg [const sensor_msgs::ImageConstPtr&] The message
 * @return void
 */
void VictimDetection::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr in_msg;
  in_msg = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  victimFrame = in_msg->image.clone();
  victimFrameTimestamp = msg->header.stamp;

  if (victimFrame.empty() )
  {
    ROS_ERROR("[victim_node] : No more Frames ");
    return;
  }
}

/**
 * @brief This method uses a FaceDetector instance to detect all
 * present faces in a given frame
 * @return void
*/
void VictimDetection::victimCallback()
{
  if(!victimNowON)
  {
    return;
  }

}


/**
  * @brief Node's state manager
  * @param newState [int] The robot's new state
  * @return void
 */
void VictimDetection::startTransition(int newState)
{

  curState = newState;

  //!< check if face detection algorithm should be running now
  victimNowON = 
  (curState == state_manager_communications::robotModeMsg::MODE_EXPLORATION) ||
  (curState == state_manager_communications::robotModeMsg::MODE_ARM_APPROACH) ||
  (curState == state_manager_communications::robotModeMsg::MODE_DF_HOLD);

  //!< shutdown if the robot is switched off
  if (curState == state_manager_communications::robotModeMsg::MODE_TERMINATING)
  {
    ros::shutdown();
    return;
  }

  prevState = curState;

  //!< this needs to be called everytime a node finishes transition
  transitionComplete(curState);
}

/**
 * @brief After completion of state transition
 * @return void
 */
void VictimDetection::completeTransition(void)
{
  ROS_INFO("[victim_node] : Transition Complete");
}
}// namespace pandora_vision
