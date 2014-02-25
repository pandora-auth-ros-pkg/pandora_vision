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
 * Author: Miltiadis-Alexios Papadopoulos
 *********************************************************************/

#include "pandora_vision_qrcode/qrCode_detection.h"

namespace vision
{
  /**
    @brief Constructor
   **/
  QrCodeDetection::QrCodeDetection() : _nh(), qrcodeNowON(false)
  {
    //!< Get Motion Detector Parameters
    getQrCodeParams();

    //!< Get General Parameters, such as frame width & height , camera id
    getGeneralParams();

    //!< Convert field of view from degrees to rads
    hfov = HFOV * CV_PI / 180;
    vfov = VFOV * CV_PI / 180;

    ratioX = hfov / frameWidth;
    ratioY = vfov / frameHeight;

    //!< Declare publisher and advertise topic
    //!< where algorithm results are posted
    _qrcodePublisher =
      _nh.advertise<vision_communications::QRAlertsVectorMsg>("qr_alert", 10);

    //!< Advertise topics for debugging if we are in debug mode
    if (debugQrCode)
    {
      _qrcodeDetector.set_debug(true);
      _qrcodeDebugPublisher =
        image_transport::ImageTransport(_nh).advertise("debug_qrcode", 1);
    }

    //!< subscribe to input image's topic
    //!< image_transport::ImageTransport it(_nh);
    _frameSubscriberFront = image_transport::ImageTransport(_nh).subscribe(
        imageTopic, 1, &QrCodeDetection::imageCallbackFront, this);
    _frameSubscriberBack = image_transport::ImageTransport(_nh).subscribe(
        imageTopicback, 1, &QrCodeDetection::imageCallbackBack, this);

    //!< initialize states - robot starts in STATE_OFF
    curState = state_manager_communications::robotModeMsg::MODE_OFF;
    prevState = state_manager_communications::robotModeMsg::MODE_OFF;

    clientInitialize();

    ROS_INFO("[QrCodeNode] : Created QrCode Detection instance");
  }



  /**
    @brief Destructor
   */
  QrCodeDetection::~QrCodeDetection()
  {
    ROS_INFO("[QrCodeNode] : Destroying QrCode Detection instance");
  }



  /**
   * @brief Get parameters referring to view and frame characteristics from
   * launch file
   * @return void
   */
  void QrCodeDetection::getGeneralParams()
  {
    //!< Get the qrcodeDummy parameter if available;
    if (_nh.hasParam("qrcodeDummy"))
    {
      _nh.getParam("qrcodeDummy", qrcodeDummy);
      ROS_DEBUG("qrcodeDummy: %d", qrcodeDummy);
    }
    else
    {
      ROS_DEBUG("[QrCodeNode] : \
          Parameter qrcodeDummy not found. Using Default");
      qrcodeDummy = false;
    }

    //!< Get the debugQrCode parameter if available;
    if (_nh.hasParam("debugQrCode"))
    {
      _nh.getParam("debugQrCode", debugQrCode);
      ROS_DEBUG_STREAM("debugQrCode : " << debugQrCode);
    }
    else
    {
      ROS_DEBUG("[QrCodeNode] : \
          Parameter debugQrCode not found. Using Default");
      debugQrCode = true;
    }

    //!< Get the Height parameter if available;
    if (_nh.hasParam("height"))
    {
      _nh.getParam("height", frameHeight);
      ROS_DEBUG_STREAM("height : " << frameHeight);
    }
    else
    {
      ROS_DEBUG("[QrCodeNode] : \
          Parameter frameHeight not found. Using Default");
      frameHeight = DEFAULT_HEIGHT;
    }

    //!< Get the listener's topic;
    if (_nh.hasParam("imageTopic"))
    {
      _nh.getParam("imageTopic", imageTopic);
      ROS_DEBUG_STREAM("imageTopic : " << imageTopic);
    }
    else
    {
      ROS_DEBUG("[QrCodeNode] : Parameter imageTopic not found. Using Default");
      imageTopic = "/camera_head/image_raw";
    }

    //!< Get the Width parameter if available;
    if (_nh.hasParam("width"))
    {
      _nh.getParam("width", frameWidth);
      ROS_DEBUG_STREAM("width : " << frameWidth);
    }
    else
    {
      ROS_DEBUG("[QrCodeNode] : Parameter frameWidth not found. Using Default");
      frameWidth = DEFAULT_WIDTH;
    }
  }



  /**
   * @brief Get parameters referring to Qrcode detection algorithm
   * @return void
   */
  void QrCodeDetection::getQrCodeParams()
  {
    //!< Get the buffer size parameter if available;
    if (_nh.hasParam("qrcodeSharpenBlur"))
    {
      _nh.getParam("qrcodeSharpenBlur", _qrcodeDetector.gaussiansharpenblur);
      ROS_DEBUG_STREAM("qrcodeSharpenBlur : "
          << _qrcodeDetector.gaussiansharpenblur);
    }
    else
    {
      ROS_DEBUG("[QrCodeNode] : \
          Parameter qrcodeSharpenBlur not found. Using Default");
      _qrcodeDetector.gaussiansharpenblur = 5;
    }

    //!< Get the difference threshold parameter if available;
    if (_nh.hasParam("qrcodeSharpenWeight"))
    {
      _nh.getParam("qrcodeSharpenWeight",
          _qrcodeDetector.gaussiansharpenweight);
      ROS_DEBUG_STREAM("qrcodeSharpenWeight : "
          << _qrcodeDetector.gaussiansharpenweight);
    }
    else
    {
      ROS_DEBUG("[QrCodeNode] : \
          Parameter qrcodeSharpenWeight not found. Using Default");
      _qrcodeDetector.gaussiansharpenweight = 0.8;
    }
  }



  /**
   * @brief Function called when new ROS message appears, for front camera
   * @param msg [const sensor_msgs::ImageConstPtr&] The message
   * @return void
   */
  void QrCodeDetection::imageCallbackFront(
      const sensor_msgs::ImageConstPtr& msg)
  {
    int res = -1;

    cv_bridge::CvImagePtr in_msg;
    in_msg = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    qrcodeFrame = in_msg->image.clone();
    qrcodeFrameTimestamp = msg->header.stamp;

    if (qrcodeFrame.empty())
    {
      ROS_ERROR("[qrcodeNode] : \
          No more Frames or something went wrong with bag file");
      ros::shutdown();
      return;
    }

    if(!qrcodeNowON)
    {
      return;
    }

    qrcodeDetectAndPost("headCamera");
  }



  /**
   * @brief Function called when new ROS message appears, for rear camera
   * @param msg [const sensor_msgs::ImageConstPtr&] The message
   * @return void
   */
  void QrCodeDetection::imageCallbackBack(const sensor_msgs::ImageConstPtr& msg)
  {
    int res = -1;

    cv_bridge::CvImagePtr in_msg;
    in_msg = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    qrcodeFrame = in_msg->image.clone();
    qrcodeFrameTimestamp = msg->header.stamp;

    if (qrcodeFrame.empty())
    {
      ROS_ERROR("[qrcodeNode] : \
          No more Frames or something went wrong with bag file");
      ros::shutdown();
      return;
    }

    if(!qrcodeNowON)
    {
      return;
    }

    qrcodeDetectAndPost("assCamera");
  }



  /**
   * @brief This method uses a QrCodeDetector instance to detect all present
   * qrcodes in a given frame
   * @param frame_id [std::string] The frame id
   * @return void
   */
  void QrCodeDetection::qrcodeDetectAndPost(std::string frame_id)
  {

    //!< Create message of QrCode Detector
    vision_communications::QRAlertsVectorMsg qrcodeVectorMsg;

    vision_communications::QRAlertMsg qrcodeMsg;

    if (qrcodeDummy)
    {
      //!< Motion Dummy Message
      qrcodeVectorMsg.header.frame_id = "QrCode";
      qrcodeVectorMsg.header.stamp = ros::Time::now();

      qrcodeMsg.QRcontent = "It's peanut butter jelly time!";
      qrcodeMsg.yaw = 0;
      qrcodeMsg.pitch = 0;

      qrcodeVectorMsg.qrAlerts.push_back(qrcodeMsg);

      //!< dummy delay
      usleep(1000 * 60);
    }
    else
    {
      /*
       * QrCode Message
       */

      //!< do detection and examine result cases
      qrcodeVectorMsg.header.frame_id = frame_id;
      qrcodeVectorMsg.header.stamp = ros::Time::now();

      _qrcodeDetector.detect_qrcode(qrcodeFrame);
      std::vector<QrCode> list_qrcodes = _qrcodeDetector.get_detected_qr();

      for(int i=0; i<(int)list_qrcodes.size(); i++)
      {
        qrcodeMsg.QRcontent = list_qrcodes[i].qrcode_desc;

        qrcodeMsg.yaw = ratioX *
          (list_qrcodes[i].qrcode_center.x - (double)frameWidth / 2);
        qrcodeMsg.pitch = -ratioY *
          (list_qrcodes[i].qrcode_center.y - (double)frameHeight / 2);

        std::cout<< "qr found!" << std::endl;

        qrcodeVectorMsg.qrAlerts.push_back(qrcodeMsg);

        std::cout << "QR found." << std::endl ;
      }

      if (debugQrCode)
      {
        publish_debug_images();
      }
    }

    if(qrcodeVectorMsg.qrAlerts.size() > 0)
    {
      _qrcodePublisher.publish(qrcodeVectorMsg);
    }
  }



  /**
   * @brief Publishing debug images
   * @return void
   */
  void QrCodeDetection::publish_debug_images()
  {
    cv_bridge::CvImage qrcodeDebug;
    qrcodeDebug.encoding = sensor_msgs::image_encodings::MONO8;
    qrcodeDebug.image = _qrcodeDetector.get_debug_frame().clone();
    _qrcodeDebugPublisher.publish(qrcodeDebug.toImageMsg());
  }



  /**
   * @brief Node's state manager
   * @param newState [int] The robot's new state
   * @return void
   */
  void QrCodeDetection::startTransition(int newState){

    curState = newState;

    //!< check if QR algorithm should be running now
    qrcodeNowON	=
      (curState ==
       state_manager_communications::robotModeMsg::MODE_EXPLORATION)
      || (curState ==
          state_manager_communications::robotModeMsg::MODE_IDENTIFICATION)
      || (curState ==
          state_manager_communications::robotModeMsg::MODE_ARM_APPROACH)
      || (curState ==
          state_manager_communications::robotModeMsg::MODE_TELEOPERATED_LOCOMOTION)
      || (curState ==
          state_manager_communications::robotModeMsg::MODE_DF_HOLD);

    //!< shutdown if the robot is switched off
    if (curState ==
        state_manager_communications::robotModeMsg::MODE_TERMINATING)
    {
      ros::shutdown();
      return;
    }

    prevState=curState;

    //!< this needs to be called everytime a node finishes transition
    transitionComplete(curState);
  }



  /**
   * @brief After completion of state transition
   * @return void
   */
  void QrCodeDetection::completeTransition()
  {
    ROS_INFO("[QrCodeNode] : Transition Complete");
  }
}

