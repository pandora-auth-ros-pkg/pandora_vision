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

#ifndef QrCodeDetection_H
#define QrCodeDetection_H

#include <iostream>
#include <stdlib.h>
#include <opencv/cvwimage.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include "state_client.h"
#include <std_srvs/Empty.h>
#include "vision_communications/QRAlertsVectorMsg.h"
#include "qrCode_detector.h"

//!< Horizontal field of view in degrees : giwrgos 61.142
#define HFOV 61.14  //68

//!< vertical field of view in degrees : giwrgos 47.79
#define VFOV 48     //50

//!< default frame height
#define DEFAULT_HEIGHT 480

//!< default frame width
#define DEFAULT_WIDTH 640

namespace vision {

  class QrCodeDetection : public StateClient {

    private:

      //!< The NodeHandle
      ros::NodeHandle _nh;

      QrCodeDetector _qrcodeDetector;

      float ratioX;
      float ratioY;

      //!< Horizontal Field Of View (rad)
      float hfov;

      //!< Vertical Field Of View (rad)
      float vfov;

      int frameWidth;
      int frameHeight;

      //!< Frame processed by MotionDetector
      cv::Mat qrcodeFrame;

      //!< MotionDetector frame timestamp
      ros::Time	qrcodeFrameTimestamp;

      //!< The topic subscribed to for the front camera
      std::string imageTopic;

      //!< The topic subscribed to for the rear camera
      std::string imageTopicback;

      //!< Publishers for QrCodeDetector result messages
      ros::Publisher _qrcodePublisher;

      //!< The subscriber that listens to the frame topic advertised by the
      //!< central node for the front camera
      image_transport::Subscriber _frameSubscriberFront;

      //!< The subscriber that listens to the frame topic advertised by the
      //!< central node for the rear camera
      image_transport::Subscriber _frameSubscriberBack;

      //!< Debug publisher for MotionDetector
      image_transport::Publisher _qrcodeDebugPublisher;

      //!< Variables for changing in dummy msg mode for debugging
      bool qrcodeDummy;

      //!< Variables for changing in debug mode. Publish images for debugging
      bool debugQrCode;

      //!< Variable used for State Managing
      bool qrcodeNowON;

      /**
       * @brief Get parameters referring to view and frame characteristics from
       * launch file
       * @return void
       */
      void getGeneralParams();

      /**
       * @brief Get parameters referring to Qrcode detection algorithm
       * @return void
       */
      void getQrCodeParams();

      /**
       * @brief Publishing debug images
       * @return void
       */
      void publish_debug_images();

      /**
       * @brief This method uses a QrCodeDetector instance to detect all present
       * qrcodes in a given frame
       * @param frame_id [std::string] The frame id
       * @return void
       */
      void qrcodeDetectAndPost(std::string frame_id);

      /**
       * Function called when new ROS message appears, for front camera
       * @param msg [const sensor_msgs::ImageConstPtr&] The message
       * @return void
       */
      void imageCallbackFront(const sensor_msgs::ImageConstPtr& msg);

      /**
       * @brief Function called when new ROS message appears, for rear camera
       * @param msg [const sensor_msgs::ImageConstPtr&] The message
       * @return void
       */
      void imageCallbackBack(const sensor_msgs::ImageConstPtr& msg);

      //!< Current state of robot
      int curState;

      //!< Previous state of robot
      int prevState;


    public:

      //!< The Constructor
      QrCodeDetection();

      //!< The Destructor
      virtual ~QrCodeDetection();

      /**
       * @brief Node's state manager
       * @param newState [int] The robot's new state
       * @return void
       */
      void startTransition(int newState);

      /**
       * @brief After completion of state transition
       * @return void
       */
      void completeTransition(void);

  };
}
#endif
