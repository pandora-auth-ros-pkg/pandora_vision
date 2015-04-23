/******************************************************************************
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
* Authors: Manos Tsardoulias
*********************************************************************/

#ifndef PANDORA_VISION_ANNOTATOR_CONTROLLER
#define PANDORA_VISION_ANNOTATOR_CONTROLLER

#include "pandora_vision_annotator/pandora_vision_annotator_connector.h"
/**
@namespace pandora_vision
@brief The main namespace for pandora vision
**/
namespace pandora_vision
{

  /**
  @class CController
  @brief The main controller for the pandora annotator. Inherits QThread
  **/
  class CController :
    public QThread
  {
    Q_OBJECT

    //------------------------------------------------------------------------//
    private:

      //!< Number of input arguments
      int  argc_;
      //!< Input arguments
      char** argv_;

      //!< ROS subscriber for the image topic
      ros::Subscriber img_subscriber_;


      //!< ROS subscriber for the predator alert
      ros::Subscriber predatorSubscriber_;
      

      //!< ROS publisher for the predator 
      ros::Publisher annotationPublisher_;


      //!< The ROS node handle
      ros::NodeHandle n_;
      //!< QImage created one time, containing the frame from the input topic
      QImage topic_img_;

      //!< Object of CConnector
      CConnector connector_;

      //!< Vector of frames
      std::vector<cv::Mat> frames;

      std::vector<std_msgs::Header> msgHeader_;
     //sensors_msgs::Image::ConstPtr imgs;
      int count,prevcount;

      int currentFrameNo_;

      int baseFrame;

      bool onlinemode;

      bool PredatorNowOn;

    //------------------------------------------------------------------------//
    public:

      /**
      @brief Default contructor
      @param argc [int] Number of input arguments
      @param argv [char **] Input arguments
      @return void
      **/
      CController(int argc,char **argv);

      /**
      @brief Default destructor
      @return void
      **/
      ~CController(void);

      /**
      @brief Initializes the Qt event connections and ROS subscribers and publishers
      @return void
      **/
      void initializeCommunications(void);

      /**
      @brief Initializes the ROS spin and Qt threads
      @return bool
      **/
      bool init();

      void receivePointCloud(const sensor_msgs::PointCloud2ConstPtr& msg);
      void receiveImage(const sensor_msgs::ImageConstPtr& msg);
      void loadBag(const std::string& filename, const std::string& topic);
      void predatorCallback(const pandora_vision_msgs::PredatorMsg& msg);
      cv::Mat getFrame(int x);



         //------------------------------------------------------------------------//
    public Q_SLOTS:

      void rosTopicGiven(void);
      void predatorEnabled(void);
      void onlineModeGiven(void);
      void offlineModeGiven(void);
    
    //------------------------------------------------------------------------//
    Q_SIGNALS:
      void updateImage();
    };
}

#endif

