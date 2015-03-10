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

#include "pandora_vision_annotator/pandora_vision_annotator_controller.h"


namespace pandora_vision
{
  /**
  @brief Thread that performs the ros::spin functionality
  @return void
  **/
  void spinThreadFunction(void)
  {
    ros::spin();
  }
  
  /**
  @brief Default contructor
  @param argc [int] Number of input arguments
  @param argv [char **] Input arguments
  @return void
  **/
  CController::CController(int argc,char **argv):
    connector_(argc,argv),
    argc_(argc),
    argv_(argv)
  {
  }

  /**
  @brief Default destructor
  @return void
  **/
  CController::~CController(void)
  {
    
  }
  
  /**
  @brief Initializes the Qt event connections and ROS subscribers and publishers
  @return void
  **/
  void CController::initializeCommunications(void)
  {
    QObject::connect(
      &connector_,SIGNAL(rosTopicGiven()),
      this, SLOT(rosTopicGiven()));
      
    QObject::connect(
      this,SIGNAL(updateImage()),
      &connector_, SLOT(updateImage()));
  }
  
  void CController::rosTopicGiven(void)
  {
    QString ros_topic = connector_.getRosTopic();
    img_subscriber_ = n_.subscribe(
      ros_topic.toStdString(), 
      1, 
      &CController::receiveImage,
      this);
  }
  
  void CController::receiveImage(const sensor_msgs::Image& msg)
  {
    cv_bridge::CvImagePtr in_msg;
    in_msg = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat temp, src = in_msg->image;
    
    cvtColor(src, temp, CV_BGR2RGB); // cvtColor Makes a copt, that what i need
    QImage dest((const uchar *) temp.data, temp.cols, temp.rows, temp.step, QImage::Format_RGB888);
    dest.bits(); // enforce deep copy, see documentation 
    
    connector_.setImage(dest);
    Q_EMIT updateImage();
  }
    
 
  /**
  @brief Initializes the ROS spin and Qt threads
  @return bool
  **/
  bool CController::init(void)
  {
    if ( ! ros::master::check() ) 
    {
      return false;
    }
    connector_.show();

    initializeCommunications();
    boost::thread spinThread(&spinThreadFunction);
    return true;
  }
}


