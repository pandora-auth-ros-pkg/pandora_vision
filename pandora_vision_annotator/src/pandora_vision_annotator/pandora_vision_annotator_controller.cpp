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
  { QString ros_topic = connector_.getRosTopic();
    /*img_subscriber_ = n_.subscribe(
      ros_topic.toStdString(),
      1,
      &CController::receiveImage,
      this);*/
    loadBag("/home/marios/pandora_ws/benchmark_dataset.bag",ros_topic.toStdString());
    count = 0;
    currentFrame = 0;
  }

  void CController::loadBag(const std::string& filename, const std::string& topic)
  {
        rosbag::Bag bag;
        bag.open(filename, rosbag::bagmode::Read);
        rosbag::View view(bag, rosbag::TopicQuery(topic));
        // Load all messages into our stereo dataset
        BOOST_FOREACH(rosbag::MessageInstance const m, view)
        {
            sensor_msgs::Image::ConstPtr img = m.instantiate<sensor_msgs::Image>();
            if(img != NULL)
            receiveImage(img);
        }
              connector_.setFrames(frames);
              connector_.setcurrentFrame();
              qDebug("sfsfsdf");

  }

  void CController::receiveImage(const sensor_msgs::Image::ConstPtr& msg)
  {

    cv_bridge::CvImageConstPtr in_msg;
    cv::Mat temp;
    try
      {
        in_msg = cv_bridge::toCvCopy(msg);
        if(msg->encoding == "8UC1")
   	    {
   		  cv::cvtColor(in_msg->image, temp, CV_GRAY2RGB);
        }

        else if (msg->encoding == "16UC1" || msg->encoding == "32FC1")

   	    {
          double min, max;
          cv::minMaxLoc(in_msg->image, &min, &max);
   	      cv::Mat img_scaled_8u;
          cv::Mat(in_msg->image-min).convertTo(img_scaled_8u, CV_8UC1, 255. / (max - min));
          cv::cvtColor(img_scaled_8u, temp, CV_GRAY2RGB);
        }
        else if(msg->encoding == "rgb8" || msg->encoding == "bgr8" )
        {
          in_msg = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
          temp = in_msg->image;
          cv::cvtColor(in_msg->image, temp, CV_BGR2RGB); // cvtColor Makes a copt, that what i need
        }

        }

      catch(cv_bridge::Exception& e)
      {
        qWarning("CController::receiveImage() while trying to convert image from '%s' to 'rgb8' an exception was thrown ((%s)", msg->encoding.c_str(), e.what());
        //connector_.setImage((const uchar *)QImage());
        return;
      }

    /*ros::Rate loop_rate(100);*/
    /*loop_rate.sleep();*/
    count++;
    qDebug("CALLBACK %d", count);
    frames.push_back(temp);
    //connectFrame();
    //QImage dest((const uchar *) temp.data, temp.cols, temp.rows, temp.step, QImage::Format_RGB888);
    //dest.bits(); // enforce deep copy, see documentation

    //connector_.setImage(dest);
    //Q_EMIT updateImage();

}
void CController::connectFrame()
{
    qDebug("%ld",frames.size());
    for(int i = 0; i < frames.size(); i++)
    {
      QImage dest((const uchar *) frames[currentFrame].data, frames[currentFrame].cols, frames[currentFrame].rows, frames[currentFrame].step, QImage::Format_RGB888);
      dest.bits(); // enforce deep copy, see documentation
      connector_.setImage(dest);
      Q_EMIT updateImage();
      //currentFrame++;
    }

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


