/******************************************************************************
   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software Foundation,
   Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA
   
   Authors : 
   * Manos Tsardoulias, etsardou@gmail.com
******************************************************************************/

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


