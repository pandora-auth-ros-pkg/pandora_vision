/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, P.A.N.D.O.R.A. Team.
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
 * Authors: Choutas Vassilis 
 *********************************************************************/


#include "pandora_vision_hazmat/hazmat_detection.h"

HazmatDetectionNode::HazmatDetectionNode()
{
  // TO DO : use nodehandle get params.
  imageTopic_ = std::string("/camera/image_raw");
  imageSubscriber_ = nodeHandle_.subscribe( imageTopic_ , 1 ,
      &HazmatDetectionNode::imageCallback, this);
  //detector_ = SiftHazmatDetector(); 
  //
  hazmatTopic_ = std::string("/alert/hazmat");
  
  hazmatPublisher_ = nodeHandle_.advertise<std_msgs::Bool>(hazmatTopic_, 10);
}

void HazmatDetectionNode::imageCallback(const sensor_msgs::Image& inputImage)
{
  cv_bridge::CvImagePtr imgPtr;
  std::stringstream ss;
  // Convert the image message to an OpenCV image.
  imgPtr = cv_bridge::toCvCopy(inputImage, sensor_msgs::image_encodings::BGR8);
  float x , y; 
  const clock_t begin_time = clock();

  bool found = detector_.detect(imgPtr->image , &x, &y);
  // bool found = detector.detect(frame , &x, &y);
  double execTime = ( clock () - begin_time ) /  
    static_cast<double>(CLOCKS_PER_SEC );
  std::cout <<"Time to execute : " << execTime << std::endl; 

  if (found )
  {
    ROS_INFO("Found Hazmat! \n");
    std_msgs::Bool msg;
    msg.data = found;
    hazmatPublisher_.publish(msg);
  }  
  ss << 1 / execTime;

  cv::putText( imgPtr->image , "fps : " + ss.str() , cv::Point( 0 , imgPtr->image.rows ) ,
      CV_FONT_HERSHEY_PLAIN , 3 , cv::Scalar( 0 , 0 , 255 ) , 3 );


  // Clear the string stream.
  ss.str( std::string() );
  
  // TO DO : Add visualization flag
  cv::imshow("Input Image" , imgPtr->image);
  if ( cv::waitKey(5) >= 0)
  {
    ROS_INFO("Goodbye! \n");
    ros::shutdown();
  }
  }
