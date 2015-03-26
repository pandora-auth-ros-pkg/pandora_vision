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


#include "pandora_vision_hazmat/detection/hazmat_detection.h"

HazmatDetectionNode::HazmatDetectionNode()
{
  ROS_INFO("Starting Hazmat Detection Node!\n");
  std::string ns = nodeHandle_.getNamespace();
  if (!nodeHandle_.getParam(ns + "subscriber_topic_names/camera_topic",
        imageTopic_))
  {
    ROS_ERROR("Could not get the topic name for image subscriber!\n");
    ROS_BREAK();
  }
  imageSubscriber_ = nodeHandle_.subscribe( imageTopic_ , 1 ,
      &HazmatDetectionNode::imageCallback, this);
  // Get the path of the current package.
  std::string packagePath = ros::package::getPath("pandora_vision_hazmat");   
  PlanarObjectDetector::setFileName(packagePath);
  DetectorFactory factory;

  // Initiliaze the object detector
  std::string featureType;
  ROS_INFO("Node name space is : %s \n", ns.c_str());
  if ( !nodeHandle_.getParam(ns + "/feature_type", featureType) )
  {
    ROS_ERROR("Could not get the feature type parameter!\n");
    ROS_INFO("Initializing default detector!\n");
    featureType = "SIFT"; 
  }
  detector_ = factory.createDetectorObject(featureType);
  // Check if the detector was initialized.
  if (detector_ == NULL )
  {
    ROS_FATAL("Could not create a detector object!\n");
    ROS_FATAL("The node will now exit!\n");
    ROS_BREAK();
  }
  ROS_INFO("Created the detector object!\n");
  
  // Initialize the alert Publisher.
  if (!nodeHandle_.getParam(ns + "publisher_topic_names/alert_topic",
        hazmatTopic_))
  {
    ROS_ERROR("Could not get the topic name for the alert publisher!\n");
    ROS_BREAK();
  }
  hazmatPublisher_ = nodeHandle_.advertise<pandora_vision_msgs::HazmatAlertsVectorMsg>(hazmatTopic_, 10);

}


void HazmatDetectionNode::imageCallback(const sensor_msgs::Image& inputImage)
{
  std::vector<Object> detectedObjects;
  cv_bridge::CvImagePtr imgPtr;
  std::stringstream ss;
  // Convert the image message to an OpenCV image.
  imgPtr = cv_bridge::toCvCopy(inputImage, sensor_msgs::image_encodings::BGR8);
  const clock_t begin_time = clock();

  bool found = detector_->detect(imgPtr->image , &detectedObjects);
  // bool found = detector.detect(frame , &x, &y);
  double execTime = ( clock () - begin_time ) /  
    static_cast<double>(CLOCKS_PER_SEC );
  if (found )
  {
    int width = 640 ;
    int height = 480;
    float hfov = 58 * PI/180.0f;
    float vfov = 45.0 * PI / 180.0f;
    float x, y;
    ROS_INFO("Found Hazmat! \n");
    pandora_vision_msgs::HazmatAlertMsg hazmatMsg;
    pandora_vision_msgs::HazmatAlertsVectorMsg hazmatVectorMsg;
    hazmatVectorMsg.header.stamp = inputImage.header.stamp;
    for (int i = 0 ; i < detectedObjects.size() ; i++)
    {
      x = detectedObjects[i].position.x - static_cast<float>(width) / 2;
      y = static_cast<float>(height) / 2 - detectedObjects[i].position.y ;
      hazmatMsg.yaw = atan(2 * x / width * tan(hfov / 2));
      hazmatMsg.pitch = atan(2 * y / height * tan(vfov / 2));
      hazmatMsg.patternType = 0;
      hazmatVectorMsg.hazmatAlerts.push_back(hazmatMsg);
    }
    ROS_INFO("Number of Hazmats Found = %d .", static_cast<int>(
          detectedObjects.size()));
    hazmatPublisher_.publish(hazmatVectorMsg);
  }  
  ROS_INFO("Execution Time for the node is : %f!\n", execTime);
  ss << execTime * 1000;

  cv::putText( imgPtr->image , "Exec Time : " + ss.str() + "ms", 
      cv::Point( 0 , imgPtr->image.rows ) ,
      CV_FONT_HERSHEY_PLAIN , 3 , cv::Scalar( 0 , 0 , 255 ) , 3 );


  // Clear the string stream.
  ss.str( std::string() );
  
  // TO DO : Add visualization flag
  cv::imshow("Input Image" , imgPtr->image);
  if ( cv::waitKey(10) >= 0)
  {
    ROS_INFO("Goodbye! \n");
    ros::shutdown();
  }
}
