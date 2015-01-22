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
  float x,y; 
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
  ss << 1 / execTime ;

  cv::putText( imgPtr->image , "fps : " + ss.str() , cv::Point( 0 , imgPtr->image.rows ) ,
      CV_FONT_HERSHEY_PLAIN , 3 , cv::Scalar( 0 , 0 , 255 ) , 3 ) ;


  // Clear the string stream.
  ss.str( std::string() ) ;
  
  // TO DO : Add visualization flag
  cv::imshow("Input Image",imgPtr->image);
  if ( cv::waitKey(5) >= 0)
  {
    ROS_INFO("Goodbye! \n");
    ros::shutdown();
  }
  }
