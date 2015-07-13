#include <string>
#include "sensor_msgs/Image.h"
#include <sensor_msgs/image_encodings.h>

#include "pandora_vision_obstacle/hard_obstacle_detection/dummy_processor.h"

namespace pandora_vision
{
namespace pandora_vision_obstacle
{
  DummyProcessor::DummyProcessor() : it(nh)
  {
    detector_.reset(new HardObstacleDetector("detector", nh));
    if (detector_ == NULL)
    {
      ROS_FATAL("Could not initialize Hard Obstacle Detector!");
      ROS_BREAK();
    }
    sub = it.subscribe("/elevation_map", 1,
      &DummyProcessor::imageCallback, this);
    pub = it.advertise("/traversability_map", 1);
  }

  void DummyProcessor::imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_64FC1);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
    CVMatStamped inputImage;
    inputImage.image = cv_ptr->image;
    inputImage.header = cv_ptr->header;

    CVMatStampedPtr inputImagePtr;
    *inputImagePtr = inputImage;

    CVMatStampedPtr outputImagePtr;

    bool flag = process(inputImagePtr, outputImagePtr);

    cv_bridge::CvImage out_msg;
    out_msg.header   = outputImagePtr->header; // Same timestamp and tf frame as input image
    out_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC1; // Or whatever
    out_msg.image    = outputImagePtr->image; // Your cv::Mat
    if (flag)
      pub.publish(out_msg.toImageMsg());
  }

  bool DummyProcessor::process(const CVMatStampedConstPtr& input,
      const CVMatStampedPtr& output)
  {
    output->header = input->getHeader();
    output->image = detector_->startDetection(input->getImage());

    return true;
  }

}
}
