#ifndef PANDORA_VISION_PREDATOR_PREDATOR_H
#define PANDORA_VISION_PREDATOR_PREDATOR_H

#include "ros/ros.h"
#include <ros/package.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <iostream>
#include <stdlib.h>

#include "TLD.h"


//!< default frame height
#define DEFAULT_HEIGHT 480

//!< default frame width
#define DEFAULT_WIDTH 640

namespace pandora_vision
{

class Predator
{
  private:
  
    //!<Node Handler
    ros::NodeHandle _nh;
    
    //!<Subscriber of RGB Image
    ros::Subscriber _inputImageSubscriber;
  
    //!< Current frame to be processed
    cv::Mat PredatorFrame;
    
    //!< Grey frame of type 8UC1
    cv::Mat grey;
    
    //!< The topic subscribed to for the front camera
    std::string imageTopic;
    
    //!< Frame height
    int frameHeight;
  
    //!< Frame width
    int frameWidth;
    
    //!<Camera Name

    std::string cameraName;
    
    //!<Frame ID
    
    std::string cameraFrameId;
    
    //!<Pointer to TLD Instance
    
    tld::TLD *tld; 
    
    //!<Semaphore used for synchronization
    bool semaphore_locked;
    
    //!<Path of exported model
    const char* modelExportFile;
    
    //!<Path of imported model
    const char* modelPath;
    
    /**
    @brief Callback for the RGB Image
    @param msg [const sensor_msgs::ImageConstPtr& msg] The RGB Image
    @return void
    **/
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    
    
  public:
  
  
  
  /**
  @brief Default Constructor
  @return void
  **/
  Predator();
  
  /**
  @brief Get parameters referring to view and frame characteristics
  @return void
  **/
  void getGeneralParams();
  
  /**
  @brief Default Destructor
  @return void
  **/
  ~Predator();
};
} // namespace pandora_vision
#endif  // PANDORA_VISION_PREDATOR_PREDATOR_H
  
  
