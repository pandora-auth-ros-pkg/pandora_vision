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
 * Authors: Manos Tsardoulias
 *********************************************************************/
 
#include <iostream>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include "std_msgs/String.h"

/**
 @struct Params
 @brief Holds essential parameters for image aquisition
**/
struct Params
{
  static std::string folder;  // The folder for the images to be saved
  static unsigned int width;  // The width for the image to be resized
  static unsigned int height; // The height of the image to be resized
};

//! Initialization of the static members
std::string Params::folder = "~/Desktop";
unsigned int Params::width = 0;
unsigned int Params::height = 0;

/**
 * @brief Callback for the requested image topic. Shows the image and 
 * provides the keybinding 's' to save the image
 **/
void imageCallback(const sensor_msgs::Image& msg)
{
  //! Static initialization of the counter
  static unsigned int counter = 0;
  
  //!< Current frame to be processed
  cv::Mat current_frame;
  cv_bridge::CvImagePtr in_msg;
  in_msg = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  current_frame = in_msg -> image.clone();
  //! Show the image
  cv::imshow("image", current_frame);

  //! Catch key presses in the opencv window
  int keyCode = cv::waitKey(5)&255;
  switch (keyCode)
  {
    case 's':   //! Saves the image
    {
      //! Cast counter to string
      std::ostringstream convert;
      convert << counter;
      
      //! Produce image path
      std::string file_name = Params::folder + "/image_" 
        + convert.str() + ".png";
      
      //! Resize if requested by the user
      if(Params::width != 0 && Params::height != 0)
      {
        cv::resize(current_frame, current_frame, cv::Size(Params::width,
          Params::height));
      } 
      
      //! Save the image
      cv::imwrite( file_name.c_str() , current_frame );
      counter++;
      ROS_INFO("Image saved : %s", file_name.c_str());
      break;
    }
    case '0':   //! Starts counting from 0
    {
      counter = 0;
      ROS_WARN("Counter set to 0");
      break;
    }
  }
}

/**
  @brief Main function of the image_save_by_topic_node
  @param argc [int] Number of input arguments
  @param argv [char**] The input arguments
  @return int : 0 for success
 **/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_save_by_topic_node");
  ros::NodeHandle n;
  
  //! Check the arguments. If 3 no resizing is requested.
  ROS_WARN("Image saver intializes... \n\tPress 's' to save image\n\t\
Press '0' to set the counter to 0");
  if(argc != 3 && argc != 5)
  {
    ROS_ERROR("You haven't provided correct input. \nUsage:\n\trosrun \
pandora_vision_support image_saver_by_topic $image_topic $folder_for_imgs\
 [$width] [$height]\nExiting...");
    exit(0);
  }
  
  //! Argument parsing
  std::string topic(argv[1]);
  std::string folder(argv[2]);
  Params::folder = folder;
  
  //!< If arguments were 5 the resize operation was requested
  if(argc == 5)
  {
    Params::width = atoi(argv[3]);
    Params::height = atoi(argv[4]);
  }
  
  //! Topic subscription
  ROS_WARN("Subscribing to topic %s", topic.c_str());
  ros::Subscriber sub = n.subscribe(topic.c_str(), 1, imageCallback);
  
  ros::spin();
  return 0;
}
