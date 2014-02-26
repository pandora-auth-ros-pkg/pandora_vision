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
* Authors:  Tsakalis Vasilis, Despoina Paschalidou
*********************************************************************/
 
#ifndef HAZMATDETECTION_H
#define HAZMATDETECTION_H

#include "hazmat_detector.h"

namespace pandora_vision
{

  class HazmatDetection : public StateClient 
  {
    private:
      
      //nodeHandle
      ros::NodeHandle nh_;
      HazmatEpsilonDetector* hazmatDetector_;
      float ratioX_;
      float ratioY_;
      
      float hfov_;  //horizontal Field Of View (rad)
      float vfov_;
      int frameWidth_; //frame width
      int frameHeight_;  //frame height
      
      cv::Mat hazmatFrame_;  // frame processed by HazmatDetector
      
      ros::Time hazmatFrameTimestamp_; // HazmatDetector frame timestamp
      
      std::string packagePath_;
      std::string saveImagePath_;
      std::string imageTopic_;
    std::string cameraName;
    std::string cameraFrameId;
      
      int hazmatNumber_;
      
      //publisher
      ros::Publisher hazmatPublisher_;

      image_transport::Subscriber sub_;
      
      // variables for changing in dummy msg mode for debugging
      bool hazmatDummy_;
      
      //variable used for State Managing
      bool hazmatNowOn_;
      
    public:
          
      //constructor
      HazmatDetection();
            
      //destructor
      ~HazmatDetection();	
      
      //get parameters from launch file
      void getGeneralParams();
      void getHazmatParams();
      
      //timer callbacks
      void hazmatCallback();
      
      //get a new image
      void imageCallback(const sensor_msgs::ImageConstPtr& msg);
      
      void spin();
      
      //Implemented from StateClient
      void startTransition(int newState);
      void completeTransition(void);
      
      int curState; //Current state of robot
      int prevState;  //Previous state of robot
  };

}
#endif
    
    
