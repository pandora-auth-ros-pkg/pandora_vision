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
* Author: Despoina Paschalidou
*********************************************************************/

#include "pandora_vision_victim/victim_detector.h"

namespace pandora_vision
{
  /**
   @brief Constructor
  */ 
  VictimDetector::VictimDetector(std::string cascade_path, 
    std::string model_path, int bufferSize, std::string rgb_classifier_path,
        std::string depth_classifier_path)
  {
    /// Initialize face detector
    _faceDetector = new FaceDetector(cascade_path, model_path, bufferSize);
    
    _rgb_classifier_path = rgb_classifier_path;
    _depth_classifier_path = depth_classifier_path;
    ROS_DEBUG("[victim_node] : VictimDetector instance created");
  }
  
  /**
    @brief Destructor
  */
  VictimDetector::~VictimDetector()
  {
    delete _faceDetector;
    ROS_DEBUG("[victim_node] : VictimDetector RgbSystemValidator instance");
  }
  
  /**
   *@brief Function that enables suitable subsystems, according
   * to the current State 
   * @param [int] _stateIndicator
   * @param [std::vector<cv::Mat>] vector of images to be processed. Size of
   * vector can be either 2 or 1, if we have both rgbd information or not
   * @return void
  */ 
  void VictimDetector::victimFusion( 
      int _stateIndicator, std::vector<cv::Mat> _rgbdImages)
  {
      if(_rgbdImages.size() == 1)
        _rgbImage = _rgbdImages[0];
      else{
         _rgbImage = _rgbdImages[0];
         _depthImage = _rgbdImages[1];
      }  
        
    switch(_stateIndicator){
      case 1:
         ///Enable Viola Jones for rgb image
        _faceDetector->findFaces(_rgbImage);
        ///Enable Viola Jones for depth image
        _faceDetector->findFaces(_depthImage);
        ///Enable rgb_system validator for rgb image
        rgbFeaturesDetect(_rgbImage);
        ///Enable rgb_system validator for depth image
        depthFeaturesDetect(_depthImage);
        break;  
      
      case 2:
        ///Enable Viola Jones for rgb image
        _faceDetector->findFaces(_rgbImage);
        ///Enable rgb_system validator for rgb image
         rgbFeaturesDetect(_rgbImage);
        break;
      
      case 3:
        ///Enable Viola Jones for rgb image
        _faceDetector->findFaces(_rgbImage);
        ///Enable Viola Jones for depth image
        _faceDetector->findFaces(_depthImage);
        break;
        
      case 4:
        ///Enable Viola Jones for rgb image
        _faceDetector->findFaces(_rgbImage);
        break;
      
      default:
        ROS_ERROR("[victim_node] : Invalid state for victim_node");
        break;
    }
  }
    
  /**
   *@brief Function that extracts handles rgb subsystem
   *@param [cv::Mat] current frame to be processed
   *@return void
  */ 
  void VictimDetector::rgbFeaturesDetect(cv::Mat _rgbImage)
  {
    _rgbSystemValidator.extractRgbFeatures(_rgbImage); 
  }
  
  /**
   *@brief Function that extracts handles depth subsystem
   *@param [cv::Mat] current frame to be processed
   *@return void
  */ 
  void VictimDetector::depthFeaturesDetect(cv::Mat _depthImage)
  {
    _depthSystemValidator.extractDepthFeatures(_depthImage); 
  }
  
}// namespace pandora_vision

