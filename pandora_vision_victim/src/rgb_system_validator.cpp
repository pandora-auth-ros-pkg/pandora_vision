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

#include "pandora_vision_victim/rgb_system_validator.h"

namespace pandora_vision
{
  /**
   @brief Constructor
  */ 
  RgbSystemValidator::RgbSystemValidator(std::string rgb_classifier_path)
  {
    _rgb_classifier_path = rgb_classifier_path;
        
    _params.svm_type    = CvSVM::C_SVC;
    _params.kernel_type = CvSVM::LINEAR;
    _params.term_crit   = cvTermCriteria(CV_TERMCRIT_ITER, 100, 1e-6);

    ROS_DEBUG("[victim_node] : RgbSystemValidator instance created");
  }
  
  /**
    @brief Destructor
  */
  RgbSystemValidator::~RgbSystemValidator()
  {
    ROS_DEBUG("[victim_node] : Destroying RgbSystemValidator instance");
  }
  
  /**
   * @brief This function extract features according to the
   * predifined features for the rgb image
   * @param inImage [cv::Mat] current rgb frame to be processed
   * @return void
  */ 
  void RgbSystemValidator::extractRgbFeatures(cv::Mat inImage)
  {
    ///Extract color and statistics oriented features
    ///for rgb image
    _channelsStatisticsDetector.findChannelsStatisticsFeatures(inImage);
    
    ///Extract edge orientation features for rgb image
    _edgeOrientationDetector.findEdgeFeatures(inImage);
     
    ///Extract haralick features for rgb image 
    _haralickFeatureDetector.findHaralickFeatures(inImage);
    
    
    if(!_rgbFeatureVector.empty())
      _rgbFeatureVector.clear();
    
    setRgbFeatureVector();
  }
  
  /**
    * @brief This function creates feature vector according to the
    * predifined features for the rgb image
    * @return void
  */ 
  void RgbSystemValidator::setRgbFeatureVector()
  {
    ///Append to rgbFeatureVector features according to color
    ///histogramms and other statistics
    std::vector<double> channelsStatictisFeatureVector = 
        _channelsStatisticsDetector.getRgbFeatures();
    for(int i = 0; i < channelsStatictisFeatureVector.size(); i++ )
          _rgbFeatureVector.push_back(channelsStatictisFeatureVector[i]);
    
    ///Append to rgbFeatureVector features according to edge orientation
    std::vector<double> edgeOrientationFeatureVector = 
        _edgeOrientationDetector.getFeatures();
    for(int i = 0; i < edgeOrientationFeatureVector.size(); i++ )
          _rgbFeatureVector.push_back(edgeOrientationFeatureVector[i]);   
    
    ///Append to rgbFeatureVector features according to haaralick features
    std::vector<double> haaralickFeatureVector = 
        _haralickFeatureDetector.getFeatures();
    for(int i = 0; i < haaralickFeatureVector.size(); i++ )
          _rgbFeatureVector.push_back(haaralickFeatureVector[i]);  
          
    ///Deallocate memory
    channelsStatictisFeatureVector.clear();
    _channelsStatisticsDetector.emptyCurrentFrameFeatureVector();
    
    edgeOrientationFeatureVector.clear();
    _edgeOrientationDetector.emptyCurrentFrameFeatureVector(); 
    
    haaralickFeatureVector.clear();
    _haralickFeatureDetector.emptyCurrentFrameFeatureVector();
  }
  
  /**
   * @brief This function returns current feature vector according
   * to the features found in rgb image
   * @return [std::vector<double>] _rgbFeatureVector, feature vector 
   * for current rgb image
   */ 
  std::vector<double> RgbSystemValidator::getRgbFeatureVector()
  {
    return _rgbFeatureVector;
  }
}// namespace pandora_vision 
