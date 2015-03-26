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

#ifndef PANDORA_VISION_HAZMAT_TRAINING_PLANAR_PATTERN_TRAINER
#define PANDORA_VISION_HAZMAT_TRAINING_PLANAR_PATTERN_TRAINER

#include "pandora_vision_hazmat/training/utilities.h"


/** 
 @class PlanarPatternTrainer
 @brief Abstract class used for training the Hazmat detector
 **/
 
class PlanarPatternTrainer{

  public:
/*
     * @brief: Function used to produce the necessary keypoints and their
     *          corresponding descriptors for an image. 
     * @param descriptors[cv::Mat*]: A pointer to the array that will be used to
     * store the descriptors of the current image.
     * @param keyPoints[std::vector<cv::KeyPoint>*] : A pointer to the vector
     * containing the Keypoints detected in the current image.
     * @param boundingBox[std::vector<cv::Point2f>*] : A pointer to the vector
     * containing the bounding box for the pattern in the current image.
     **/
    virtual void getFeatures(const cv::Mat& frame,
        cv::Mat *descriptors, 
      std::vector<cv::KeyPoint>* keyPoints,
      std::vector<cv::Point2f>* boundingBox) = 0;
 /*
     * @brief: Function used to produce the necessary keypoints and their
     *          corresponding descriptors for an image. 
     * @param descriptors[cv::Mat*]: A pointer to the array that will be used to
     * store the descriptors of the current image.
     * @param keyPoints[std::vector<cv::KeyPoint>*] : A pointer to the vector
     * containing the Keypoints detected in the current image.
     **/
    virtual void getFeatures(const cv::Mat& frame,
        cv::Mat *descriptors,
        std::vector<cv::KeyPoint>* keyPoints) = 0;
    // The function that will calculate the histogram of the pattern.
    virtual void calculateHistogram(cv::Mat *hist);
    
    virtual const std::string getFeatureType() = 0 ;
    
    /**
    @brief Sets the current image that will be used to train the detector.
    @param img [const cv::Mat] : The image of the pattern that we want 
                                to detect .
    **/            
    virtual void setCurrentImage(const cv::Mat &img)
    {
      currentImage_ = img ;
    }

        
    /**
     @brief Main training function that reads input images and stores 
            the results in a corresponding xml file.
    **/ 
    void train();
    
  /**
   @brief Saves the training data to a proper XML file.
   @param patternName [const std::string &] : The name of the pattern.
   @param descriptors [const cv::Mat &] : The descriptors of the pattern.
   @param keyPoints [const std::vector<cv::Keypoint>] : The key points
          detected on the pattern.
  **/                  
   void saveData(const std::string &patternName ,
      const cv::Mat &descriptors ,
      const std::vector<cv::KeyPoint> &keyPoints , 
      const std::vector<cv::Point2f> &boundingBox ,
      const cv::Mat &histogram );

  protected:
    cv::Mat currentImage_ ; //!< Current image used for traing.
  };


#endif  // PANDORA_VISION_HAZMAT_TRAINING_PLANAR_PATTERN_TRAINER
