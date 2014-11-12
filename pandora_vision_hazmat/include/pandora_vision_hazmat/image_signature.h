<<<<<<< HEAD
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


#ifndef PANDORA_VISION_HAZMAT_IMAGE_SIGNATURE_H
#define PANDORA_VISION_HAZMAT_IMAGE_SIGNATURE_H
=======
#ifndef IMAGE_SIGNATURE_H
#define IMAGE_SIGNATURE_H
>>>>>>> Added new Hazmat detector classes.

#include "pandora_vision_hazmat/hazmat_detector.h"

/**
 @class ImageSignature 
 @brief Class that implements the image signature saliency map.
**/

<<<<<<< HEAD
class ImageSignature
{
  public : 
    
    
=======
class ImageSignature : public HazmatDetector
{
  public : 
    
>>>>>>> Added new Hazmat detector classes.
    // Function that calculates the image signature.
    static void calculateSignature(const cv::Mat &image , 
      cv::Mat *imgSign);
    
    // Function the creates the mask that will be applied to the 
    // incoming frame based on the saliency map produced by the 
    // signature of the image.
<<<<<<< HEAD
    static void createSaliencyMapMask(const cv::Mat &frame , 
      cv::Mat *mask );
       
    // Return the array that containts the signs of an arbitrary
    // matrix.
    static void signFunction(const cv::Mat &array , cv::Mat *signs );
     
    // Constructor
    ImageSignature() {};
    
    virtual ~ImageSignature() {}; 
    
  private :

};

#endif  // PANDORA_VISION_HAZMAT_IMAGE_SIGNATURE_H_
=======
    void virtual createMask(const cv::Mat &frame , cv::Mat *mask , 
      const cv::Mat &data = cv::Mat() );
    
    // Return the array that containts the signs of an arbitrary
    // matrix.
    static void signFunction(const cv::Mat &array , cv::Mat *signs );
    
    // The class who is being decorated is called to detect the 
    // pattern we wish to find.
    
    //~ bool virtual detect(const cv::Mat &frame , 
      //~ float *x , float *y )
      //~ 
    //~ {
      //~ return detector_->detect(frame,x,y);
    //~ }
    
    // Function used for reading the training data from an xml file.
    // The necessary data has to be read by the object point by the
    // detector_ pointer.
    //~ void virtual readData( void ) 
    //~ {
      //~ return ;
      //~ }
    
    // Function used to get the best feature matches between a frame
    // and a number of patterns.
    // In this decorator class it has no need to be implemented
    // since it's purpose is to create the correct filter.
    //~ int virtual getBestMatches( const cv::Mat &frame ,
     //~ const cv::Mat &features , double *minDist , double *maxDist  ) 
     //~ {
       //~ return 0;
     //~ }
     
    // Constructor
    ImageSignature(HazmatDetector *baseDetector);
    
    virtual ~ImageSignature() 
    {
    } 
    
  private :
  
    // Pointer the base class that will be decorated.
    HazmatDetector *detector_ ;
  };

#endif
>>>>>>> Added new Hazmat detector classes.
