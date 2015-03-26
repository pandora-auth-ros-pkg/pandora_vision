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

#include "pandora_vision_hazmat/filters/histogram_mask.h"

int HistogramMask::scale_ = 10;
int HistogramMask::channels_[2] = { 0 , 1 };
int HistogramMask::thresh_ = 100;
int HistogramMask::maxValue_ = 255;
float HistogramMask::hueRange_[2] = { 0 , 180};
float HistogramMask::satRange_[2] = { 0 , 255};
float* HistogramMask::ranges_[2]  = {  hueRange_ , satRange_ };


// YUV histogram back projection filter constructor
HistogramMask::HistogramMask() 
{
  // TO DO : READ PARAMETERS FROM XML FILE.

}


void HistogramMask::createBackProjectionMask(const cv::Mat &frame , 
      cv::Mat *mask , const cv::Mat hist )
{
  cv::Mat hsvFrame;
  cv::Mat backprojection;
  
  if ( !frame.data )
  {
    ROS_INFO( "Invalid Frame received! \n");
    return;
  }
  
  // Convert the incoming BGR frame to HSV color space
  // to perform the back projection.
  cv::cvtColor( frame , hsvFrame , CV_BGR2HSV );
  
  // Lower the resolution of the frame to speed up processing
  // and create a more dense back projection.
  cv::resize(hsvFrame, hsvFrame, cv::Size( frame.size().width / scale_ ,
     frame.size().height / scale_ ) );
    
  // Calculate the backProjection on the frame.
  cv::calcBackProject( &hsvFrame, 1, channels_ , hist , backprojection , 
    const_cast<const float **> (ranges_) );
  
  
  
  // Resize the resulting mask to the size of the frame so that it 
  // can be used as a mask on the frame.
  cv::resize( backprojection , backprojection , frame.size() );
  
  double t = threshold( backprojection , backprojection , thresh_ , 
    maxValue_ , cv::THRESH_BINARY);
  #ifdef DEBUG
  cv::Mat segmentedFrame;
  frame.copyTo(segmentedFrame , backprojection);
  cv::imshow("Segmented Frame" , segmentedFrame);
  #endif

  // If the mask is not empty :
  // Perform bitwise or between the backprojection and the mask
  if ( mask->data )
    cv::bitwise_or(*mask , backprojection , *mask );
  else
  {
    // Assign the resulting backprojection as the mask.
    *mask = backprojection;
  }
  
  return;
}


