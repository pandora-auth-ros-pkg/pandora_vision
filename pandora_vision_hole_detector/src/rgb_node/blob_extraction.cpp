/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, P.A.N.D.O.R.A. Team.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*
*  are met:
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
#include "rgb_node/blob_extraction.h"

namespace pandora_vision
{
  /**
    @brief Class constructor
  */ 
  BlobDetector::BlobDetector()
  {
    ROS_INFO("[rgb_node] : Blob extractor instance created");
  }
    
  /**
    @brief Class destructor
  */
  BlobDetector::~BlobDetector()
  {
    ROS_INFO("[rgb_node] : Blob extractor instance destroyed");
  }
  
  /**
    @brief Detects blobs in an image
    @param[in] frame [const cv::Mat&] The input image
    @param[out] keyPointsOut [std::vector<cv::KeyPoint>*] The ouput
    @return void
   **/
  void BlobDetector::detectBlobs(const cv::Mat& frame,
      std::vector<cv::KeyPoint>* keyPointsOut)
  {
    cv::SimpleBlobDetector::Params params;

    params.minThreshold = RgbParameters::blob_min_threshold; 
    params.maxThreshold = RgbParameters::blob_max_threshold; 
    params.thresholdStep = RgbParameters::blob_threshold_step;

    params.minArea = RgbParameters::blob_min_area;
    params.maxArea = RgbParameters::blob_max_area;

    params.minConvexity = RgbParameters::blob_min_convexity; 
    params.maxConvexity = RgbParameters::blob_max_convexity;

    params.minInertiaRatio = RgbParameters::blob_min_inertia_ratio;

    params.maxCircularity = RgbParameters::blob_max_circularity;
    params.minCircularity = RgbParameters::blob_min_circularity; 
    params.filterByColor = RgbParameters::blob_filter_by_color;
    params.filterByCircularity = RgbParameters::blob_filter_by_circularity;

    cv::SimpleBlobDetector blobDetector(params);
    blobDetector.create("SimpleBlob");

    std::vector<cv::KeyPoint> keyPoints;

    //!< detect blobs. store their center point
    blobDetector.detect(frame, keyPoints);
    
    cv::Mat out; 
    std::vector< std::vector <cv::Point> > contours;
    std::vector< std::vector <cv::Point> > approxContours;
    cv::drawKeypoints( frame, keyPoints, out, CV_RGB(0,255,255), cv::DrawMatchesFlags::DEFAULT);
    approxContours.resize( contours.size() );
    for( int i = 0; i < contours.size(); ++i )
    {
      cv::approxPolyDP( cv::Mat(contours[i]), approxContours[i], 4, 1 );
      cv::drawContours( out, contours, i, CV_RGB(255,0,255));
      cv::drawContours( out, approxContours, i, CV_RGB(255,0,255));
    }
  
    for (int keypointId = 0; keypointId < keyPoints.size(); keypointId++)
    {
      //!< if the keypoint is out of image limits, discard it
      if (keyPoints[keypointId].pt.x < frame.cols &&
          keyPoints[keypointId].pt.x >= 0 &&
          keyPoints[keypointId].pt.y < frame.rows &&
          keyPoints[keypointId].pt.y >= 0)
      {
        keyPointsOut->push_back(keyPoints[keypointId]);
      }
    }
  }

}
