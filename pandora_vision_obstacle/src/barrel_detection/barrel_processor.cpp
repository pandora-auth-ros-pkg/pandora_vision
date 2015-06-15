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
 * Authors:
 *   Chatzieleftheriou Eirini <eirini.ch0@gmail.com>
 *********************************************************************/

#include <string>
#include "pandora_vision_obstacle/barrel_detection/barrel_processor.h"

namespace pandora_vision
{
namespace pandora_vision_obstacle
{
  BarrelProcessor::BarrelProcessor() :
    sensor_processor::Processor<ImagesStamped, POIsStamped>()
  {}

  void BarrelProcessor::getSymmetryObject(const cv::Mat& inputImage, cv::Rect* roi)
  {
    namedWindow("");
    moveWindow("", 0, 0);
    cv::Point accumIndex(-1, -1);

    if(!inputImage.data)
      return;

    /* Determine the shape of Hough accumulationmatrix */
    float rhoDivs   = hypotf( inputImage.rows, inputImage.cols ) + 1;
    float thetaDivs = 180.0;

    FastSymmetryDetector detector( inputImage.size(), cv::Size(rhoDivs, thetaDivs), 1 );


    cv::Rect region( 0, inputImage.rows, thetaDivs * 2.0, rhoDivs * 0.5 );
    //setMouseCallback( "", onMouse, static_cast<void*>( &region ) );
    cv::Mat temp, edge;

    /* Adjustable parameters, depending on the scene condition */                                                                                                 
    //int canny_thresh_1 = 30;
    //int canny_thresh_2 = 90;
    //int min_pair_dist  = 25;
    //int max_pair_dist  = 500;
    //int no_of_peaks    = 1;


    int canny_thresh_1 = BarrelDetection::fsd_canny_thresh_1;
    int canny_thresh_2 = BarrelDetection::fsd_canny_thresh_2;
    int min_pair_dist  = BarrelDetection::fsd_min_pair_dist;
    int max_pair_dist  = BarrelDetection::fsd_max_pair_dist;
    int no_of_peaks    = BarrelDetection::fsd_no_of_peaks;

    temp = inputImage.clone();

    /* Find the edges */
    if(temp.channels() == 3)
      cvtColor( temp, edge, CV_BGR2GRAY );
    cv::Canny( edge, edge, canny_thresh_1, canny_thresh_2 );

    /* Vote for the accumulation matrix */
    detector.vote( edge, min_pair_dist, max_pair_dist );

    /* Draw the symmetrical line */
    std::vector<std::pair<cv::Point, cv::Point> > result = detector.getResult( no_of_peaks );
    float maxDist;
    float maxY;
    float minY;
    detector.getMaxDist(&maxDist);                                                                                                                              
    detector.getYCoords(&maxY, &minY);

    for( int i = 0; i < result.size(); i ++ )
    {
      (*roi) = cv::Rect(
          result[i].second.x - maxDist / 2, 
          result[i].second.y, 
          result[i].first.x - result[i].second.x + maxDist, 
          result[i].first.y - result[i].second.y);
      if(BarrelDetection::show_respective_barrel)
      {
        cv::line(temp, result[i].first, result[i].second, cv::Scalar(0, 0, 255), 2);
        cv::rectangle(temp, 
            cv::Point(result[i].second.x - maxDist / 2, result[i].second.y), 
            cv::Point(result[i].first.x + maxDist / 2, result[i].first.y), 
            cv::Scalar(255, 0, 0), 
            2);
        cv::Point slope;
        int length = std::abs((*roi).width / 2);
        slope.x = result[i].second.x - result[i].first.x;
        slope.y = result[i].second.y - result[i].first.y;
        float magnitude = std::sqrt(slope.x * slope.x + slope.y * slope.y);
        slope.x /= magnitude;
        slope.y /= magnitude;
        // Rotate vector 90 degrees clockwisely 
        float temp1 = slope.x;
        slope.x = -slope.y;
        slope.y = temp1;
        // A point on the symmetry line
        cv::Point s1 = 
          cv::Point((*roi).x + (*roi).width / 2, 
              (*roi).y + (*roi).height / 2);
        cv::Point s2;
        s2.x = s1.x + slope.x * length;
        s2.y = s1.y + slope.y * length;

        cv::Point s2;
        s3.x = s1.x - slope.x * length;
        s3.y = s1.y - slope.y * length;
        cv::line(temp, s3, s2, cv::Scalar(0, 255, 0), 2);

        /* Visualize the Hough accum matrix */
        cv::Mat accum = detector.getAccumulationMatrix();
        accum.convertTo( accum, CV_8UC3 );
        cv::applyColorMap( accum, accum, COLORMAP_JET );
        cv::resize( accum, accum, cv::Size(), 2.0, 0.5 );

        /* Draw lines based on cursor position */
        if(accumIndex.x != -1 && accumIndex.y != -1 ) {
          std::pair<cv::Point, cv::Point> point_pair = detector.getLine( accumIndex.y, accumIndex.x );                                                                           
          cv::line( temp, point_pair.first, point_pair.second, CV_RGB(0, 255, 0), 2 );
        }

        /* Show the original and edge images */
        cv::Mat appended = cv::Mat::zeros( temp.rows + accum.rows, temp.cols * 2, CV_8UC3 );
        temp.copyTo( cv::Mat(appended, cv::Rect(0, 0, temp.cols, temp.rows)) );
        cv::cvtColor( edge, cv::Mat(appended, cv::Rect(temp.cols, 0, edge.cols, edge.rows)), CV_GRAY2BGR );
        accum.copyTo( cv::Mat( appended, Rect(0, temp.rows, accum.cols, accum.rows) ) );
        cv::imshow("Candidate Barrel", appended);
      }
    }
  }

  void BarrelProcessor::validateRoi(
      const cv::Mat& rgbImage, 
      const cv::Mat& depthImage, 
      const cv::Rect& rectRoi,
      bool* valid)
  {
    // Validate based on variance in RGB ROI
    cv::Scalar mean;
    cv::Scalar stddev;
    cv::Mat roiRgb = rgbImage(rectRoi);
    cv::Mat roiDepth = depthImage(rectRoi);
    cv::meanStdDev ( roiRgb, mean, stddev );
    if(stddev.val[0] > BarrelDetection::roi_variance_thresh)
    {
      (*valid) = false;
      return;
    }
  }

  bool BarrelProcessor::process(const ImagesStampedConstPtr& input,
      const POIsStampedPtr& output)
  {
    output->header = input->getHeader();
    output->frameWidth = input->getRgbImage().cols;

    // output->pois = ...(input->getImage());
    cv::Rect roi;
    getSymmetryObject(input->getDepthImage(), &roi);
    bool valid = true;
    validateRoi(input->getRgbImage(), input->getDepthImage(), roi, &valid);

    if (output->pois.empty())
    {
      return false;
    }
    return true;
  }
}  // namespace pandora_vision_obstacle
}  // namespace pandora_vision
