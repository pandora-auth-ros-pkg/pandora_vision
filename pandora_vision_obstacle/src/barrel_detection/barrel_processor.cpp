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
#include <vector>
#include <limits>
#include <utility>
#include "pandora_vision_obstacle/barrel_detection/barrel_processor.h"

namespace pandora_vision
{
namespace pandora_vision_obstacle
{
  BarrelProcessor::BarrelProcessor() :
    sensor_processor::Processor<ImagesStamped, POIsStamped>()
  {}
 
  /**
    @brief Find symmetric object inside frame
    @description Use fast symmetry detector algorith to find symmetric objects
    based on edges extracted from Canny edge detector  
    @param[in] inputImage [const cv::Mat&] Input depth image where we do the 
    processing
    @param[in out] roi [cv::Rect*] Here the candidate roi is stored
    @param[in] symmetricStartPoint [cv::Point*] The symmetry's line start point
    @param[in] symmetricEndPoint [cv::Point*] The symmetry's line end point
    @return void
   **/
  void BarrelProcessor::getSymmetryObject(
      const cv::Mat& inputImage,
      cv::Rect* roi,
      cv::Point* symmetricStartPoint,
      cv::Point* symmetricEndPoint)
  {
    cv::namedWindow("");
    cv::moveWindow("", 0, 0);
    cv::Point accumIndex(-1, -1);

    if (!inputImage.data)
      return;

    /* Determine the shape of Hough accumulationmatrix */
    float rhoDivs = hypotf(inputImage.rows, inputImage.cols) + 1;
    float thetaDivs = 180.0;

    FastSymmetryDetector detector(inputImage.size(), cv::Size(rhoDivs, thetaDivs), 1);

    cv::Rect region(0, inputImage.rows, thetaDivs * 2.0, rhoDivs * 0.5);
    // setMouseCallback( "", onMouse, static_cast<void*>( &region ) );
    cv::Mat temp, edge, depth8UC3;

    /* Adjustable parameters, depending on the scene condition */
    // int canny_thresh_1 = 30;
    // int canny_thresh_2 = 90;
    // int min_pair_dist  = 25;
    // int max_pair_dist  = 500;
    // int no_of_peaks    = 1;


    int cannyThresh1 = BarrelDetection::fsd_canny_thresh_1;
    int cannyThresh2 = BarrelDetection::fsd_canny_thresh_2;
    int minPairDist  = BarrelDetection::fsd_min_pair_dist;
    int maxPairDist  = BarrelDetection::fsd_max_pair_dist;
    int noOfPeaks    = BarrelDetection::fsd_no_of_peaks;

    temp = inputImage.clone();

    double minVal, maxVal;
    cv::minMaxLoc(temp, &minVal, &maxVal); 
    temp.convertTo(edge, CV_8U, 255.0 / (maxVal - minVal), -minVal * 255.0 / (maxVal - minVal));

    temp.convertTo(depth8UC3, CV_8UC3, 255);
    /* Find the edges */
    if (edge.channels() == 3)
      cvtColor(edge, edge, CV_BGR2GRAY);
    cv::Canny(edge, edge, cannyThresh1, cannyThresh2);

    /* Vote for the accumulation matrix */
    detector.vote(edge, minPairDist, maxPairDist);

    /* Draw the symmetrical line */
    std::vector<std::pair<cv::Point, cv::Point> > result = detector.getResult(noOfPeaks);
    float maxDist;
    float maxY;
    float minY;
    detector.getMaxDistance(&maxDist);
    detector.getYCoords(&maxY, &minY);

    for (int i = 0; i < result.size(); i ++)
    {
      //float len1 = std::sqrt(result[i].first.x * result[i].first.x + result[i].first.y * result[i].first.y);
      //float len2 = std::sqrt(result[i].second.x * result[i].second.x + result[i].second.y * result[i].second.y);

      //float dot = result[i].first.x * result[i].second.x + result[i].first.y * result[i].second.y;

      //float a = dot / (len1 * len2);

      //float angle;
      //if (a >= 1.0)
      //  angle = 0.0;
      //else if (a <= -1.0)
      //  angle = 3.14;
      //else
      //  angle = std::acos(a); // 0..PI
      //angle = angle * 180 / 3.14;
      cv::Point minPoint, maxPoint;
      minPoint.x = std::min(result[i].first.x, result[i].second.x);
      minPoint.y = std::min(result[i].first.y, result[i].second.y);
      maxPoint.x = std::max(result[i].first.x, result[i].second.x);
      maxPoint.y = std::max(result[i].first.y, result[i].second.y);
      float startROIX = minPoint.x - maxDist / 2;
      float startROIY = minPoint.y;
      float widthROI = maxDist;
      float heightROI = maxPoint.y - minPoint.y;
      if (startROIX < 0)
        startROIX = 0;
      if (startROIY < 0)
        startROIY = 0;
      if (startROIX + widthROI > inputImage.cols)
        widthROI = inputImage.cols - startROIX;
      if (startROIY + heightROI > inputImage.rows)
        heightROI = inputImage.rows - startROIY;
      (*roi) = cv::Rect(
          startROIX,
          startROIY,
          widthROI,
          heightROI);
      (*symmetricStartPoint) = result[i].second;
      (*symmetricEndPoint) = result[i].first;


      if (BarrelDetection::show_respective_barrel)
      {
        cv::line(depth8UC3, result[i].first, result[i].second, cv::Scalar(0, 0, 255), 2);
        cv::rectangle(depth8UC3,
            cv::Point(result[i].second.x - maxDist / 2, result[i].second.y),
            cv::Point(result[i].first.x + maxDist / 2, result[i].first.y),
            cv::Scalar(255, 0, 0),
            2);

        /* Visualize the Hough accum matrix */
        cv::Mat accum = detector.getAccumulationMatrix();
        accum.convertTo(accum, CV_8UC3);
        cv::applyColorMap(accum, accum, cv::COLORMAP_JET);
        cv::resize(accum, accum, cv::Size(), 2.0, 0.5);

        /* Draw lines based on cursor position */
        if (accumIndex.x != -1 && accumIndex.y != -1)
        {
          std::pair<cv::Point, cv::Point> pointPair = detector.getLine(accumIndex.y, accumIndex.x);
          cv::line(depth8UC3, pointPair.first, pointPair.second, CV_RGB(0, 255, 0), 2);
        }

        /* Show the original and edge images */
        cv::imshow("edge", depth8UC3);
        cv::waitKey(10);
        // cv::Mat appended = cv::Mat::zeros( depth8UC3.rows + accum.rows, depth8UC3.cols * 2, CV_8UC3 );
        // depth8UC3.copyTo( cv::Mat(appended, cv::Rect(0, 0, depth8UC3.cols, depth8UC3.rows)) );
        // cv::cvtColor( edge, cv::Mat(appended, cv::Rect(depth8UC3.cols, 0, edge.cols, edge.rows)), CV_GRAY2BGR );
        // accum.copyTo( cv::Mat( appended, Rect(0, depth8UC3.rows, accum.cols, accum.rows) ) );
        // cv::imshow("Candidate Barrel", appended);
      }
    }
  }

  /**
    @brief Validates the ROI for barrel existence
    @description Keep homogeneous regions in rgb, with decreasing depth
    from left to the symmetry line and increasing depth from symmetry line to
    right, with almost identical variation between two parts and with almost stable 
    depth through the symmetry line. 
    @param[in] rgbImage [const cv::Mat&] The rgb image
    @param[in] depthImage [const cv::Mat&] The depth image
    @param[in] rectRoi [const cv::Rect&] The ROI to validate
    @param[in] symmetricStartPoint [const cv::Point&] The symmetry's line start point
    @param[in] symmetricEndPoint [const cv::Point&] The symmetry's line end point
    @return [bool] A flag indicating roi's validity
   **/
  bool BarrelProcessor::validateRoi(
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
    if (stddev.val[0] > BarrelDetection::roi_variance_thresh)
    {
      (*valid) = false;
      return;
    }
    cv::Point slope;
    int length = std::abs(rectRoi.width / 2);
    slope.x = symmetricEndPoint.x - symmetricStartPoint.x;
    slope.y = symmetricEndPoint.y - symmetricStartPoint.y;
    float magnitude = std::sqrt(slope.x * slope.x + slope.y * slope.y);
    slope.x /= magnitude;
    slope.y /= magnitude;
    // Rotate vector 90 degrees clockwisely
    float temp1 = slope.x;
    slope.x = -slope.y;
    slope.y = temp1;
    // A point on the symmetry line
    cv::Point s1 =
      cv::Point(rectRoi.x + rectRoi.width / 2,
          rectRoi.y + rectRoi.height / 2);
    cv::Point s2;
    s2.x = s1.x + slope.x * length;
    s2.y = s1.y + slope.y * length;

    cv::Point s3;
    s3.x = s1.x - slope.x * length;
    s3.y = s1.y - slope.y * length;

    // In order to calculate circularity place points of the perpendicular
    // line into a vector

    cv::LineIterator it(depthImage, s3, s2, 8);
    std::vector<cv::Vec3b> buf(it.count);
    std::vector<cv::Point> points(it.count);
    std::vector<float> differentialPoints(it.count);
    float maxDifferential = std::numeric_limits<float>::min();

    // Ignore values at the border
    for (int linePoint = 0; linePoint < 10; linePoint ++, ++it)
    {
      buf[linePoint] = (const cv::Vec3b)* it;
      points[linePoint] = it.pos();
      differentialPoints[linePoint] = 0.0;
    }

    for (int linePoint = 10; linePoint < it.count - 10; linePoint ++, ++it)
    {
      buf[linePoint] = (const cv::Vec3b)* it;
      points[linePoint] = it.pos();
      if (depthImage.at<float>(points[linePoint].y, points[linePoint].x) != 0.0)
        differentialPoints[linePoint] =
          depthImage.at<float>(points[linePoint].y, points[linePoint].x)
          - depthImage.at<float>(points[linePoint - 1].y, points[linePoint - 1].x);
      else
        differentialPoints[linePoint] = 0;
      if (std::abs(differentialPoints[linePoint]) > maxDifferential)
        maxDifferential = std::abs(differentialPoints[linePoint]);
    }

    for (int linePoint = it.count - 10; linePoint < it.count; linePoint ++, ++it)
    {
      buf[linePoint] = (const cv::Vec3b)* it;
      points[linePoint] = it.pos();
      differentialPoints[linePoint] = 0.0;
    }

    // calculate differentiation of depth values, through the perpendicular
    // line(s) for the left side of the barrel and the right side of the
    // barrel. Calculate avg of differentiation for left and right side
    // separately.
    int leftRightBorder = static_cast<int>(points.size() / 2);
    float sumLeftLine = 0.0;

    for (int leftLinePoint = 0; leftLinePoint <= leftRightBorder; leftLinePoint ++)
    {
      sumLeftLine += differentialPoints[leftLinePoint];
    }

    float avgLeftLinePoint = 0.0;
    //if (leftRightBorder + 1 > 0)
    //  avgLeftLinePoint = sumLeftLine / (leftRightBorder + 1);
    //else
    //  avgLeftLinePoint = 0.0;
    avgLeftLinePoint = sumLeftLine;


    float sumRightLine = 0.0;
    for (int rightLinePoint = leftRightBorder + 1; rightLinePoint < points.size(); rightLinePoint ++)
    {
      sumRightLine += differentialPoints[rightLinePoint];
    }

    float avgRightLinePoint = 0.0;
    //if (points.size() - leftRightBorder > 0)
    //  avgRightLinePoint = sumRightLine / (points.size() - leftRightBorder);
    //else
    //  avgRightLinePoint = 0.0;
    avgRightLinePoint = sumRightLine;

    // We expect that starting from the left side of the barrel,
    // we have a decreasing depth up to the peak of the barrel
    // (negative differential avg), and then increasing depth.
    if (avgLeftLinePoint >= 0 || avgRightLinePoint <= 0)
    {
      (*valid) = false;
      return;
    }

    // We must have a symmetry between the differential avgs of the
    // left and right sides of the barrel.
    if (std::abs(avgLeftLinePoint + avgRightLinePoint) > BarrelDetection::differential_depth_unsymmetry_thresh)
    {
      (*valid) = false;
      return;
    }
  }

  float BarrelProcessor::findDepthDistance(const cv::Mat& depthImage,
      const cv::Rect& roi)
  {
    float depth = 0.0f;

    for (size_t row = 1; row < roi.height; row ++)
    {
    for (size_t col = 1; col < roi.width; col ++)
    {
      /// Find depth distance
      depth += depthImage.at<float>(row, col);
    }
    }
    depth /= static_cast<float>(roi.area());
    return depth;
  }

  bool BarrelProcessor::process(const ImagesStampedConstPtr& input,
      const POIsStampedPtr& output)
  {
    output->header = input->getHeader();
    output->frameWidth = input->getRgbImage().cols;
    output->frameHeight = input->getRgbImage().rows;

    // output->pois = ...(input->getImage());
    cv::Rect roi;
    getSymmetryObject(input->getDepthImage(), &roi);
    bool valid = true;
    validateRoi(input->getRgbImage(), input->getDepthImage(), roi, &valid);
    cv::Rect roi;
    cv::Point symmetricStartPoint;
    cv::Point symmetricEndPoint;
    getSymmetryObject(input->getDepthImage(), &roi, &symmetricStartPoint, &symmetricEndPoint);
    bool valid;
    // Find the depth distance of the soft obstacle
    float depthDistance;
    if (roi.area() > 0.1)
    {
      bool valid = true;
      validateRoi(input->getRgbImage(), input->getDepthImage(),
          roi, &valid, symmetricStartPoint, symmetricEndPoint);

      ObstaclePOIPtr poi(new ObstaclePOI);
      std::vector<POIPtr> pois;
      if (valid)
      {
        depthDistance = findDepthDistance(input->getDepthImage(), roi);
        ObstaclePOIPtr poi(new ObstaclePOI);
        poi->setPoint(cv::Point((roi.x + roi.width / 2),
              (roi.y + roi.height / 2)));
        poi->setWidth(roi.width);
        poi->setHeight(roi.height);

        poi->setProbability(1.0);
        poi->setType(pandora_vision_msgs::ObstacleAlert::BARREL);

        poi->setDepth(depthDistance);
        output->pois.push_back(poi);
        ROS_INFO("barrel found");
      }

    if (output->pois.empty())
    {
      return false;
    }
    return true;
  }
}  // namespace pandora_vision_obstacle
}  // namespace pandora_vision
