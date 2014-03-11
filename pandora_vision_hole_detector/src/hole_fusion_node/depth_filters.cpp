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
 * Authors: Alexandros Filotheou, Manos Tsardoulias
 *********************************************************************/

#include "hole_fusion_node/depth_filters.h"

namespace pandora_vision
{
  /**
    @brief Checks for valid holes just by depth difference between the center
    of the blob and the edges of the bounding box
    @param[in] depthImage [const cv::Mat&] The depth image
    @param[in] inKeyPoints [const std::vector<cv::KeyPoint>&] The keypoints
    @param[in] inRectangles [const std::vector<std::vector<cv::Point2f> >&] The
    bounding boxes' vertices
    @param[in][out] msgs [std::vector<std::string>*] Messages for debug reasons
    @param[in] inflationSize [const in&t] The number of pixels by which the
    bounding rectange will be inflated
    @return std::set<unsigned int> The indices of valid (by this filter) blobs
   **/
  std::set<unsigned int> DepthFilters::checkHolesDepthDiff(
    const cv::Mat& depthImage,
    const std::vector<cv::KeyPoint>& inKeyPoints,
    const std::vector<std::vector<cv::Point2f> >& inRectangles,
    std::vector<std::string>* msgs,
    const int& inflationSize)
  {
    #ifdef DEBUG_TIME
    Timer::start("checkHolesDepthDiff","applyFilter");
    #endif

    std::set<unsigned int> valid;

    std::vector<std::vector<cv::Point> > inflatedRectangles;

    //!< since not all rectangles may not make it, store the indices
    //!< of the original keypoints that correspond to valid inflated rectangles
    std::vector<unsigned int> validKeyPointsIndices;
    std::vector<cv::KeyPoint> validKeyPoints;
    float key_y;
    float key_x;
    float vert_y;
    float vert_x;
    double theta;
    double keypointVertDist;

    for (unsigned int i = 0; i < inRectangles.size(); i++)
    {
      std::vector<cv::Point> inflatedVertices;
      int inflatedVerticesWithinImageLimits = 0;

      for (int j = 0; j < 4; j++)
      {
        key_y = inKeyPoints[i].pt.y;
        key_x = inKeyPoints[i].pt.x;

        vert_y = inRectangles[i][j].y;
        vert_x = inRectangles[i][j].x;

        theta = atan2(key_y - vert_y, key_x - vert_x);

        keypointVertDist = sqrt(pow(key_x -vert_x, 2) + pow(key_y -vert_x, 2));

        // check if the inflated vertex has gone out of bounds
        if (vert_x - inflationSize * cos(theta) < depthImage.cols &&
          vert_x - inflationSize * cos(theta) > 0 &&
          vert_y - inflationSize * sin(theta) < depthImage.rows &&
          vert_y - inflationSize * sin(theta) > 0)
        {
          inflatedVerticesWithinImageLimits++;
        }

        inflatedVertices.push_back(
          cv::Point(round(vert_x - inflationSize * cos(theta)),
            round(vert_y - inflationSize * sin(theta))));
      } //!< end for rectangle's points

      //!< if one or more vertices are out of bounds discard the whole
      //!< inflated rectangle
      if (inflatedVerticesWithinImageLimits < 4)
      {
        inflatedVertices.clear();
        continue;
      }
      else
      {
        validKeyPointsIndices.push_back(i);
        validKeyPoints.push_back(inKeyPoints[i]);
        inflatedRectangles.push_back(inflatedVertices);
      }
    } //!< end for rectangles


    for(unsigned int i = 0 ; i < validKeyPointsIndices.size() ; i++)
    {
      float mean = 0;
      for(unsigned int j = 0 ; j < 4; j++)
      {
        int x = inflatedRectangles[i][j].x;
        int y = inflatedRectangles[i][j].y;
        if(x < 0 || x >= depthImage.cols || y < 0 || y >= depthImage.rows)
        {
          mean = -40000.0;  //!< Invalid value
          break;
        }
        mean += depthImage.at<float>(y,x);
      }
      mean /= 4;

      float value = depthImage.at<float>(
        inKeyPoints[validKeyPointsIndices[i]].pt.y,
        inKeyPoints[validKeyPointsIndices[i]].pt.x) - mean;

      if(value > 0 && value < HoleFusionParameters::depth_difference)
      {
        valid.insert(validKeyPointsIndices[i]);
      }
      msgs->push_back(TOSTR(value));
    }

    #ifdef DEBUG_TIME
    Timer::tick("checkHolesDepthDiff");
    #endif

    return valid;
  }



  /**
    @brief Checks for valid holes by area / depth comparison
    @param[in] depthImage [const cv::Mat&] The depth image
    @param[in] inKeyPoints [const std::vector<cv::KeyPoint>&] The keypoints
    @param[in] inRectangles [const std::vector<std::vector<cv::Point2f> >&] The
    bounding boxes' vertices
    @param[in][out] msgs [std::vector<std::string>*] Messages for debug reasons
    @return std::set<unsigned int> The indices of valid (by this filter) blobs
   **/
  std::set<unsigned int> DepthFilters::checkHolesDepthArea(
    const cv::Mat& depthImage,
    const std::vector<cv::KeyPoint>& inKeyPoints,
    const std::vector<std::vector<cv::Point2f> >& inRectangles,
    std::vector<std::string>* msgs)
  {
    #ifdef DEBUG_TIME
    Timer::start("checkHolesDepthArea","applyFilter");
    #endif
    std::set<unsigned int> valid;
    for(unsigned int i = 0 ; i < inKeyPoints.size() ; i++)
    {
      float mean = 0;
      float area = sqrt(
        pow(inRectangles[i][0].x - inRectangles[i][1].x,2) +
        pow(inRectangles[i][0].y - inRectangles[i][1].y,2));

      area *= sqrt(
        pow(inRectangles[i][1].x - inRectangles[i][2].x,2) +
        pow(inRectangles[i][1].y - inRectangles[i][2].y,2));


      for(unsigned int j = 0 ; j < inRectangles[i].size() ; j++)
      {
        int x = inRectangles[i][j].x;
        int y = inRectangles[i][j].y;
        mean += depthImage.at<float>(y,x);
      }
      mean /= inRectangles[i].size();

      //!< Cubic fitting  --- Small hole
      //!< Normal : -18480  * x^3+87260.3 * x^2-136846 * x+74094
      float vlow = -18480.0 * pow(mean,3) + 87260.3 * pow(mean,2) -
        136846 * mean + 68500.0 - area;
      //!< Exponential fitting --- Small hole
      //!< Normal : -23279.4  * x^3+112218 * x^2-182162 * x+105500
      float vhigh = -23279.4 * pow(mean,3) + 112218.0 * pow(mean,2) -
        182162.0 * mean + 112500 - area;

      msgs->push_back(TOSTR(vlow)+std::string(",")+TOSTR(vhigh));

      if(vlow < 0 && vhigh > 0)
      {
        valid.insert(i);
      }
    }

    #ifdef DEBUG_TIME
    Timer::tick("checkHolesDepthArea");
    #endif

    return valid;
  }



  /**
    @brief Brushfire from a blobs's outline to its bounding box
    with an inflation size (inflates the rectangle by inflationSize pixels).
    If the points between the blob's outline and the inflated rectangle
    lie on one plane, this blob is a hole.
    @param[in] inImage [const cv::Mat&] The input depth image
    @param[in] initialPointCloud [const pcl::PointCloud<pcl::PointXYZ>::Ptr&]
    The original point cloud acquired from the depth sensor
    @param[in] keyPoints [const std::vector<cv::KeyPoint>&] The keypoints of
    blobs
    @param[in] inOutlines [const std::vector<std::vector<cv::point> >&&] The
    points the outline consists of
    @param[in] rectangles [const std::vector<std::vector<cv::point2f> >&&]
    The bounding boxes' vertices
    @param[in] inflationsize [const int&] Grow the rectangle by inflationsize as
    to acquire more points to check for plane existence.
    @return std::set<unsigned int> The indices of valid (by this filter)
    blobs
   **/
  std::set<unsigned int> DepthFilters::checkHolesBrushfireOutlineToRectangle(
    const cv::Mat& inImage,
    const PointCloudXYZPtr& initialPointCloud,
    const std::vector<cv::KeyPoint>& keyPoints,
    const std::vector<std::vector<cv::Point> >& inOutlines,
    const std::vector<std::vector<cv::Point2f> >& rectangles,
    const int& inflationSize)
  {
    #ifdef DEBUG_TIME
    Timer::start("checkHolesBrushfireOutlineToRectangle","applyFilter");
    #endif

    std::set<unsigned int> valid;

    std::vector<std::vector<cv::Point> > inflatedRectangles;
    float key_y;
    float key_x;
    float vert_y;
    float vert_x;
    double theta;
    double keypointVertDist;

    //!< for every valid inflated rectangle store a point
    //!< from which the brushfire will begin
    std::vector<cv::Point> brushfireBeginPoints;

    for (unsigned int i = 0; i < rectangles.size(); i++)
    {
      std::vector<cv::Point> inflatedVertices;
      int inflatedVerticesWithinImageLimits = 0;

      for (int j = 0; j < 4; j++)
      {
        key_y = keyPoints[i].pt.y;
        key_x = keyPoints[i].pt.x;

        vert_y = rectangles[i][j].y;
        vert_x = rectangles[i][j].x;

        theta = atan2(key_y - vert_y, key_x - vert_x);

        keypointVertDist = sqrt(pow(key_x -vert_x, 2) + pow(key_y -vert_x, 2));

        // check if the inflated vertex has gone out of bounds
        if (vert_x - inflationSize * cos(theta) < inImage.cols &&
          vert_x - inflationSize * cos(theta) > 0 &&
          vert_y - inflationSize * sin(theta) < inImage.rows &&
          vert_y - inflationSize * sin(theta) > 0)
        {
          inflatedVerticesWithinImageLimits++;
        }

        inflatedVertices.push_back(
          cv::Point(round(vert_x - inflationSize * cos(theta)),
            round(vert_y - inflationSize * sin(theta))));
      } // end for rectangle's points

      //!< if inflatedVerticesWithinImageLimits < 4 in the end,
      //!< we won't need this point since the inflated rectangle
      //!< will be discarded.
      cv::Point potentialBrushfireBeginPoint(
        round(vert_x - inflationSize / 2 * cos(theta)),
        round(vert_y - inflationSize / 2 * sin(theta)));

      //!< if one or more vertices are out of bounds
      //!< discard the whole inflated rectangle
      if (inflatedVerticesWithinImageLimits < 4)
      {
        inflatedVertices.clear();
        continue;
      }
      else
      {
        valid.insert(i);
        inflatedRectangles.push_back(inflatedVertices);
        brushfireBeginPoints.push_back(potentialBrushfireBeginPoint);
      }
    } //!< end for rectangles

    /**
     * For each inflated rectangle, store in visitedPoints
     * the visited points of brushfire.
     * (visitedPoints = points between the blob's outline
     * and the inflated rectangle)
     * We will test if these points all lie on one plane.
     **/
    //!< The indices of the valid (by this filter) keypoints
    std::set<unsigned int> finalIndices;
    std::set<unsigned int>::iterator validIterator = valid.begin();
    for (unsigned int i = 0; i < inflatedRectangles.size(); i++)
    {

      //!< The canvas image will hold the blobs' outlines, and their rectangles.
      cv::Mat canvas = cv::Mat::zeros(inImage.size(), CV_8UC1);
      cv::RNG rng(12345);
      cv::Scalar color = cv::Scalar(
        rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255));

      //!< Draw the outline of the i-th valid blob
      for(unsigned int j = 0; j < inOutlines[i].size(); j++)
      {
        canvas.at<uchar>(inOutlines[i][j].y, inOutlines[i][j].x) = 255;
      }

      //!< Draw the inflated rectangle that corresponds to it
      for(int j = 0; j < 4; j++)
      {
        cv::line(canvas, inflatedRectangles[i][j],
          inflatedRectangles[i][(j + 1) % 4], color, 1, 8);
      }

      std::set<unsigned int> visitedPoints;
      BlobDetection::brushfirePoint(brushfireBeginPoints[i],
        &canvas, &visitedPoints);

      /**
       * In order to construct a new point cloud
       * we need a priori knowledge of the valid points
       * of the input point cloud.
       **/
      int numValidVisitedPoints = 0;
      for(std::set<unsigned int>::iterator it = visitedPoints.begin();
        it != visitedPoints.end(); it++)
      {
        //!< if the point in the original point cloud is not missing (noise)
        if (!(initialPointCloud->points[*visitedPoints.find(*it)].x !=
            initialPointCloud->points[*visitedPoints.find(*it)].x))
        {
          numValidVisitedPoints++;
        }
      }

      PointCloudXYZPtr visitedPointsPointCloud (new PointCloudXYZ);

      visitedPointsPointCloud->width = numValidVisitedPoints;
      visitedPointsPointCloud->height = 1;
      visitedPointsPointCloud->points.resize
        (visitedPointsPointCloud->width * visitedPointsPointCloud->height);

      visitedPointsPointCloud->header.frame_id =
        initialPointCloud->header.frame_id;
      visitedPointsPointCloud->header.stamp =
        initialPointCloud->header.stamp;

      int pointCloudPointsIndex = 0;
      for(std::set<unsigned int>::iterator it = visitedPoints.begin();
        it != visitedPoints.end(); it++)
      {
        //!< if the point in the original point cloud is not missing (noise)
        if (!(initialPointCloud->points[*visitedPoints.find(*it)].x !=
            initialPointCloud->points[*visitedPoints.find(*it)].x))
        {
          visitedPointsPointCloud->points[pointCloudPointsIndex].x =
            initialPointCloud->points[*visitedPoints.find(*it)].x;

          visitedPointsPointCloud->points[pointCloudPointsIndex].y =
            initialPointCloud->points[*visitedPoints.find(*it)].y;

          visitedPointsPointCloud->points[pointCloudPointsIndex].z =
            initialPointCloud->points[*visitedPoints.find(*it)].z;

          pointCloudPointsIndex++;
        }
      }

      //!< Check if the visitedPointsPointCloud points are on a plane
      int numPlanes = PlanesDetection::locatePlanes(visitedPointsPointCloud,
        false);

      if (numPlanes == 1)
      {
        finalIndices.insert(*validIterator);
      }

      //!< Increment the validIterator so that it points to the next element
      //!< in the valid set
      validIterator++;
    }

    #ifdef DEBUG_TIME
    Timer::tick("checkHolesBrushfireOutlineToRectangle");
    #endif

    return finalIndices;
  }



  /**
    @brief  Given the bounding box of a blob, inflate it.
    All the points that lie on the (edges of the) rectangle should
    also lie on exactly one plane for the blob to be a hole.
    @param[in] inImage [const cv::Mat&] The input depth image
    @param[in] initialPointCloud [const pcl::PointCloud<pcl::PointXYZ>::Ptr&]
    The original point cloud,  uninterpolated, undistorted.
    @param[in] inKeyPoints [const std::vector<cv::KeyPoint>&] The keypoints of
    blobs
    @param[in] rectangles [const std::vector<std::vector<cv::Point2f> >&] The
    bounding boxes' vertices
    @param[in] inflationSize [cosnt int&] grow the rectangle by inflationSize
    as to acquire more points to check for plane existence.
    @return std::set<unsigned int> The indices of valid (by this filter)
    blobs
   **/
  std::set<unsigned int> DepthFilters::checkHolesRectangleOutline(
    const cv::Mat& inImage,
    const PointCloudXYZPtr& initialPointCloud,
    const std::vector<cv::KeyPoint>& inKeyPoints,
    const std::vector<std::vector<cv::Point2f> >& rectangles,
    const int& inflationSize)
  {
    #ifdef DEBUG_TIME
    Timer::start("checkHolesRectangleOutline","applyFilter");
    #endif

    std::set<unsigned int> valid;

    std::vector<std::vector<cv::Point> > inflatedRectangles;
    float key_y;
    float key_x;
    float vert_y;
    float vert_x;
    double theta;
    double keypointVertDist;

    for (unsigned int i = 0; i < rectangles.size(); i++)
    {
      std::vector<cv::Point> inflatedVertices;
      int inflatedVerticesWithinImageLimits = 0;

      for (int j = 0; j < 4; j++)
      {
        key_y = inKeyPoints[i].pt.y;
        key_x = inKeyPoints[i].pt.x;

        vert_y = rectangles[i][j].y;
        vert_x = rectangles[i][j].x;

        theta = atan2(key_y - vert_y, key_x - vert_x);

        keypointVertDist = sqrt(pow(key_x -vert_x, 2) + pow(key_y -vert_x, 2));

        //!< check if the inflated vertex has gone out of bounds
        if (vert_x - inflationSize * cos(theta) < inImage.cols &&
          vert_x - inflationSize * cos(theta) >= 0 &&
          vert_y - inflationSize * sin(theta) < inImage.rows &&
          vert_y - inflationSize * sin(theta) >= 0)
        {
          inflatedVerticesWithinImageLimits++;
        }

        inflatedVertices.push_back(
          cv::Point(round(vert_x - inflationSize * cos(theta)),
            round(vert_y - inflationSize * sin(theta))));
      } //!< end for rectangle's points

      //!< If one or more vertices are out of bounds discard the whole
      //!< inflated rectangle
      if (inflatedVerticesWithinImageLimits < 4)
      {
        inflatedVertices.clear();
        continue;
      }
      else
      {
        valid.insert(i);
        inflatedRectangles.push_back(inflatedVertices);
      }
    } //!< end for rectangles

    /**
     * For each inflated rectangle, store in visitedPoints
     * the points that constitute the rectangle.
     * We will test if these points all lie on one plane.
     **/
    //!< The indices of the valid (by this filter) keypoints
    std::set<unsigned int> finalIndices;
    std::set<unsigned int>::iterator validIterator = valid.begin();
    for (unsigned int i = 0; i < inflatedRectangles.size(); i++)
    {
      //!< The canvas image will hold the blobs' outlines, and their rectangles.
      cv::Mat canvas = cv::Mat::zeros(inImage.size(), CV_8UC1);
      cv::RNG rng(12345);
      cv::Scalar color = cv::Scalar(
        rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255));


      //!< Draw the inflated rectangle that corresponds to it
      for(int j = 0; j < 4; j++)
      {
        cv::line(canvas, inflatedRectangles[i][j],
          inflatedRectangles[i][(j + 1) % 4], color, 1, 8);
      }

      std::set<unsigned int> visitedPoints;
      for (unsigned int rows = 0; rows < inImage.rows; rows++)
      {
        for (unsigned int cols = 0; cols < inImage.cols; cols++)
        {
          if (canvas.data[rows * inImage.cols + cols] != 0)
          {
            visitedPoints.insert(rows * inImage.cols + cols);
          }
        }
      }


      /**
       * In order to construct a new point cloud
       * we need a priori knowledge of the valid points
       * of the input point cloud.
       * */
      int numValidVisitedPoints = 0;
      for(std::set<unsigned int>::iterator it = visitedPoints.begin();
        it != visitedPoints.end(); it++)
      {
        //!< If the point in the original point cloud is not missing (noise)
        if (!(initialPointCloud->points[*visitedPoints.find(*it)].x !=
            initialPointCloud->points[*visitedPoints.find(*it)].x))
        {
          numValidVisitedPoints++;
        }
      }

      PointCloudXYZPtr visitedPointsPointCloud (new PointCloudXYZ);

      visitedPointsPointCloud->width = numValidVisitedPoints;
      visitedPointsPointCloud->height = 1;
      visitedPointsPointCloud->points.resize
        (visitedPointsPointCloud->width * visitedPointsPointCloud->height);

      visitedPointsPointCloud->header.frame_id =
        initialPointCloud->header.frame_id;
      visitedPointsPointCloud->header.stamp =
        initialPointCloud->header.stamp;

      int pointCloudPointsIndex = 0;
      for(std::set<unsigned int>::iterator it = visitedPoints.begin();
        it != visitedPoints.end(); it++)
      {
        //!< if the point in the original point cloud is not missing (noise)
        if (!(initialPointCloud->points[*visitedPoints.find(*it)].x !=
            initialPointCloud->points[*visitedPoints.find(*it)].x))
        {
          visitedPointsPointCloud->points[pointCloudPointsIndex].x =
            initialPointCloud->points[*visitedPoints.find(*it)].x;

          visitedPointsPointCloud->points[pointCloudPointsIndex].y =
            initialPointCloud->points[*visitedPoints.find(*it)].y;

          visitedPointsPointCloud->points[pointCloudPointsIndex].z =
            initialPointCloud->points[*visitedPoints.find(*it)].z;

          pointCloudPointsIndex++;
        }
      }

      //!< Check if the visitedPointsPointCloud points are on a plane
      int numPlanes = PlanesDetection::locatePlanes(visitedPointsPointCloud,
        false);

      if (numPlanes == 1)
      {
        finalIndices.insert(*validIterator);
      }

      //!< Increment the validIterator so that it points to the next element
      //!< in the valid set
      validIterator++;
    }

    #ifdef DEBUG_TIME
    Timer::tick("checkHolesRectangleOutline");
    #endif

    return finalIndices;
  }


  /**
    @brief Checks the homogenity of the gradient of depth in an area
    enclosed by @param inOutlines
    @param[in] interpolatedDepthImage [const cv::Mat&] The input depth image
    @param[in] inKeyPoints [const std::vector<cv::KeyPoint>&] The keypoints
    of blobs
    @param[in] inOutlines [const std::vector<std::vector<cv::Point> >&]
    @param[out] msgs [std::vector<std::string>*] Debug messages
    @param[out] probabilitiesVector [std::vector<float>*] A vector
    of probabilities hinting to the certainty degree which with the
    candidate hole is associated. While the returned set may be reduced in
    size, the size of this vector is the same throughout and equal to the
    number of keypoints found and published by the rgb node
    @return std::set<unsigned int> The indices of valid (by this filter)
    blobs
   **/
  std::set<unsigned int> DepthFilters::checkHolesDepthHomogenity(
    const cv::Mat& interpolatedDepthImage,
    const std::vector<cv::KeyPoint>& inKeyPoints,
    const std::vector<std::vector<cv::Point> >& inOutlines,
    std::vector<std::string>* msgs,
    std::vector<float>* probabilitiesVector)
  {
    std::set<unsigned int> valid;

    //!< Temporary cv::Mats
    cv::Mat interpolatedDepthImageEdges;
    cv::Mat visualizableDenoisedImage;
    cv::Mat interpolatedTemp;

    interpolatedDepthImage.copyTo(interpolatedTemp);

    //!< Facilitate the edge detection by converting the 32FC1 image
    //!< values to a range of 0-255
    visualizableDenoisedImage = Visualization::scaleImageForVisualization
      (interpolatedTemp, DepthParameters::scale_method);

    //!< from now onwards every image is in the range of 0-255
    EdgeDetection::applySobel(visualizableDenoisedImage,
      &interpolatedDepthImageEdges);

    //!< Threshold the interpolatedDepthImageEdges image
    cv::threshold(interpolatedDepthImageEdges, interpolatedDepthImageEdges,
      DepthParameters::threshold_lower_value, 255, 3);

    //!< make all non zero pixels have a value of 255
    cv::threshold(interpolatedDepthImageEdges, interpolatedDepthImageEdges,
      0, 255, 0);

    for (unsigned int o = 0; o < inOutlines.size(); o++)
    {
      int numBlacks = 0;
      int numWhites = 0;
      for (unsigned int rows = 0; rows < interpolatedDepthImage.rows; rows++)
      {
        for (unsigned int cols = 0; cols < interpolatedDepthImage.cols; cols++)
        {
          //!< Check whether the current point resides inside the given outline
          if (cv::pointPolygonTest(
              inOutlines[o], cv::Point(cols, rows), false) > 0)
          {
            if (interpolatedDepthImageEdges.at<unsigned char>(rows, cols) != 0)
            {
              numWhites++;
            }
            else
            {
              numBlacks++;
            }
          }
        }
      }

      //!< If there is a variation in depth gradient inside the potential hole
      if (numWhites != 0)
      {
        valid.insert(o);
        probabilitiesVector->at(o) =
          (float) numWhites / (numWhites + numBlacks);
      }
      else
      {
        probabilitiesVector->at(o) = 0.0;
      }

      ROS_ERROR("probability [%f %f] : %f",
        inKeyPoints[o].pt.x, inKeyPoints[o].pt.y, probabilitiesVector->at(o));
    }

    return valid;
  }



  /**
    @brief Apply a cascade-like hole checker. Each filter applied is attached
    to an order which relates to the sequence of the overall filter execution.
    @param[in] interpolatedDepthImage [const cv::Mat&] The denoised depth image
    @param[in] initialPointCloud [const pcl::PointCloud<pcl::PointXYZ>::Ptr]
    The undistorted input point cloud
    @param[in][out] conveyor [HoleFilters::HolesConveyor*] A struct that
    contains the final valid holes
    @return void
   **/
  void DepthFilters::checkHoles(
    const cv::Mat& interpolatedDepthImage,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& initialPointCloud,
    HoleFilters::HolesConveyor* conveyor)
  {
    #ifdef DEBUG_TIME
    Timer::start("checkHoles","findHoles");
    #endif

    std::set<unsigned int> indexes;
    std::vector<std::string> finalMsgs;

    std::map<int, int> filtersOrder;

    if (HoleFusionParameters::run_checker_depth_diff > 0)
    {
      filtersOrder[HoleFusionParameters::run_checker_depth_diff] = 1;
    }
    if (HoleFusionParameters::run_checker_outline_of_rectangle > 0)
    {
      filtersOrder[HoleFusionParameters::run_checker_outline_of_rectangle] = 2;
    }
    if (HoleFusionParameters::run_checker_depth_area > 0)
    {
      filtersOrder[HoleFusionParameters::run_checker_depth_area] = 3;
    }
    if (HoleFusionParameters::run_checker_brushfire_outline_to_rectangle > 0)
    {
      filtersOrder[HoleFusionParameters::run_checker_brushfire_outline_to_rectangle]
        = 4;
    }

    std::vector<cv::Mat> imgs;
    std::vector<std::string> msgs;

    for (std::map<int, int>::iterator o_it = filtersOrder.begin();
      o_it != filtersOrder.end(); ++o_it)
    {
      applyFilter(
        o_it->second,
        interpolatedDepthImage,
        initialPointCloud,
        conveyor,
        HoleFusionParameters::rectangle_inflation_size,
        &imgs,
        &msgs);
    } //!< o_it iterator ends

    #ifdef DEBUG_SHOW
    if(HoleFusionParameters::debug_show_check_holes) // Debug
    {
      Visualization::multipleShow("checkHoles functions", imgs, msgs, 1200, 1);
    }
    #endif
    #ifdef DEBUG_TIME
    Timer::tick("checkHoles");
    #endif
  }


  /**
    @brief Apply a cascade-like hole checker. Each filter applied is attached
    to an order which relates to the sequence of the overall filter execution.
    @param[in] method [const unsigned int&] The filter identifier to execute
    @param[in] img [const cv::Mat&] The input depth image
    @param[in] pointCloud [const pcl::PointCloud<pcl::PointXYZ>::Ptr&] The
    original point cloud that corresponds to the input depth image
    @param[in][out] conveyor [HoleFilters::HolesConveyor*] The structure that
    holds the final holes' data
    @param[in] inflationSize [const int&] The amount of pixels by which each
    bounding box is inflated
    @param[in][out] imgs [std::vector<cv::Mat>*] A vector of images which shows
    the holes that are considered valid by each filter
    @param[in][out] msgs [std::vector<std::string>*] Debug messages
    @return void
   **/
  void DepthFilters::applyFilter(
    const unsigned int& method,
    const cv::Mat& img,
    const PointCloudXYZPtr& pointCloud,
    HoleFilters::HolesConveyor* conveyor,
    const int& inflationSize,
    std::vector<cv::Mat>* imgs,
    std::vector<std::string>* msgs)
  {
    #ifdef DEBUG_TIME
    Timer::start("applyFilter","checkHoles");
    #endif
    std::string windowMsg;
    std::set<unsigned int> indexes;
    std::vector<std::string> finalMsgs;
    std::vector<std::string> msgs_;
    std::vector<cv::KeyPoint> finalKeyPoints;
    std::vector<std::vector<cv::Point2f> > finalRectangles;
    std::vector<std::vector<cv::Point> > finalBlobsOutlineVector;

    //!< Initialize structures
    indexes.clear();
    finalKeyPoints.clear();
    finalRectangles.clear();
    finalBlobsOutlineVector.clear();
    finalMsgs.clear();
    msgs_.clear();

    switch(method)
    {
      //!< Filter #1 (through difference of depth)------------------------------
      case 1 :
        {
          indexes = checkHolesDepthDiff(
            img,
            conveyor->keyPoints,
            conveyor->rectangles,
            &msgs_,
            inflationSize);
          windowMsg = "Filter: Depth difference";
          break;
        }
        //!< Filter #2------------------------------------------------------------
        //!< Inflate the bounding boxes by an inflation size.
        //!< For a blob to be at least a potential hole, all the points that
        //!< constitute the inflated rectangle should lie on exactly one plane.
      case 2 :
        {
          indexes = checkHolesRectangleOutline(
            img,
            pointCloud,
            conveyor->keyPoints,
            conveyor->rectangles,
            inflationSize);
          windowMsg = "Filter: Outline of rectangle on plane";
          break;
        }
        //!< Filter #3 (depth & area comparison)----------------------------------
      case 3 :
        {
          indexes = checkHolesDepthArea(
            img,
            conveyor->keyPoints,
            conveyor->rectangles,
            &msgs_);
          windowMsg = "Filter: Area / Depth";
          break;
        }
        //!< Filter #4------------------------------------------------------------
        //!< Brushfire from blob outline to blob bounding box
        //!< with an inflation size (inflates the rectangle by x pixels).
        //!< If the points between the blob's outline and the inflated rectangle
        //!< lie on one plane, this blob is a hole.
      case 4 :
        {
          indexes = checkHolesBrushfireOutlineToRectangle(
            img,
            pointCloud,
            conveyor->keyPoints,
            conveyor->outlines,
            conveyor->rectangles,
            inflationSize);
          windowMsg = "Filter: Points around blob to plane";
          break;
        }
    }

    for(std::set<unsigned int>::iterator it = indexes.begin() ;
      it != indexes.end() ; it++)
    {
      finalKeyPoints.push_back(conveyor->keyPoints[*it]);
      finalRectangles.push_back(conveyor->rectangles[*it]);
      finalBlobsOutlineVector.push_back(conveyor->outlines[*it]);

      if(msgs_.size() == conveyor->keyPoints.size())
      {
        finalMsgs.push_back(msgs_[*it]);
      }
    }

    conveyor->outlines = finalBlobsOutlineVector;
    conveyor->rectangles = finalRectangles;
    conveyor->keyPoints = finalKeyPoints;

    #ifdef DEBUG_SHOW
    if(HoleFusionParameters::debug_show_check_holes) // Debug
    {
      std::string msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
      msg += STR(" ") + windowMsg;
      msgs->push_back(msg);

      cv::Mat tmp;
      tmp = Visualization::showHoles(
        windowMsg.c_str(),
        img,
        -1,
        conveyor->keyPoints,
        conveyor->rectangles,
        finalMsgs,
        conveyor->outlines);

      imgs->push_back(tmp);
    }
    #endif
    #ifdef DEBUG_TIME
    Timer::tick("applyFilter");
    #endif
  }

}

