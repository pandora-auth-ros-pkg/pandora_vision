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

#include "hole_fusion_node/rgb_filters.h"

namespace pandora_vision
{
  /**
    @brief Checks for difference of mean value of luminosity between the
    pixels that comprise the blob's bounding box edges and the points
    inside the blob's outline.
    @param[in] inImage [const cv::Mat&] The RGB image
    @param[in] inKeyPoints [const std::vector<cv::KeyPoint>&] The vector
    of the candidate holes's keypoints
    @param[in] inRectangles [const std::vector<std::vector<cv::Point2f> >&]
    The vector of the candidate holes's bounding boxes
    @param[in] inOutlines [const std::vector<std::vector<cv::Point> >&]
    The vector of the candidate holes's outline points
    @param[in] inflationSize [cosnt int&] grow the rectangle by inflationSize
    as to acquire more points to check for plane existence.
    @param[out] probabilitiesVector [std::vector<float>*] A vector
    of probabilities hinting to the certainty degree which with the
    candidate hole is associated. While the returned set may be reduced in
    size, the size of this vector is the same throughout and equal to the
    number of keypoints found and published by the rgb node
    @return std::set<unsigned int> The indices of valid (by this filter)
    blobs
   **/
  std::set<unsigned int> RgbFilters::checkHolesLuminosityDiff(
    const cv::Mat& inImage,
    const std::vector<cv::KeyPoint>& inKeyPoints,
    const std::vector<std::vector<cv::Point2f> >& inRectangles,
    const std::vector<std::vector<cv::Point> >& inOutlines,
    const int& inflationSize,
    std::vector<float>* probabilitiesVector)
  {
    std::set<unsigned int> valid;

    std::vector<std::vector<cv::Point> > inflatedRectangles;
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
        probabilitiesVector->at(i) = 0.0;
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
     **/
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

      //!< Instead of applying the formula
      //!< Y = 0.299 * R + 0.587 * G + 0.114 * B to find the luminosity of each
      //!< pixel, turn the inImage into grayscale
      cv::Mat luminosityImage(inImage.size(), CV_8UC1);
      cv::cvtColor(inImage, luminosityImage, CV_BGR2GRAY);

      float boundingBoxLuminosity = 0;
      int boundingBoxDivisor = 0;
      float blobLuminosity = 0;
      int blobDivisor = 0;
      for (unsigned int rows = 0; rows < inImage.rows; rows++)
      {
        for (unsigned int cols = 0; cols < inImage.cols; cols++)
        {
          //!< Mean bounding box luminosity test
          if (canvas.data[rows * inImage.cols + cols] != 0)
          {
            boundingBoxLuminosity +=
              luminosityImage.at<unsigned char>(rows, cols);
            boundingBoxDivisor += 1;
          }

          //!< Mean outline luminosity test
          if (cv::pointPolygonTest(
              inflatedRectangles[i], cv::Point(cols, rows), false > 0))
          {
            blobLuminosity += inImage.at<unsigned char>(rows, cols);
            blobDivisor += 1;
          }
        }
      }

      //!< Mean luminosity of the points that the inflated rectangle is
      //consisted of, derived from the original bounding box of the blob
      float meanBoundingBoxLuminosity =
        boundingBoxLuminosity / boundingBoxDivisor;

      //!< Mean luminosity of the entire blob
      float meanBlobLuminosity =
        blobLuminosity / blobDivisor;

      std::set<unsigned int>::iterator it = valid.begin();
      std::advance(it, i);

      //!< If the luminosity of the inside of the candidate hole is greater
      //!< than the luminosity of its bounding box, it surely is not a hole
      if (meanBlobLuminosity > meanBoundingBoxLuminosity)
      {
        probabilitiesVector->at(*it) = 0.0;
        valid.erase(i);
      }
      else
      {
        probabilitiesVector->at(*it) =
          (1 - meanBlobLuminosity / meanBoundingBoxLuminosity);
      }
    }
  }

}

