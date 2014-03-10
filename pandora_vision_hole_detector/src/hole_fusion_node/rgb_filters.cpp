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
    @brief Checks for color homogenity in a region where points are
    constrained inside each @param inOutlines's elements. A candidate hole
    is considered valid if its H-V histogram has above a certain
    number of bins occupied.
    @param[in] inImage [const cv::Mat&] The RGB image
    @param[in] inKeyPoints [const std::vector<cv::KeyPoint>&] The vector
    of the candidate holes's keypoints
    @param[in] inOutlines [const std::vector<std::vector<cv::Point> >&]
    The vector of the candidate holes's outline points
    @param[out] probabilitiesVector [std::vector<float>*] A vector
    of probabilities hinting to the certainty degree which with the
    candidate hole is associated. While the returned set may be reduced in
    size, the size of this vector is the same throughout and equal to the
    number of keypoints found and published by the rgb node
    @return std::set<unsigned int> The indices of valid (by this filter)
    blobs
   **/
  std::set<unsigned int> RgbFilters::checkHolesColorHomogenity(
    const cv::Mat& inImage,
    const std::vector<cv::KeyPoint>& inKeyPoints,
    const std::vector<std::vector<cv::Point> >& inOutlines,
    std::vector<float>* probabilitiesVector)
  {
    //!< The valid (by this filter) blobs' indices
    std::set<unsigned int> valid;

    //!< inImage transformed from BGR format to HSV
    cv::Mat inImageHSV;
    cv::cvtColor(inImage, inImageHSV, cv::COLOR_BGR2HSV);

    for (unsigned int i = 0; i < inKeyPoints.size(); i++)
    {
      //!< Create the mask needed for the histogram of the
      //!< points inside this blobs'outline
      cv::Mat blobMask = cv::Mat::zeros(inImage.size(), CV_8UC1);

      //!< Draw the points inside the blob
      for (unsigned int rows = 0; rows < inImage.rows; rows++)
      {
        for (unsigned int cols = 0; cols < inImage.cols; cols++)
        {
          if (cv::pointPolygonTest(
              inOutlines[i], cv::Point(cols, rows), false) > 0)
          {
            blobMask.at<unsigned char>(rows, cols) = 255;
          }
        }
      }
      //!< Blob mask is now ready


      //!< Histogram-related parameters
      int h_bins = 180;
      int v_bins = 256;
      int histSize[] = { h_bins, v_bins };

      //!< hue varies from 0 to 179, saturation or value from 0 to 255
      float h_ranges[] = { 0, 180 };
      float v_ranges[] = { 0, 256 };

      const float* ranges[] = { h_ranges, v_ranges };

      //!< Use the 0-th and 2-nd channels - H and V
      int channels[] = { 0, 1 };


      //!< Calculate the blob's histogram
      cv::MatND blobHistogram;
      cv::calcHist(&inImageHSV, 1, channels, blobMask, blobHistogram,
        2, histSize, ranges, true, false);

      /*
       *cv::imshow("blobHistogram", blobHistogram);
       *cv::waitKey(1);
       */

      //!< Break the 180 X 256 into boxes of box_x X box_y (vertically by
      //!< horizontally). Measure how many non-zero points there are in each
      //!< box. If there are more than a threshold value, count that box as
      //!< an overall non zero box
      int box_x = 20;
      int box_y = 16;

      int overallNonZeroBoxes = 0;
      for (unsigned int rows = 0; rows < 180 / box_x; rows++)
      {
        for (unsigned int cols = 0; cols < 256 / box_y; cols++)
        {
          int nonZeroInBox = 0;
          for (unsigned int b_x = 0; b_x < box_x; b_x++)
          {
            for (unsigned int b_y = 0; b_y < box_y; b_y++)
            {
              if (blobHistogram.at<float>(
                  rows * box_x + b_x, cols * box_y + b_y) != 0)
              {
                nonZeroInBox++;
              }
            }
          }

          if (nonZeroInBox >
            HoleFusionParameters::non_zero_points_in_box_blob_histogram)
          {
            overallNonZeroBoxes++;
          }
        }
      }

      valid.insert(i);
      probabilitiesVector->at(i) =
        (float) overallNonZeroBoxes / (180 / box_x * 256 / box_y);

      /*
       *ROS_ERROR("probability: [%f %f] : %f",
       *  inKeyPoints[i].pt.x, inKeyPoints[i].pt.y, probabilitiesVector->at(i));
       */

    }

    return valid;
  }



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

    //!< The vector holding all the points that constitute each inflated
    //!< rectangle
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

    //!< The indices of the valid (by this filter) keypoints
    std::set<unsigned int> finalIndices;
    std::set<unsigned int>::iterator validIterator = valid.begin();
    for (unsigned int i = 0; i < inflatedRectangles.size(); i++)
    {
      //!< The canvas image will hold the blobs' outlines
      cv::Mat canvas = cv::Mat::zeros(inImage.size(), CV_8UC1);
      cv::RNG rng(12345);
      cv::Scalar color = cv::Scalar(
        rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));


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

      int boundingBoxLuminosity = 0;
      int boundingBoxDivisor = 0;
      int blobLuminosity = 0;
      int blobDivisor = 0;
      for (unsigned int rows = 0; rows < inImage.rows; rows++)
      {
        for (unsigned int cols = 0; cols < inImage.cols; cols++)
        {
          //!< Mean bounding box luminosity test
          if (canvas.data[rows * inImage.cols + cols] != 0)
          {
            boundingBoxLuminosity +=
              (uint8_t)luminosityImage.at<unsigned char>(rows, cols);
            boundingBoxDivisor += 1;
          }

          //!< Mean outline luminosity test
          if (cv::pointPolygonTest(
              inOutlines[i], cv::Point(cols, rows), false) > 0)
          {
            blobLuminosity +=
              (uint8_t)luminosityImage.at<unsigned char>(rows, cols);
            blobDivisor += 1;
          }
        }
      }

      //!< Mean luminosity of the points that the inflated rectangle is
      //consisted of, derived from the original bounding box of the blob
      float meanBoundingBoxLuminosity =
        (float) boundingBoxLuminosity / boundingBoxDivisor;

      //!< Mean luminosity of the entire blob
      float meanBlobLuminosity =
        (float) blobLuminosity / blobDivisor;

      std::set<unsigned int>::iterator it = valid.begin();
      std::advance(it, i);

      //!< If the luminosity of the inside of the candidate hole is greater
      //!< than the luminosity of its bounding box, it surely is not a hole
      if (meanBlobLuminosity > meanBoundingBoxLuminosity)
      {
        probabilitiesVector->at(*it) = 0.0;
      }
      else
      {
        finalIndices.insert(*validIterator);
        probabilitiesVector->at(*it) =
          1 - meanBlobLuminosity / meanBoundingBoxLuminosity;
      }

      //!< Increment the validIterator so that it points to the next element
      //!< in the valid set
      validIterator++;
    }
    return finalIndices;
  }



  /**
    @brief Given a set of keypoints and their respective outline and
    bounding box points, and a model histogram, this filter looks for near
    equation between the histograms of the points that consist the bounding
    box and the model histogram, and for major difference between the
    histograms of the bounding box and the points inside the outline of the
    blob.
    @param[in] inImage [const cv::Mat&] The input RGB image
    @param[in] inHistogram [const cv::MatND&]
    The model histogram's H and S component
    @param[in] inKeyPoints [const std::vector<cv::KeyPoint>&] The vector
    of the candidate holes's keypoints
    @param[in] inRectangles [const std::vector<std::vector<cv::Point2f> >&]
    The vector of the candidate holes's bounding boxes
    @param[in] inOutlines [const std::vector<std::vector<cv::Point> >&]
    The vector of the candidate holes's outline points
    @param[in] inflationSize [cosnt int&] grow the rectangle by
    inflationSize as to acquire more points to check for plane existence.
    @param[out] probabilitiesVector [std::vector<float>*] A vector
    of probabilities hinting to the certainty degree which with the
    candidate hole is associated. While the returned set may be reduced in
    size, the size of this vector is the same throughout and equal to the
    number of keypoints found and published by the rgb node
    @return std::set<unsigned int> The indices of valid (by this filter)
    blobs
   **/
  std::set<unsigned int> RgbFilters::checkHolesTextureDiff(
    const cv::Mat& inImage,
    const cv::MatND& inHistogram,
    const std::vector<cv::KeyPoint>& inKeyPoints,
    const std::vector<std::vector<cv::Point2f> >& inRectangles,
    const std::vector<std::vector<cv::Point> >& inOutlines,
    const int& inflationSize,
    std::vector<float>* probabilitiesVector)
  {
    std::set<unsigned int> valid;

    //!< inImage transformed from BGR format to HSV
    cv::Mat inImageHSV;
    cv::cvtColor(inImage, inImageHSV, cv::COLOR_BGR2HSV);


    //!< The vector holding all the points that constitute each inflated
    //!< rectangle
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

    //!< The indices of the valid (by this filter) keypoints
    std::set<unsigned int> finalIndices;
    std::set<unsigned int>::iterator validIterator = valid.begin();
    for (unsigned int i = 0; i < inflatedRectangles.size(); i++)
    {
      //!< Create the masks needed for the histograms of the outline points
      //!< and the points inside the blobs'outline
      cv::Mat rectangleMask = cv::Mat::zeros(inImage.size(), CV_8UC1);
      cv::Mat blobMask = cv::Mat::zeros(inImage.size(), CV_8UC1);

      cv::RNG rng(12345);
      cv::Scalar color = cv::Scalar(
        rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));


      //!< Draw the inflated rectangle that corresponds to it
      for(int j = 0; j < 4; j++)
      {
        cv::line(rectangleMask, inflatedRectangles[i][j],
          inflatedRectangles[i][(j + 1) % 4], color, 1, 8);
      }

      //!< Draw the points inside the blob
      for (unsigned int rows = 0; rows < inImage.rows; rows++)
      {
        for (unsigned int cols = 0; cols < inImage.cols; cols++)
        {
          if (cv::pointPolygonTest(
              inOutlines[i], cv::Point(cols, rows), false) > 0)
          {
            blobMask.at<unsigned char>(rows, cols) = 255;
          }
        }
      }
      //!< Masks are now ready

      //!< Histogram-related parameters
      int h_bins = 180;
      int s_bins = 256;
      int histSize[] = { h_bins, s_bins };

      //!< hue varies from 0 to 179, saturation from 0 to 255
      float h_ranges[] = { 0, 180 };
      float s_ranges[] = { 0, 256 };

      const float* ranges[] = { h_ranges, s_ranges };

      //!< Use the 0-th and 1-st channels
      int channels[] = { 0, 1 };


      //!< Produce and normalize the histogram for the
      //!< inflated rectangle's points
      cv::MatND rectangleHistogram;
      cv::calcHist(&inImageHSV, 1, channels, rectangleMask, rectangleHistogram,
        2, histSize, ranges, true, false);
      cv::normalize(rectangleHistogram, rectangleHistogram, 0, 1,
        cv::NORM_MINMAX, -1, cv::Mat());

      //!< Produce and normalize the histogram for the
      //!< points inside the outline of the blob
      cv::MatND blobHistogram;
      cv::calcHist(&inImageHSV, 1, channels, blobMask, blobHistogram,
        2, histSize, ranges, true, false);
      cv::normalize(blobHistogram, blobHistogram, 0, 1,
        cv::NORM_MINMAX, -1, cv::Mat());

      //!< Find the correlation between the model histogram and the histogram
      //!< of the inflated rectangle
      double rectangleToModelCorrelation= cv::compareHist(
        rectangleHistogram, inHistogram, CV_COMP_CORREL);

      //!< Find the correlation between the model histogram and the histogram
      //!< of the points inside the blob
      double blobToModelCorrelation = cv::compareHist(
        blobHistogram, inHistogram, CV_COMP_CORREL);

      //!< This blob is considered valid if there is a correlation between
      //!< the histograms of the rectangle and the model histogram (inHistogram)
      //!< greater than a threshold and, simultaneously, the blob's histogram
      //!< is more loosely correlated to the model histogram than the
      //!< rectangle's histogram is
      if (rectangleToModelCorrelation >=
        HoleFusionParameters::match_texture_threshold &&
        rectangleToModelCorrelation > blobToModelCorrelation)
      {
        finalIndices.insert(*validIterator);
        probabilitiesVector->at(*validIterator) =
          rectangleToModelCorrelation - blobToModelCorrelation;
      }
      else
      {
        probabilitiesVector->at(*validIterator) = 0.0;
      }

      //!< Increment the validIterator so that it points to the next element
      //!< in the valid set
      validIterator++;
    }

    return finalIndices;
  }


  /**
    @brief Given a set of keypoints and their respective outline and
    bounding box points, and a model histogram, this filter creates the
    back project of the @param inImage based on @param inHistogram and
    exports a vector of probabilities, that is a vector of how probable it
    is for a candidate hole's bounding box points to have a high probability
    in the back project image, and for the points inside the candidate
    hole's outline to have a low probability in the back project image
    @param[in] inImage [const cv::Mat&] The input RGB image
    @param[in] inHistogram [const cv::MatND&]
    The model histogram's H and S component
    @param[in] inKeyPoints [const std::vector<cv::KeyPoint>&] The vector
    of the candidate holes's keypoints
    @param[in] inRectangles [const std::vector<std::vector<cv::Point2f> >&]
    The vector of the candidate holes's bounding boxes
    @param[in] inOutlines [const std::vector<std::vector<cv::Point> >&]
    The vector of the candidate holes's outline points
    @param[in] inflationSize [cosnt int&] grow the rectangle by
    inflationSize as to acquire more points to check for plane existence.
    @param[out] probabilitiesVector [std::vector<float>*] A vector
    of probabilities hinting to the certainty degree which with the
    candidate hole is associated. While the returned set may be reduced in
    size, the size of this vector is the same throughout and equal to the
    number of keypoints found and published by the rgb node
    @return std::set<unsigned int> The indices of valid (by this filter)
    blobs
   **/
  std::set<unsigned int> RgbFilters::checkHolesTextureBackProject(
    const cv::Mat& inImage,
    const cv::MatND& inHistogram,
    const std::vector<cv::KeyPoint>& inKeyPoints,
    const std::vector<std::vector<cv::Point2f> >& inRectangles,
    const std::vector<std::vector<cv::Point> >& inOutlines,
    const int& inflationSize,
    std::vector<float>* probabilitiesVector)
  {
    std::set<unsigned int> valid;

    //!< inImage transformed from BGR format to HSV
    cv::Mat inImageHSV;
    cv::cvtColor(inImage, inImageHSV, cv::COLOR_BGR2HSV);

    //!< Histogram-related parameters
    //!< hue varies from 0 to 179, saturation from 0 to 255
    float h_ranges[] = { 0, 180 };
    float s_ranges[] = { 0, 256 };

    const float* ranges[] = { h_ranges, s_ranges };

    //!< Use the 0-th and 1-st channels
    int channels[] = { 0, 1 };

    //!< Calulate the inImageHSV's back project.
    //!< We will use it to find a mean probability for inHistogram's occurence
    //!< in the points consiting the inflated rectangle (below) and the points
    //!< inside the outline
    cv::MatND backProject;
    cv::calcBackProject(&inImageHSV, 1, channels, inHistogram, backProject,
      ranges, 1, true);

    //!< Is this needed?
    //!<cv::normalize(backProject, backProject, 0, 1,
    //!< cv::NORM_MINMAX, -1, cv::Mat());

    //!< The vector holding all the points that constitute each inflated
    //!< rectangle
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

    //!< The indices of the valid (by this filter) keypoints
    std::set<unsigned int> finalIndices;
    std::set<unsigned int>::iterator validIterator = valid.begin();
    for (unsigned int i = 0; i < inflatedRectangles.size(); i++)
    {
      //!< Create the masks needed for the histograms of the outline points
      cv::Mat rectangleMask = cv::Mat::zeros(inImage.size(), CV_8UC1);

      cv::RNG rng(12345);
      cv::Scalar color = cv::Scalar(
        rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));


      //!< Draw the inflated rectangle that corresponds to it
      for(int j = 0; j < 4; j++)
      {
        cv::line(rectangleMask, inflatedRectangles[i][j],
          inflatedRectangles[i][(j + 1) % 4], color, 1, 8);
      }

      float rectangleSum = 0;
      int rectanglePoints = 0;
      float blobSum = 0;
      int blobPoints = 0;
      for (unsigned int rows = 0; rows < inImage.rows; rows++)
      {
        for (unsigned int cols = 0; cols < inImage.cols; cols++)
        {
          if (rectangleMask.at<unsigned char>(rows, cols) != 0)
          {
            rectangleSum += backProject.at<float>(rows, cols);
            rectanglePoints++;
          }
          if (cv::pointPolygonTest(
              inOutlines[i], cv::Point(cols, rows), false) > 0)
          {
            blobSum += backProject.at<float>(rows, cols);
            blobPoints++;
          }
        }
      }

      //!< The average probability of the points consisting the inflated
      //!< rectangle matching the inHistogram
      float rectangleMatchProbability = rectangleSum / rectanglePoints;

      //!< The average probability of the points inside the blob's outline
      //!< matching the inHistogram
      float blobMatchProbability = blobSum / blobPoints;

      //!< This blob is considered valid, with a non zero validity probability,
      //!< if the points consisting the inflated rectangle have a greater
      //!< resemblance (through the probability-expressing values of the
      //!< back project cv::MatND) to the inHistogram than the one of the points
      //!< inside the blob's outline
      if (rectangleMatchProbability > blobMatchProbability)
      {
        probabilitiesVector->at(*validIterator) =
          rectangleMatchProbability - blobMatchProbability;

        finalIndices.insert(*validIterator);
      }
      else
      {
        probabilitiesVector->at(*validIterator) = 0.0;
      }

      //!< Increment the validIterator so that it points to the next element
      //!< in the valid set
      validIterator++;
    }

    return finalIndices;
  }
}
