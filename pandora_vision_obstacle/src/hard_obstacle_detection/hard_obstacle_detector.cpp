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
 *  THIS HARDWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS HARDWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors:
 *  Choutas Vassilis <vasilis4ch@gmail.com>
 *  Angelos Triantafyllidis <aggelostriadafillidis@gmail.com>
 *********************************************************************/

#include <string>
#include "pandora_vision_obstacle/hard_obstacle_detection/hard_obstacle_detector.h"

namespace pandora_vision
{
  HardObstacleDetector::HardObstacleDetector()
  {
  }

  HardObstacleDetector::HardObstacleDetector(const std::string& name,
      const ros::NodeHandle& nh)
  {
    ROS_INFO_NAMED(nodeName_, "Hard obstacle node is on");

    nodeName_ = name;

    double robotXDimention, robotYDimention;
    nh.param("robot/robotXDimention", robotXDimention, 0.345);
    nh.param("robot/robotYDimention", robotYDimention, 0.345);

    double resolution;
    nh.param("cellResolution", resolution, 0.02);

    robotRows_ = robotXDimention / resolution;
    robotCols_ = robotYDimention / resolution;

    // Minimum acceptable value for the process to work properly
    robotStrength_ = robotRows_ * robotCols_;

    robotMask_ = cv::Mat::ones(robotRows_, robotCols_, CV_64FC1);

    edge_method_ = 1;
    edges_threshold_ = 30;

    show_input_image = false;
    show_edges_image = false;
    show_edges_thresholded_image = false;
    show_edges_and_unknown_image = false;
    show_new_map_image = false;
    show_unknown_probabilities = false;
  }

  cv::Mat HardObstacleDetector::startDetection(const cv::Mat& inputImage)
  {
    // Check if input type is CV_64FC1
    if (inputImage.depth() != CV_64FC1)
    {
      ROS_ERROR_NAMED(nodeName_, "Hard obstacle node input image type was wrong");
      ROS_BREAK();
    }

    ROS_INFO_NAMED(nodeName_, "Hard obstacle detection has started");

    if (show_input_image)
    {
      // Show the input image
      showImage("The input image", inputImage, 1);
    }

    cv::Mat edgesImage;
    detectEdges(inputImage, &edgesImage);

    // Pass the unkown areas in edges image.
    fillUnkownAreas(inputImage, &edgesImage, 0);

    if (show_edges_and_unknown_image)
    {
      showImage("The edges image with unkown areas", edgesImage, 1);
    }

    // Pass the robot mask on the complete area that was made.
    cv::Mat newMap;
    robotMaskOnMap(edgesImage, &newMap);

    return newMap;
  }

  /*****************************************************************************
   *                         Visualization  methods                            *
   ****************************************************************************/
  void HardObstacleDetector::showImage(
    const std::string& title, const cv::Mat& image, int time)
  {
    if (image.depth() == CV_64FC1)
    {
      cv::Mat scaledImage = scaleFloatImageToInt(image);
      cv::cvtColor(scaledImage, scaledImage, CV_GRAY2RGB);

      // If value is negative, make it green for visualization
      for (unsigned int rows = 0; rows < image.rows; rows++)
      {
        for (unsigned int cols = 0; cols < image.cols; cols++)
        {
          if (image.at<double>(rows, cols) < 0)
          {
            scaledImage.at<unsigned char>(rows, 3 * cols + 0) = 0;
            scaledImage.at<unsigned char>(rows, 3 * cols + 1) = 255;
            scaledImage.at<unsigned char>(rows, 3 * cols + 2) = 0;
          }
        }
      }
    }
    cv::imshow(title, image);
    cv::waitKey(time);
  }

  void HardObstacleDetector::visualizeUnknownProbabilities(
    const std::string& title, const cv::Mat& image, int time)
  {
    if (image.depth() == CV_64FC1)
    {
      cv::Mat scaledImage = scaleFloatImageToInt(image);
      cv::cvtColor(scaledImage, scaledImage, CV_GRAY2RGB);

      // If value is negative, make it green for visualization
      for (unsigned int rows = 0; rows < image.rows; rows++)
      {
        for (unsigned int cols = 0; cols < image.cols; cols++)
        {
          double pixelValue = image.at<double>(rows, cols);
          double probability = applyFoldedNormalDistribution(pixelValue);

          // Check the probability of the pixel and give color for visualization
          if (probability < 0.35)
          {
            // Give Red color
            scaledImage.at<unsigned char>(rows, 3 * cols + 0) = 0;
            scaledImage.at<unsigned char>(rows, 3 * cols + 1) = 0;
            scaledImage.at<unsigned char>(rows, 3 * cols + 2) = 255;
          }
          else if (probability < 0.67)
          {
            // Give Yellow color
            scaledImage.at<unsigned char>(rows, 3 * cols + 0) = 255;
            scaledImage.at<unsigned char>(rows, 3 * cols + 1) = 255;
            scaledImage.at<unsigned char>(rows, 3 * cols + 2) = 0;
          }
          else if (probability < 1.0)
          {
            // Give Blue color
            scaledImage.at<unsigned char>(rows, 3 * cols + 0) = 255;
            scaledImage.at<unsigned char>(rows, 3 * cols + 1) = 0;
            scaledImage.at<unsigned char>(rows, 3 * cols + 2) = 0;
          }
        }
      }
      cv::imshow(title, scaledImage);
      cv::waitKey(time);
    }
  }

  double HardObstacleDetector::applyFoldedNormalDistribution(double inValue)
  {
    double probability;

    return probability;
  }

  /*****************************************************************************
   *                     End of Visualization methods                          *
   ****************************************************************************/

  cv::Mat HardObstacleDetector::scaleFloatImageToInt(const cv::Mat& inImage)
  {
    cv::Mat outImage;

    double min;
    double max;
    cv::minMaxIdx(inImage, &min, &max);

    inImage.convertTo(outImage, CV_8UC1, 255.0 / max);

    return outImage;
  }

  void HardObstacleDetector::fillUnkownAreas(
    const cv::Mat& inImage, cv::Mat* outImage, int method)
  {
    ROS_INFO_NAMED(nodeName_, "Hard obstacle node fills unkown area");

    switch (method)
    {
      case 0 :
        for (unsigned int rows = 0; rows < inImage.rows; rows++)
        {
          for (unsigned int cols = 0; cols < inImage.cols; cols++)
          {
            // Pass from the input mat the negative values as our policy dictates.
            if (inImage.at<double>(rows, cols) < 0)
            {
              outImage->at<double>(rows, cols) = -1 / robotStrength_;
            }
          }
        }
        break;
      case 1 :
        inImage.copyTo(*outImage);
        for (unsigned int rows = 0; rows < inImage.rows; rows++)
        {
          for (unsigned int cols = 0; cols < inImage.cols; cols++)
          {
            // If negative values in the input image, convert them to -1.
            if (outImage->at<double>(rows, cols) < 0)
            {
              outImage->at<double>(rows, cols) = -1;
            }
          }
        }
        break;
    }
  }

  void HardObstacleDetector::robotMaskOnMap(
    const cv::Mat& inImage, cv::Mat* outImage)
  {
    ROS_INFO_NAMED(nodeName_, "Hard obstacle node convolutes map with robot");

    cv::Mat newMap;

    cv::filter2D(inImage, newMap, -1, robotMask_,
      cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);

    if (show_new_map_image)
    {
      showImage("The new map after robot mask convolution", newMap, 1);
    }

    if (show_unknown_probabilities)
    {
      // Visualization of unkown areas based on their optimistic probabilities
      visualizeUnknownProbabilities("Unknown areas probabilities", newMap, 1);
    }

    // After convolution there might be negative values, so we need
    // to set them to -1.
    fillUnkownAreas(newMap, outImage, 1);
  }


  /*****************************************************************************
   *                         Edge Detection methods                            *
   ****************************************************************************/
  void HardObstacleDetector::applyCanny(
    const cv::Mat& inImage, cv::Mat* outImage)
  {
    if (inImage.depth() != CV_8U)
    {
      ROS_ERROR_NAMED(nodeName_, "At canny detection, inappropriate image depth");
      ROS_BREAK();
    }
    ROS_INFO_NAMED(nodeName_, "Canny edge detection is called");

    // Reduce the noise of the input Image
    cv::blur(inImage, *outImage,
      cv::Size(cannyBlurKernelSize_, cannyBlurKernelSize_));

    // Detect edges with canny
    cv::Canny(*outImage, *outImage, cannyLowThreshold_,
      cannyLowThreshold_ * 3, cannyKernelSize_);
  }

  void HardObstacleDetector::applyScharr(
    const cv::Mat& inImage, cv::Mat* outImage)
  {
    if (inImage.depth() != CV_8U)
    {
      ROS_ERROR_NAMED(nodeName_, "At scharr detection, inappropriate image depth");
      ROS_BREAK();
    }
    ROS_INFO_NAMED(nodeName_, "Scharr edge detection is called");

    cv::Mat blured;
    // Reduce the noise of the input image
    cv::GaussianBlur(inImage, blured, cv::Size(3, 3), 0, 0, cv::BORDER_DEFAULT);

    cv::Mat grad_x, grad_y;
    cv::Mat abs_grad_x, abs_grad_y;

    // Gradient X
    //        src, dst, ddepth, 1, 0, scale, delta, border_type
    cv::Scharr(blured, grad_x, CV_16S, 1, 0, 1, 0, cv::BORDER_DEFAULT);
    cv::convertScaleAbs(grad_x, abs_grad_x);

    // Gradient Y
    //        src, dst, ddepth, 1, 0, kernelSize, scale, delta, border_type
    cv::Scharr(blured, grad_y, CV_16S, 0, 1, 1, 0, cv::BORDER_DEFAULT);
    cv::convertScaleAbs(grad_y, abs_grad_y);

    // Total Gradient (approximate)
    cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, *outImage);
  }

  void HardObstacleDetector::applySobel(
    const cv::Mat& inImage, cv::Mat* outImage)
  {
    if (inImage.depth() != CV_8U)
    {
      ROS_ERROR_NAMED(nodeName_, "At sobel detection, inappropriate image depth");
      ROS_BREAK();
    }
    ROS_INFO_NAMED(nodeName_, "Sobel edge detection is called");

    cv::Mat blured;
    // Reduce the noise of the input image
    cv::GaussianBlur(inImage, blured, cv::Size(3, 3), 0, 0, cv::BORDER_DEFAULT);

    cv::Mat grad_x, grad_y;
    cv::Mat abs_grad_x, abs_grad_y;

    // Gradient X
    //        src, dst, ddepth, 1, 0, kernelSize, scale, delta, border_type
    cv::Sobel(blured, grad_x, CV_16S, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT);
    cv::convertScaleAbs(grad_x, abs_grad_x);

    // Gradient Y
    //        src, dst, ddepth, 1, 0, kernelSize, scale, delta, border_type
    cv::Sobel(blured, grad_y, CV_16S, 0, 1, 3, 1, 0, cv::BORDER_DEFAULT);
    cv::convertScaleAbs(grad_y, abs_grad_y);

    // Total Gradient (approximate)
    cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, *outImage);
  }

  void HardObstacleDetector::detectEdges(
    const cv::Mat& inImage, cv::Mat* outImage)
  {
    ROS_INFO_NAMED(nodeName_, "Hard obstacle node detects edges");

    cv::Mat scaledImage = scaleFloatImageToInt(inImage);

    switch (edge_method_)
    {
      case 0 :
        applyCanny(scaledImage, outImage);
        break;
      case 1 :
        applyScharr(scaledImage, outImage);
        break;
      case 2 :
        applySobel(scaledImage, outImage);
        break;
    }

    if (show_edges_image)
    {
      showImage("The edges image", *outImage, 1);
    }

    // Apply threshold to the edges
    cv::threshold(*outImage, *outImage, edges_threshold_, 255, CV_THRESH_BINARY);

    if (show_edges_thresholded_image)
    {
      showImage("The thresholded edges image", *outImage, 1);
    }

    // Convert the type of the output image to CV_64FC1.
    outImage->convertTo(*outImage, CV_64FC1, 1.0 / 255.0, 0.0);
  }
}  // namespace pandora_vision
