/*********************************************************************
*
* Software License Agreement (BSD License)
*
* Copyright (c) 2014, P.A.N.D.O.R.A. Team.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of the P.A.N.D.O.R.A. Team nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* Author: Victor Daropoulos
*********************************************************************/

#include "pandora_vision_landoltc/landoltc3d_detector.h"
#define SHOW_DEBUG_IMAGE true

namespace pandora_vision
{
//!< Constructor
LandoltC3dDetector::LandoltC3dDetector()
{
  _minDiff = 60;

  _threshold = 90;
  
}

//!< Destructor
LandoltC3dDetector::~LandoltC3dDetector()
{
  ROS_INFO("[landoltc3d_node]: landoltc3d_detector instance destroyed");
}

/**
  @brief Function for the initialization of the reference image
  @param void
  @return void
**/
void LandoltC3dDetector::initializeReferenceImage(std::string path)
{
  //!<Loading reference image passed as argument to main
  cv::Mat ref;
  //std::cout << path << std::endl;
  ROS_DEBUG_STREAM("path: " << path);
  ref = cv::imread(path);
  if (!ref.data)
    //std::cout << "Pattern image not loaded" << std::endl;
    ROS_DEBUG("Pattern image not loaded");

  //!< Turning to gray and binarizing ref image

  cv::cvtColor(ref, ref, CV_BGR2GRAY);
  cv::threshold(ref, ref, 0, 255, cv::THRESH_BINARY_INV | CV_THRESH_OTSU);

  cv::findContours(ref, _refContours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
  
  clahe = cv::createCLAHE(4, cv::Size(8, 8));
}

/**
  @brief Function for applying BradleyThresholding on Image
  @param in [cv::Mat&] Input Image to be thresholded
  @param out [cv::Mat*] Output, thresholded image
  @return void
**/

void LandoltC3dDetector::applyBradleyThresholding(const cv::Mat& input, cv::Mat* output)
{
  int64_t sum = 0;
  
  int count = 0;
      
  int index;
      
  uint64_t* integralImg =new uint64_t[input.cols*input.rows];
          
  unsigned char* in;
  
  in=(unsigned char*)input.data;
      
  unsigned char* out=(unsigned char*)output->data;
  
  int x1, y1, x2, y2;
      
  int s2 = input.cols/16;
      
        
  for(int i = 0; i < input.cols; i++)
  {
    sum = 0;
        
    for(int j = 0; j < input.rows; j++)
    {
      index = j*input.cols+i;
      sum+=in[index];
      if(i == 0) 
      integralImg[index]=sum;
      else
      integralImg[index]=integralImg[index-1]+sum;
    }
  }

  for(int i = 0; i < input.cols; i++)
  {
    for(int j = 0; j < input.rows; j++)
    {
      index = j*input.cols+i;
      
      x1 = i-s2;
      x2 = i+s2;
      y1 = j-s2;
      y2 = j+s2;
          
      if (x1 < 0) x1 = 0;
      if (x2 >= input.cols) x2 = input.cols-1;
      if (y1 < 0) y1 = 0;
      if (y2 >= input.rows) y2 = input.rows-1;
          
      count=(x2-x1)*(y2-y1);
          
      sum = integralImg[y2*input.cols+x2]-integralImg[y1*input.cols+x2]-
      integralImg[y2*input.cols+x1]+integralImg[y1*input.cols+x1];
          
      if((int64_t)(in[index]*count) < (int64_t)(sum*(1.0-0.15)))
      {
        out[index]=0;
      }
      else
      {
        out[index]=255;
      }
    }
  }
  
  //cv::imshow("bradley",*output);
  
  delete[] integralImg;
}
      

/**
  @brief Rasterize line between two points
  @param A [cv::Point] The start point of a line
  @param B [cv::Point] The end point of a line
  @return void
**/

void LandoltC3dDetector::rasterizeLine(cv::Point A, cv::Point B)
{
  uint16_t* votingData = reinterpret_cast<uint16_t*>(_voting.data);
  cv::Rect r(0, 0, _voting.cols, _voting.rows);

  //!< if line is out of frame return
  if(!cv::clipLine(r, A, B)) return;

  float rounding = 0.49999;

  int dx = B.x - A.x;
  int dy = B.y - A.y;

  //!<X major line

  if (abs(dx) >= abs(dy))
  {
    if (B.x < A.x) std::swap(A, B);

    //!<calculation of values of line, gradient and intercept
    float gradient = (B.y - A.y) / static_cast<float>(B.x - A.x);
    float yIntercept = A.y - gradient * A.x + rounding;
    for (int i = A.x; i < B.x; i++)
    {
      float y = gradient * i + yIntercept;
      votingData[(static_cast<int>(y) * _voting.cols) + i] += 1.0;
    }
  }

  //!<Y major line

  if (abs(dx) < abs(dy))
  {
    if (B.y < A.y) std::swap(A, B);

    //!<calculation of values of line, gradient and intercept
    float gradient = (B.x - A.x) / static_cast<float>(B.y - A.y);
    float xIntercept = A.x - gradient * A.y + rounding;
    for (int i = A.y; i < B.y; i++)
    {
      float x = gradient * i + xIntercept;
      votingData[i * _voting.cols + static_cast<int>(x)] += 1.0;
    }
  }
}

/**
  @brief Finds Centers based on gradient
  @param rows [int] Number of rows of matrix
  @param cols [int] Number of columns of matrix
  @param grX [float*] X gradient component
  @param grY [float*] Y gradient component
  @return void
**/

void LandoltC3dDetector::findCenters(int rows, int cols, float* grX, float* grY)
{
  cv::Point center;

  //!< Rasterization of lines between thresholded points

  for (int x = 0; x < cols; x++)
  {
    for (int y = 0; y < rows; y++)
    {
      int i = y * cols + x;
      float dx = grX[i];
      float dy = grY[i];
      float mag = dx * dx + dy * dy;
      if (mag > (_minDiff * _minDiff))
      {
        mag = sqrt(mag);
        float s = 20 / mag;
        rasterizeLine(cv::Point(x + dx * s, y + dy * s), cv::Point(x - dx * s, y - dy * s));
      }
    }
  }

  const uint16_t* readvoting = (const uint16_t*)_voting.data;
  int de = 2;
  int bullcount = 0;

  //!<Searching for landoltC centers

  for (int y = de; y < rows - de ; y++)
  {
    for (int x = de; x < cols - de; x++)
    {
      int i = y * cols + x;
      int cur = readvoting[i];
      if (cur >= _threshold)
      {
        bool biggest = true;

        //!<Search if there's a bigger center in a smaller area
        for (int dy = -de; dy < de && biggest; dy++)
        {
          for(int dx = -de; dx < de; dx++)
          {
            float her = readvoting[(y + dy) * cols + (x + dx)];
            if (cur < her)
            {
              biggest = false;
              break;
            }
          }
        }

        if (biggest)
        {
          center.x = x;
          center.y = y;
          _centers.push_back(center);
         // std::cout << "Bullseye " << bullcount++ << " xy " << center.x << "," << center.y << std::endl;
        }
      }
    }
  }

  return;
}

/**
  @brief Finds LandoltC Contours on RGB Frames
  @param inImage [const cv::Mat&] Input Image
  @param rows [int] Number of rows of matrix
  @param cols [int] Number of columns of matrix
  @param ref [std::vector<cv::Point>] Vector containing contour points of reference image
  @return void
**/

void LandoltC3dDetector::findLandoltContours(const cv::Mat& inImage, int rows, int cols, std::vector<cv::Point> ref)
{
  cv::RNG rng(12345);

  //!<find contours and moments in frame used for shape matching later

  std::vector<std::vector<cv::Point> > contours;
  cv::findContours(inImage, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
  std::vector<cv::Moments> mu(contours.size());
  for(int i = 0; i < contours.size(); i++)
  {
    mu[i] = moments(contours[i], false);
  }

  std::vector<cv::Point2f> mc(contours.size());
  for(int i = 0; i < contours.size(); i++)
  {
    mc[i] = cv::Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
  }

  std::vector<cv::Point> approx;

  //!<Shape matching using Hu Moments, and contour center proximity

  for(int i = 0; i < contours.size(); i++)
  {
    approxPolyDP(cv::Mat(contours[i]), approx, arcLength(cv::Mat(contours[i]), true) * 0.02, true);
    std::vector<cv::Point> cnt = contours[i];
    double prec = cv::matchShapes(cv::Mat(ref), cv::Mat(cnt), CV_CONTOURS_MATCH_I3, 0);

    for(std::vector<cv::Point>::iterator it = _centers.begin(); it != _centers.end(); ++it)
    {
      if (!isContourConvex(cv::Mat(cnt)) && fabs(mc[i].x - (*it).x) < 9 && fabs(mc[i].y - (*it).y) < 9 && prec < 0.3)
      {
        //std::cout << "Prec is : " << prec << std::endl;
        cv::Rect bounding_rect = boundingRect((contours[i]));
        cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        cv::drawContours(_coloredContours, contours, i, color, CV_FILLED, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
        _fillColors.push_back(color);
        _newCenters.push_back(mc[i]);
        // cv::imshow("Contours", _coloredContours);
        _rectangles.push_back(bounding_rect);

      }
    }
  }
}


/**
  @brief Function called from ImageCallBack that Initiates LandoltC search in the frame
  @param input [cv::Mat*] Matrix containing the frame received from the camera
  @return void
**/

void LandoltC3dDetector::begin(cv::Mat* input)
{
  cv::Mat gray, gradX, gradY, dst, abs_grad_x, abs_grad_y, thresholded, grad_x, grad_y;
  cv::Mat erodeKernel(cv::Size(1, 1), CV_8UC1, cv::Scalar(1));

  cv::cvtColor(*input, gray, CV_BGR2GRAY);

  _voting = cv::Mat::zeros(input->rows, input->cols, CV_16U);
  _coloredContours = cv::Mat::zeros(input->rows, input->cols, input->type());
  thresholded = cv::Mat::zeros(input->rows, input->cols, CV_8UC1);
  
  bilateralFilter(gray, dst, 3, 6, 1.5);
  
  gray = dst.clone();
  
  clahe->apply(gray, dst);
  
  cv::Sobel(dst, gradX, CV_32F, 1, 0, 3);
  cv::Sobel(dst, gradY, CV_32F, 0, 1, 3);

  float* gradXF = reinterpret_cast<float*>(gradX.data);
  float* gradYF = reinterpret_cast<float*>(gradY.data);

  findCenters(dst.rows, dst.cols, gradXF, gradYF);
  
  for(std::size_t i = 0; i < _centers.size(); i++)
  {
    cv::circle(*input, _centers.at(i), 2, (0, 0, 255), -1);
  }
  
  //Scharr(dst, grad_x, CV_32F, 1, 0, 1, 0);
  //Sobel( dst, grad_x, CV_32F, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT );
  convertScaleAbs( gradX, abs_grad_x );
  
  //Scharr(dst, grad_y, CV_32F, 0, 1, 1, 0);
  //Sobel( dst, grad_y, CV_32F, 0, 1, 3, 1, 0, cv::BORDER_DEFAULT );
  convertScaleAbs( gradY, abs_grad_y );
      
  addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, dst ); 
  
  cv::imshow("dst", dst);
  
  //applyBradleyThresholding(dst, &thresholded);
  cv::adaptiveThreshold(dst, thresholded, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV, 25, 2);
  
  cv::erode(thresholded, thresholded, erodeKernel);
  
  cv::imshow("thresholded", thresholded);
  
  findLandoltContours(thresholded, input->rows, input->cols, _refContours[0]);

  for(std::size_t i = 0; i < _rectangles.size(); i++)
  {
    cv::rectangle(*input, _rectangles.at(i), cv::Scalar(0, 0, 255), 1, 8, 0);
  }
  
  fuse();
  
  #ifdef SHOW_DEBUG_IMAGE
    cv::imshow("Raw", *input);
    cv::waitKey(5);
  #endif

  _centers.clear();
  _newCenters.clear();
  _rectangles.clear();
  _fillColors.clear();

}


void LandoltC3dDetector::fuse()
{
  for(int i = 0; i < _newCenters.size(); i++)
  {
    if(bbox.contains(_newCenters.at(i)))
    {
      ROS_INFO("It contains it");
    }
  }
  
  bbox = cv::Rect(-1, -1, -1, -1);
  
}


} // namespace pandora_vision
