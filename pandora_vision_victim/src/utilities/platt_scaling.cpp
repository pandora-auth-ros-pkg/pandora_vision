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
*   Marios Protopapas <protopapas_marios@hotmail.com>
*   Kofinas Miltiadis <mkofinas@gmail.com>
*********************************************************************/

#include <vector>
#include <opencv2/opencv.hpp>

#include "pandora_vision_victim/utilities/platt_scaling.h"

namespace pandora_vision
{
  PlattScaling::PlattScaling()
  {
    A_ = 1.0;
    B_ = 0.0;
  }

  PlattScaling::~PlattScaling()
  {
  }

  void PlattScaling::sigmoidTrain(const cv::Mat& predictedLabelsMat,
      const cv::Mat& actualLabelsMat)
  {
    double prior1 = 0, prior0 = 0;
    size_t numRows = actualLabelsMat.rows;
    for (int ii = 0; ii < predictedLabelsMat.rows; ii++)
    {
      if (actualLabelsMat.at<double>(ii, 0) > 0)
        prior1 += 1;
      else
        prior0 += 1;
    }

    int max_iter = 100;  // Maximal number of iterations
    double min_step = 1e-10;  // Minimal step taken in line search
    double sigma = 1e-12;  // For numerically strict PD of Hessian
    double eps = 1e-5;
    double hiTarget = (prior1 + 1.0) / (prior1 + 2.0);
    double loTarget = 1 / (prior0 + 2.0);

    double* t = new double[actualLabelsMat.rows];
    double fApB, p, q, h11, h22, h21, g1, g2, det, dA, dB, gd, stepsize;
    double newA, newB, newf, d1, d2, Avector, Bvector;
    int iter;

    // Initial Point and Initial Fun Value
    Avector = 0.0;
    Bvector = log((prior0 + 1.0) / (prior1 + 1.0));
    double fval = 0.0;

    for (int ii = 0; ii <actualLabelsMat.rows; ii++)
    {
      if (actualLabelsMat.at<double>(ii, 0) > 0)
        t[ii] = hiTarget;
      else
        t[ii]=loTarget;
      fApB = predictedLabelsMat.at<double>(ii, 0) * Avector + Bvector;
      if (fApB >= 0)
        fval += t[ii] * fApB + log(1 + exp(-fApB));
      else
        fval += (t[ii] - 1) * fApB + log(1 + exp(fApB));
    }
    for (iter = 0; iter < max_iter; iter++)
    {
      // Update Gradient and Hessian (use H' = H + sigma I)
      h11 = sigma;  // numerically ensures strict PD
      h22 = sigma;
      h21 = 0.0;
      g1 = 0.0;
      g2 = 0.0;
      for (int ii = 0; ii < predictedLabelsMat.rows; ii++)
      {
        fApB = predictedLabelsMat.at<double>(ii, 0) * Avector + Bvector;
        if (fApB >= 0)
        {
          p = exp(-fApB) / (1.0 + exp(-fApB));
          q = 1.0 / (1.0 + exp(-fApB));
        }
        else
        {
          p = 1.0 / (1.0 + exp(fApB));
          q = exp(fApB) / (1.0 + exp(fApB));
        }
        d2 = p * q;
        h11 += predictedLabelsMat.at<double>(ii, 0) * predictedLabelsMat.at<double>(ii, 0) * d2;
        h22 += d2;
        h21 += predictedLabelsMat.at<double>(ii, 0) * d2;
        d1 = t[ii] - p;
        g1 += predictedLabelsMat.at<double>(ii, 0) * d1;
        g2 += d1;
      }

      // Stopping Criteria
      if (fabs(g1) < eps && fabs(g2) < eps)
        break;

      // Finding Newton direction: -inv(H') * g
      det= h11 * h22 - h21 * h21;
      dA =- (h22 * g1 - h21 * g2) / det;
      dB=-(-h21 * g1 + h11 * g2) / det;
      gd = g1 * dA + g2 * dB;


      stepsize = 1;  // Line Search
      while (stepsize >= min_step)
      {
        newA = Avector + stepsize * dA;
        newB = Bvector + stepsize * dB;

        // New function value
        newf = 0.0;
        for (int ii = 0; ii < actualLabelsMat.rows; ii++)
        {
          fApB = predictedLabelsMat.at<double>(ii, 0) * newA + newB;
          if (fApB >= 0)
            newf += t[ii] * fApB + log(1 + exp(-fApB));
          else
            newf += (t[ii] - 1) * fApB +log(1 + exp(fApB));
        }
        // Check sufficient decrease
        if (newf < fval + 0.0001 * stepsize * gd)
        {
          Avector = newA;
          Bvector = newB;
          fval = newf;
          break;
        }
        else
          stepsize = stepsize / 2.0;
      }

      if (stepsize < min_step)
      {
        std::cout << "Line search fails in two-class probability estimates" << std::endl;
        break;
      }
    }

    if (iter >= max_iter)
      std::cout << "Reaching maximal iterations in two-class probability estimates" << std::endl;
    free(t);
    A_ = Avector;
    B_ = Bvector;
  }

  float PlattScaling::sigmoidPredict(float predicted)
  {
    float probability;
    float fApB = predicted * A_ + B_;

    float expFApB = static_cast<float>(exp(-abs(fApB)));
    if (fApB >= 0)
    {
      probability =  expFApB / (1.0f + expFApB);
    }
    else
    {
      probability = 1.0f / (1.0f + expFApB);
    }
    return probability;
  }

  std::vector<float> PlattScaling::sigmoidPredict(const cv::Mat& predicted)
  {
    std::vector<float> probabilityVector;
    cv::Mat fApB = predicted * A_ + B_;
    cv::Mat absFApB = -cv::abs(fApB);
    cv::Mat expFApB;
    cv::exp(absFApB, expFApB);

    for (int ii = 0; ii < expFApB.rows; ii++)
    {
      if (fApB.at<float>(ii, 0) >= 0)
      {
        probabilityVector.push_back(expFApB.at<float>(ii, 0) / (1.0f + expFApB.at<float>(ii , 0)));
      }
      else
      {
        probabilityVector.push_back(1.0f / (1.0f + expFApB.at<float>(ii, 0)));
      }
    }
    return probabilityVector;
  }

  void PlattScaling::load(const std::string& fileName)
  {
    cv::FileStorage fs(fileName, cv::FileStorage::READ);
    fs["A"] >> A_ ;
    fs["B"] >> B_ ;
    fs.release();
  }

  void PlattScaling::save(const std::string& fileName)
  {
    cv::FileStorage fs(fileName, cv::FileStorage::WRITE);
    fs << "A" << A_;
    fs << "B" << B_;
    fs.release();
  }
}  // namespace pandora_vision
