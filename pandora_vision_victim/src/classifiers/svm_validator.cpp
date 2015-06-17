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
*   Kofinas Miltiadis <mkofinas@gmail.com>
*   Protopapas Marios <protopapas_marios@hotmail.com>
*********************************************************************/

#include <string>

#include <ros/console.h>

#include "pandora_vision_victim/classifiers/svm_validator.h"
#include "pandora_vision_victim/utilities/file_utilities.h"

/**
 * @namespace pandora_vision
 * @brief The main namespace for PANDORA vision
 */
namespace pandora_vision
{
  /**
   * @brief Constructor. Initializes SVM classifier parameters and loads
   * classifier model.
   * @param classifierPath [const std::string&] The path to the classifier
   * model.
   */
  SvmValidator::SvmValidator(const ros::NodeHandle& nh,
      const std::string& imageType,
      const std::string& classifierType)
      : AbstractValidator(nh, imageType, classifierType)
  {
    svmValidator_.load(classifierPath_.c_str());

    double svmC;
    if (!nh.getParam(classifierType_ + "/" + imageType_ + "/svm_C", svmC))
    {
      ROS_ERROR_STREAM(nodeMessagePrefix_ << ": Could not retrieve "
          << "the C parameter for the SVM classifier!");
      ROS_BREAK();
    }

    double svmGamma;
    if (!nh.getParam(classifierType_ + "/" + imageType_ + "/svm_gamma", svmGamma))
    {
      ROS_ERROR_STREAM(nodeMessagePrefix_ << ": Could not retrieve "
          << "the gamma parameter for the SVM Classifier!");
      ROS_BREAK();
    }

    int svmType;
    if (!nh.getParam(classifierType_ + "/" + imageType_ + "/svm_type", svmType))
    {
      ROS_ERROR_STREAM(nodeMessagePrefix_ << ": Could not retrieve "
          << "the type of the SVM Classifier!");
      svmType = CvSVM::C_SVC;
      ROS_INFO_STREAM(nodeMessagePrefix_ << ": Using the default type " << svmType);
    }

    int kernelType;
    if (!nh.getParam(classifierType_ + "/" + imageType_ + "/kernel_type", kernelType))
    {
      ROS_ERROR_STREAM(nodeMessagePrefix_ << ": Could not retrieve "
          << "the kernel type of the SVM Classifier!");
      kernelType = CvSVM::RBF;
      ROS_INFO_STREAM(nodeMessagePrefix_ << ": Using the default type " << kernelType);
    }

    svmParams_.C = svmC;
    svmParams_.gamma = svmGamma;
    svmParams_.svm_type = svmType;
    svmParams_.kernel_type = kernelType;
    svmParams_.term_crit = cvTermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS,
        10000, 1e-6);
  }

  /**
   * @brief Default Destructor.
   */
  SvmValidator::~SvmValidator()
  {
  }

  /**
   * @brief Function that loads the trained classifier and makes a prediction
   * according to the feature vector given for each image
   * @return void
   */
  void SvmValidator::predict(const cv::Mat& featuresMat,
      float* classLabel, float* prediction)
  {
    *classLabel = svmValidator_.predict(featuresMat, false);
    *prediction = svmValidator_.predict(featuresMat, true);
  }
}  // namespace pandora_vision
