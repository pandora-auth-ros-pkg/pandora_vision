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
*   Marios Protopapas
*   Kofinas Miltiadis <mkofinas@gmail.com>
*********************************************************************/

#include <string>

#include <ros/ros.h>

#include "pandora_vision_victim/classifiers/svm_classifier.h"
#include "pandora_vision_victim/utilities/file_utilities.h"

namespace pandora_vision
{
  /**
   * @brief Constructor
   */
  SvmClassifier::SvmClassifier(const std::string& ns,
      int numFeatures, const std::string& datasetPath,
      const std::string& classifierType,
      const std::string& imageType)
      : AbstractClassifier(ns, numFeatures, datasetPath, classifierType, imageType)
  {
    ROS_INFO("[victim_node] : Creating SVM training instance!");

    classifierPtr_.reset(new CvSVM());

    ROS_INFO("Created %s %s Training Instance.", imageType_.c_str(), classifierType_.c_str());
  }

  /**
   * @brief Destructor
   */
  SvmClassifier::~SvmClassifier()
  {
    ROS_DEBUG("[victim_node] : Destroying Svm training instance");
  }

  /**
   * @brief Trains the corresponding classifier using the input features and training labels.
   * @param trainingFeatures[const cv::Mat&] The matrix containing the features that describe the
   * training set
   * @param trainingLabels[const cv::Mat&] The corresponding labels that define the class of each
   * training sample.
   * @param classifierFileDest[const std::string&] The file where the classifier will be stored.
   * @return bool True on successfull completions, false otherwise.
   */
  bool SvmClassifier::train(const cv::Mat& trainingSetFeatures, const cv::Mat trainingSetLabels,
      const std::string& classifierFileDest)
  {
    if (trainingSetFeatures.empty())
    {
      ROS_ERROR("[PANDORA_VISION_VICTIM_NEURAL_NETWORK]: The features matrix is empty!");
      return false;
    }
    if (trainingSetLabels.empty())
    {
      ROS_ERROR("[PANDORA_VISION_VICTIM_NEURAL_NETWORK]: The labels matrix is empty!");
      return false;
    }

    if (vparams.autoTrain)
    {
      std::cout << "(SVM 'grid search' => may take some time!)" << std::endl;
      classifierPtr_->train_auto(trainingSetFeatures, trainingSetLabels, cv::Mat(), cv::Mat(),
          vparams.params, 10, CvSVM::get_default_grid(CvSVM::C),
          CvSVM::get_default_grid(CvSVM::GAMMA),
          CvSVM::get_default_grid(CvSVM::P),
          CvSVM::get_default_grid(CvSVM::NU),
          CvSVM::get_default_grid(CvSVM::COEF),
          CvSVM::get_default_grid(CvSVM::DEGREE),
          true);

      vparams.params = classifierPtr_->get_params();
      std::cout << "Using optimal Parameters" << std::endl;
      std::cout << "degree=" << vparams.params.degree << std::endl;
      std::cout << "gamma=" << vparams.params.gamma << std::endl;
      std::cout << "coef0=" << vparams.params.coef0 << std::endl;
      std::cout << "C=" << vparams.params.C << std::endl;
      std::cout << "nu=" << vparams.params.nu << std::endl;
      std::cout << "p=" << vparams.params.p << std::endl;
    }
    else
    {
      classifierPtr_->train(trainingSetFeatures, trainingSetLabels, cv::Mat(), cv::Mat(), vparams.params);
    }


    classifierPtr_->save(classifierFile_.c_str());
    return true;
  }

  /**
   * @brief Validates the resulting classifier using the given features 
   * extracted from the test set.
   * @param testSetFeatures[const cv::Mat&] The test set features matrix
   * @param validationResults[cv::Mat*] The results for the test set.
   * @return void
   */
  void SvmClassifier::validate(const cv::Mat& testSetFeatures, cv::Mat* validationResults)
  {
    // uncomment to produce the platt probability
    // float prediction;
    // double A, B;
    // for (int ii = 0; ii < testFeaturesMat.rows; ii++)
    // {
    // prediction = classifierPtr_->predict(testFeaturesMat.row(ii), true);
    // results.at<double>(ii, 0)= prediction;
    // }
    // sigmoid_train(results, testLabelsMat, &A, &B);
    // std::cout << "A=" << A << std::endl;
    // std::cout << "B=" << B << std::endl;

    // uncomment for ONE_CLASS SVM
    // for (int ii = 0; ii < results.rows; ii++)
    // for (int jj = 0; jj < results.cols; jj++)
    // if(results.at<float>(ii, jj) == 0)
    // results.at<float>(ii, jj) = -1;

    classifierPtr_->predict(testSetFeatures, *validationResults);
    return;
  }

}  // namespace pandora_vision

