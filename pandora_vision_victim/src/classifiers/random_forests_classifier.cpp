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
*********************************************************************/

#include <string>

#include <ros/ros.h>

#include "pandora_vision_victim/classifiers/random_forests_classifier.h"
#include "pandora_vision_victim/utilities/file_utilities.h"

namespace pandora_vision
{
  /**
   * @brief Constructor
   */
  RandomForestsClassifier::RandomForestsClassifier(const std::string& ns,
      int numFeatures, const std::string& datasetPath,
      const std::string& classifierType,
      const std::string& imageType)
      : AbstractClassifier(ns, numFeatures, datasetPath, classifierType, imageType)
  {
    ROS_INFO("Created %s %s Training Instance.", imageType_.c_str(), classifierType_.c_str());
  }

  /**
   * @brief Destructor
   */
  RandomForestsClassifier::~RandomForestsClassifier()
  {
    ROS_DEBUG("[victim_node] : Destroying Random Forests training instance");
  }

  void RandomForestsClassifier::trainSubSystem()
  {
    int numTrainingFiles = file_utilities::findNumberOfAnnotations(trainingAnnotationsFile_);
    int numTestFiles = file_utilities::findNumberOfAnnotations(testAnnotationsFile_);

    cv::Mat trainingFeaturesMat = cv::Mat::zeros(numTrainingFiles, numFeatures_, CV_64FC1);
    cv::Mat trainingLabelsMat = cv::Mat::zeros(numTrainingFiles, 1, CV_64FC1);
    cv::Mat testFeaturesMat = cv::Mat::zeros(numTestFiles, numFeatures_, CV_64FC1);
    cv::Mat testLabelsMat = cv::Mat::zeros(numTestFiles, 1, CV_64FC1);

    if (loadClassifierModel_ && file_utilities::exist(classifierFile_.c_str()))
    {
      randomForestsClassifierPtr_->load(classifierFile_.c_str());
    }
    else
    {
      if (file_utilities::exist(trainingFeaturesMatrixFile_.c_str()) == false ||
          trainingSetFeatureExtraction_)
      {
        std::cout << "Create necessary training matrix" << std::endl;
        std::string prefix = "training_";

        bool vocabularyNeeded = constructBagOfWordsVocabulary(trainingDirectory_,
            trainingAnnotationsFile_);

        constructFeaturesMatrix(trainingDirectory_, trainingAnnotationsFile_,
            &trainingFeaturesMat, &trainingLabelsMat);

        std::cout << "Normalize features" << std::endl;
        normalizeFeaturesAndSaveNormalizationParameters(&trainingFeaturesMat);

        trainingFeaturesMat.convertTo(trainingFeaturesMat, CV_32FC1);
        trainingLabelsMat.convertTo(trainingLabelsMat, CV_32FC1);

        file_utilities::saveFeaturesInFile(trainingFeaturesMat, trainingLabelsMat,
            prefix, trainingFeaturesMatrixFile_, trainingLabelsMatrixFile_,
            imageType_);

        if (vocabularyNeeded)
        {
          std::cout << "Save bag of words vocabulary" << std::endl;
          const std::string bagOfWordsFile = imageType_ + classifierType_ + "bag_of_words.xml";
          const std::string bagOfWordsFilePath = filesDirectory_ + bagOfWordsFile;
          file_utilities::saveToFile(bagOfWordsFilePath, "bag_of_words",
              featureExtraction_->getBagOfWordsVocabulary());
        }
      }
      else
      {
        trainingFeaturesMat = file_utilities::loadFiles(
            trainingFeaturesMatrixFile_, "training_features_mat");
        trainingLabelsMat = file_utilities::loadFiles(
            trainingLabelsMatrixFile_, "training_labels_mat");
      }

      // Start Training Process
      std::cout << "Starting training process for the rgb images" << std::endl;
      randomForestsClassifierPtr_->train(trainingFeaturesMat, CV_ROW_SAMPLE, trainingLabelsMat,
          cv::Mat(), cv::Mat(), cv::Mat(), cv::Mat(), randomForestsParams_);
      randomForestsClassifierPtr_->save(classifierFile_.c_str());
      std::cout << "Finished training process" << std::endl;
    }
    if (file_utilities::exist(testFeaturesMatrixFile_.c_str()) == false ||
        testSetFeatureExtraction_)
    {
      std::cout << "Create necessary test matrix" << std::endl;
      std::string prefix = "test_";

      constructFeaturesMatrix(testDirectory_, testAnnotationsFile_,
          &testFeaturesMat, &testLabelsMat);

      loadNormalizationParametersAndNormalizeFeatures(&testFeaturesMat);

      testFeaturesMat.convertTo(testFeaturesMat, CV_32FC1);
      testLabelsMat.convertTo(testLabelsMat, CV_32FC1);

      file_utilities::saveFeaturesInFile(testFeaturesMat, testLabelsMat,
          prefix, testFeaturesMatrixFile_, testLabelsMatrixFile_, imageType_);
    }
    else
    {
      testFeaturesMat = file_utilities::loadFiles(
          testFeaturesMatrixFile_, "test_features_mat");
      testLabelsMat = file_utilities::loadFiles(
          testLabelsMatrixFile_, "test_labels_mat");
    }

    cv::Mat results = cv::Mat::zeros(numTestFiles, 1, CV_32FC1);
    for (int ii = 0; ii < testFeaturesMat.rows; ii++)
      results.at<float>(ii) = randomForestsClassifierPtr_->predict(testFeaturesMat.row(ii));
    file_utilities::saveToFile(resultsFile_, "results", results);
    evaluate(results, testLabelsMat);

  }

}  // namespace pandora_vision

