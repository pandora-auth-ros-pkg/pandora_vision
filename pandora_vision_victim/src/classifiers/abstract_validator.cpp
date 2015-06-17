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

#include <cmath>
#include <string>

#include <ros/console.h>

#include "pandora_vision_victim/classifiers/abstract_validator.h"
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
  AbstractValidator::AbstractValidator(const ros::NodeHandle& nh,
      const std::string& imageType,
      const std::string& classifierType)
  {
    nodeMessagePrefix_ = "[PANDORA_VISION_VICTIM_" + boost::to_upper_copy<std::string>(imageType)
        + "_" + boost::to_upper_copy<std::string>(classifierType) + "]";

    ROS_INFO_STREAM(nodeMessagePrefix_ << ": Creating " << imageType
        << " " << classifierType << " Validator instance");

    imageType_ = imageType;
    classifierType_ = classifierType;

    std::string classifierPath;
    if (!nh.getParam(classifierType_ + "/" + imageType_ + "/classifier_path", classifierPath))
    {
      ROS_ERROR_STREAM(nodeMessagePrefix_ << ": Could not retrieve "
          << "the classifier path parameter!");
      ROS_BREAK();
    }

    double probabilityScaling;
    if (!nh.getParam(classifierType_ + "/" + imageType_ + "/probability_scaling", probabilityScaling))
    {
      ROS_ERROR_STREAM(nodeMessagePrefix_ << ": Could not retrieve"
          << " the probability scaling parameter!");
      ROS_BREAK();
    }

    double probabilityTranslation;
    if (!nh.getParam(classifierType_ + "/" + imageType_ + "/probability_translation", probabilityTranslation))
    {
      ROS_ERROR_STREAM(nodeMessagePrefix_ << ": Could not retrieve "
          << "the probability translation parameter!");
      ROS_BREAK();
    }

    classifierPath_ = classifierPath;
    ROS_INFO_STREAM(classifierPath_);

    probabilityScaling_ = probabilityScaling;
    probabilityTranslation_ = probabilityTranslation;

    packagePath_ = ros::package::getPath("pandora_vision_victim");

    std::string paramFile = packagePath_ + "/config/" + imageType_ + "_" + classifierType_ + "_training_params.yaml";
    cv::FileStorage fs(paramFile, cv::FileStorage::READ);
    fs.open(paramFile, cv::FileStorage::READ);

    typeOfNormalization_ = static_cast<int>(fs["type_of_normalization"]);

    fs.release();

    std::string filesDirectory = packagePath_ + "/data/";
    if (typeOfNormalization_ == 1)
    {
      std::string normalizationParamOne = imageType_ + "_" + classifierType_ + "_min_values.xml";
      std::string normalizationParamOnePath = filesDirectory + normalizationParamOne;
      std::string normalizationParamTwo = imageType_ + "_" + classifierType_ + "_max_values.xml";
      std::string normalizationParamTwoPath = filesDirectory + normalizationParamTwo;

      std::string normalizationParamOneTag = "min";
      std::string normalizationParamTwoTag = "max";
      normalizationParamOneVec_ = file_utilities::loadFiles(
          normalizationParamOnePath, normalizationParamOneTag);
      normalizationParamTwoVec_ = file_utilities::loadFiles(
          normalizationParamTwoPath, normalizationParamTwoTag);
    }
    else if (typeOfNormalization_ == 2)
    {
      std::string normalizationParamOne = imageType_ + "_" + classifierType_ + "_mean_values.xml";
      std::string normalizationParamOnePath = filesDirectory + normalizationParamOne;
      std::string normalizationParamTwo = imageType_ + "_" + classifierType_ + "_standard_deviation_values.xml";
      std::string normalizationParamTwoPath = filesDirectory + normalizationParamTwo;

      std::string normalizationParamOneTag = "mean";
      std::string normalizationParamTwoTag = "std_dev";
      normalizationParamOneVec_ = file_utilities::loadFiles(
          normalizationParamOnePath, normalizationParamOneTag);
      normalizationParamTwoVec_ = file_utilities::loadFiles(
          normalizationParamTwoPath, normalizationParamTwoTag);
    }

    if (boost::iequals(imageType_ , "rgb"))
    {
      featureExtraction_ = new RgbFeatureExtraction();
    }
    else if (boost::iequals(imageType_, "depth"))
    {
      featureExtraction_ = new DepthFeatureExtraction();
    }
    else
    {
      ROS_ERROR_STREAM(nodeMessagePrefix_ << ": Wrong image type!"
          << " Cannot implement validator!");
      ROS_BREAK();
    }

    bool vocabularyNeeded = featureExtraction_->bagOfWordsVocabularyNeeded();
    if (vocabularyNeeded)
    {
      const std::string bagOfWordsFile = imageType_ + "_" + classifierType_ + "_bag_of_words.xml";
      const std::string bagOfWordsFilePath = filesDirectory + bagOfWordsFile;
      cv::Mat vocabulary = file_utilities::loadFiles(bagOfWordsFilePath,
          "bag_of_words");
      featureExtraction_->setBagOfWordsVocabulary(vocabulary);
    }
    featureExtractionUtilities_ = new FeatureExtractionUtilities();
    ROS_INFO_STREAM(nodeMessagePrefix_ << ": Initialized " << imageType_ << " "
        << classifierType_ << " Validator instance");
  }

  /**
   * @brief Default Destructor.
   */
  AbstractValidator::~AbstractValidator()
  {
  }

  /**
   * @brief This function extracts features according to the predefined
   * feature extraction algorithms.
   * @param inImage [const cv::Mat&] Frame to extract features from.
   * @return void
   */
  void AbstractValidator::extractFeatures(const cv::Mat& inImage)
  {
    featureExtraction_->extractFeatures(inImage);
  }

  /**
   * @brief This function extract features according to the
   * predifined features for the rgb image
   * @param inImage [cv::Mat] current rgb frame to be processed
   * @return void
  */
  void AbstractValidator::calculatePredictionProbability(const cv::Mat& inImage, float* classLabel, float* probability)
  {
    ROS_INFO_STREAM(nodeMessagePrefix_ << ": Extracting features");
    extractFeatures(inImage);
    ROS_INFO_STREAM(nodeMessagePrefix_ << ": Extracted Features");
    if (!featureVector_.empty())
      featureVector_.clear();
    featureVector_ = featureExtraction_->getFeatureVector();
    ROS_INFO_STREAM(nodeMessagePrefix_ << ": Find prediction probability");
    float prediction;

    cv::Mat featuresMat = cv::Mat(featureVector_);
    // Make features matrix a row vector.
    if (featuresMat.cols == 1)
      transpose(featuresMat, featuresMat);
    /// Normalize the data
    ROS_INFO_STREAM(nodeMessagePrefix_ << ": Normalize features");
    if (typeOfNormalization_ == 1)
    {
      double newMin = -1.0;
      double newMax = 1.0;
      featureExtractionUtilities_->performMinMaxNormalization(newMax, newMin,
          &featuresMat, normalizationParamOneVec_, normalizationParamTwoVec_);
    }
    else if (typeOfNormalization_ == 2)
    {
      featureExtractionUtilities_->performZScoreNormalization(
          &featuresMat, normalizationParamOneVec_, normalizationParamTwoVec_);
    }

    featuresMat.convertTo(featuresMat, CV_32FC1);
    predict(featuresMat, classLabel, &prediction);
    *probability = predictionToProbability(prediction);
    ROS_INFO_STREAM(nodeMessagePrefix_ << ": Class Label = " << *classLabel);
    ROS_INFO_STREAM(nodeMessagePrefix_ << ": Prediction = " << prediction);
  }

  /**
   * @brief This function calculates the classification probability according to
   * the classifier prediction.
   * @param prediction [float] The classifier prediction.
   * @return [float] The classification probability.
   */
  float AbstractValidator::predictionToProbability(float prediction)
  {
    if (prediction < 0.0f)
      prediction = fabs(prediction);

    // Normalize probability to [-1,1]
    float probability = std::tanh(probabilityScaling_ * prediction -
        probabilityTranslation_);
    // Normalize probability to [0,1]
    probability = (1.0f + probability) / 2.0f;
    ROS_INFO_STREAM(nodeMessagePrefix_ << ": Probability = " << probability);
    return probability;
  }
}  // namespace pandora_vision
