/*********************************************************************
*
* Software License Agreement (BSD License)
*
* Copyright (c) 2015, P.A.N.D.O.R.A. Team.
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
* Authors:
*   Marios Protopapas
*   Kofinas Miltiadis <mkofinas@gmail.com>
*********************************************************************/

#ifndef PANDORA_VISION_VICTIM_SVM_TRAINING_H
#define PANDORA_VISION_VICTIM_SVM_TRAINING_H

#include <cmath>
#include <iostream>
#include <fstream>

#include <opencv2/opencv.hpp>

#include <boost/filesystem.hpp>
#include <boost/lambda/bind.hpp>
#include <boost/algorithm/string.hpp>

#include <ros/ros.h>
#include <ros/package.h>

#include "pandora_vision_victim/victim_parameters.h"
#include "pandora_vision_victim/feature_extractors/feature_extraction.h"
#include "pandora_vision_victim/utilities/file_utilities.h"
#include "pandora_vision_victim/utilities/feature_extraction_utilities.h"

#define USE_OPENCV_GRID_SEARCH_AUTOTRAIN 1

namespace pandora_vision
{
  class SvmTraining
  {
    public:
      /**
       * @brief The Constructor
       */
      SvmTraining(const std::string& ns, int _num_feat, const std::string& datasetPath);

      /**
       * @brief The Destructor
       */
      virtual ~SvmTraining();

      /**
      @brief This method constructs the training matrix
      to be used for the training
      @param file_name [std::string]: file name to save the extracted training matrix
      @return void
      **/
      void constructFeaturesMatrix(const boost::filesystem::path& directory,
                                   const std::string& annotationsFile,
                                   cv::Mat* featuresMat, cv::Mat* labelsMat);
      /**
       * @brief
       * @param dataMat [cv::Mat*]
       * @return void
       */
      void performPcaAnalysis(cv::Mat* dataMat);

      /**
      @brief Function that implements the training for the subsystems
      according to the given training sets. It applies svm and extracts
      a suitable model
      @return void
      **/
      virtual void trainSubSystem()
      {
      }

      /**
      @brief Function that evaluates the training
      @param [cv::Mat&] predicted the predicted results
      @param [cv::Mat&] the actual results
      @return void
      **/
      void evaluate(const cv::Mat& predicted, const cv::Mat& actual);

      /**
      @brief Function that computes the vectors A,B necessary for the computation
      of the probablistic output of the svm bases on platt's binary svm
      probablistic Output
      @param dec_values [cv::Mat]: the distance from the hyperplane of the
      predicted results of the given test dataset
      @param labels [cv::Mat]: the true labels of the dataset
      @param A [double&]:, the vector A to be computed
      @param B [double&]: the vector B to be computed
      @return void
      **/
      void sigmoid_train(cv::Mat dec_values, cv::Mat labels, double* A, double* B);

    protected:
      //! The NodeHandle
      ros::NodeHandle _nh;

      VictimParameters vparams;

      //! String containing the type of the images used in the feature
      //! extraction process.
      std::string imageType_;

      //! Variable used for State Managing
      bool trainingNowON;

      //! Variable used to decide whether to perform the feature extraction or
      //! to read features from a file.
      bool doFeatureExtraction_;

      //! Variable used to decide whether to perform PCA analysis on the
      //! features or not.
      bool doPcaAnalysis_;

      //! Variable used to decide whether to perform feature normalization or
      //! not and what type of normalization should be chosen. 0 stands for no
      //! normalization, 1 stands for min-max normalization and 2 stands for
      //! z-Score normalization.
      int typeOfNormalization_;

      std::string path_to_samples;
      std::string package_path;

      int num_feat;
      float accuracy_;
      float precision_;
      float recall_;
      float fmeasure_;

      cv::Mat trainingFeaturesMat_;
      cv::Mat trainingLabelsMat_;
      cv::Mat testFeaturesMat_;
      cv::Mat testLabelsMat_;

      /// Feature Extractor
      FeatureExtraction* featureExtraction_;
      /// Feature Extraction Utilities used to perform feature normalization
      /// and/or feature selection.
      FeatureExtractionUtilities* featureExtractionUtilities_;
      /// Set up SVM's parameters
      CvSVMParams params;
      CvParamGrid CvParamGrid_gamma, CvParamGrid_C;

      /// Train the SVM
      CvSVM SVM;
  };
}// namespace pandora_vision
#endif  // PANDORA_VISION_VICTIM_SVM_TRAINING_H
