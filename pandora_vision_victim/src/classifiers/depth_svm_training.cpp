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

#include <string>
#include <vector>

#include "pandora_vision_victim/classifiers/depth_svm_training.h"

namespace pandora_vision
{
  /**
   * @brief Constructor
   */
  DepthSvmTraining::DepthSvmTraining(const std::string& ns,
      int numFeatures, const std::string& datasetPath,
      const std::string& classifierType,
      const std::string& imageType)
      : SvmTraining(ns, numFeatures, datasetPath, classifierType, imageType)
  {
    std::string paramFile = packagePath_ + "/config/depth_svm_training_params.yaml";
    cv::FileStorage fs(paramFile, cv::FileStorage::READ);
    fs.open(paramFile, cv::FileStorage::READ);

    std::string trainingSetExtraction = fs["training_set_feature_extraction"];
    std::string testSetExtraction = fs["test_set_feature_extraction"];
    std::string loadModel = fs["load_classifier_model"];
    std::string pcaAnalysis = fs["do_pca_analysis"];

    trainingSetFeatureExtraction_ = trainingSetExtraction.compare("true") == 0;
    testSetFeatureExtraction_ = testSetExtraction.compare("true") == 0;
    loadClassifierModel_ = loadModel.compare("true") == 0;
    doPcaAnalysis_ = pcaAnalysis.compare("true") == 0;
    typeOfNormalization_ = static_cast<int>(fs["type_of_normalization"]);

    fs.release();

    imageType_ = "depth_";
    featureExtraction_ = new DepthFeatureExtraction();
    std::cout << "Created Depth SVM Training Instance" << std::endl;
  }

  /**
   * @brief Destructor
   */
  DepthSvmTraining::~DepthSvmTraining()
  {
  }

  /**
   * @brief Function that implements the training for the subsystems
   * according to the given training sets. It applies SVM and extracts
   * a suitable model.
   * @return void
   */
  void DepthSvmTraining::trainSubSystem()
  {
  }
}  // namespace pandora_vision
