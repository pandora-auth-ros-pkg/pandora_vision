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
*   Marios Protopapas <protopapas_marios@hotmail.com>
*********************************************************************/

#include <vector>

#include "pandora_vision_victim/feature_extractors/rgb_feature_extraction.h"
#include "pandora_vision_victim/utilities/file_utilities.h"
/**
 * @namespace pandora_vision
 * @brief The main namespace for PANDORA vision
 */
namespace pandora_vision
{
  /**
   * @brief Default Constructor
   */
  RgbFeatureExtraction::RgbFeatureExtraction() : FeatureExtraction()
  {
  }

  /**
   * @brief Default Destructor
   */
  RgbFeatureExtraction::~RgbFeatureExtraction()
  {
  }

  /**
   * @brief This function extracts features from RGB images according to
   * a predefined set of feature extraction algorithms.
   * @param inImage [const cv::Mat&] RGB frame to extract features from.
   * @return void
   */
  void RgbFeatureExtraction::extractFeatures(const cv::Mat& inImage)
  {
    /// Extract Color Statistics features from RGB image
    std::vector<double> channelsStatisticsFeatureVector;
    ChannelsStatisticsExtractor::findColorChannelsStatisticsFeatures(inImage,
        &channelsStatisticsFeatureVector);

    /// Extract Edge Orientation features from RGB image
    std::vector<double> edgeOrientationFeatureVector;
    EdgeOrientationExtractor::findEdgeFeatures(inImage,
        &edgeOrientationFeatureVector);

    /// Extract Haralick features from RGB image
    std::vector<double> haralickFeatureVector;
    HaralickFeaturesExtractor::findHaralickFeatures(inImage,
        &haralickFeatureVector);

    /// Clear feature vector
    if (!featureVector_.empty())
      featureVector_.clear();

    /// Append Color Statistics features to RGB feature vector.
    for (int ii = 0; ii < channelsStatisticsFeatureVector.size(); ii++)
      featureVector_.push_back(channelsStatisticsFeatureVector[ii]);

    /// Append Edge Orientation features to RGB feature vector.
    for (int ii = 0; ii < edgeOrientationFeatureVector.size(); ii++)
      featureVector_.push_back(edgeOrientationFeatureVector[ii]);

    /// Append Haralick features to RGB feature vector.
    for (int ii = 0; ii < haralickFeatureVector.size(); ii++)
      featureVector_.push_back(haralickFeatureVector[ii]);
  }
}  // namespace pandora_vision
