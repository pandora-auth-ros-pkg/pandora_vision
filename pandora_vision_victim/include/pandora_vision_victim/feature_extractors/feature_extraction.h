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
* Author: Kofinas Miltiadis <mkofinas@gmail.com>
*********************************************************************/

#ifndef PANDORA_VISION_VICTIM_FEATURE_EXTRACTION_H
#define PANDORA_VISION_VICTIM_FEATURE_EXTRACTION_H

#include <string>
#include <vector>
#include <boost/filesystem.hpp>
#include <opencv2/opencv.hpp>

/**
 * @namespace pandora_vision
 * @brief The main namespace for PANDORA vision
 */
namespace pandora_vision
{
  /**
   * @class FeatureExtraction
   * @brief This class extracts features from images.
   */
  class FeatureExtraction
  {
    public:
      /**
       * @brief Default Constructor
       */
      FeatureExtraction()
      {
        if (!featureVector_.empty())
          featureVector_.clear();
        if (!featureMatrix_.empty())
          featureMatrix_.clear();
      }

      /**
       * @brief Default Destructor
       */
      virtual ~FeatureExtraction()
      {
      }

      /**
       * @brief This function extracts features according to the predefined
       * feature extraction algorithms.
       * @param inImage [const cv::Mat&] Frame to extract features from.
       * @ return void
       */
      virtual void extractFeatures(const cv::Mat& inImage)
      {
      }

      /**
       * @brief
       */
      virtual void constructFeaturesMatrix(
          const boost::filesystem::path& directory,
          cv::Mat* featuresMat, cv::Mat* labelsMat)
      {
      }

      /**
       * @brief
       */
      std::vector<double> getFeatureVector() const
      {
        return featureVector_;
      }

      /**
       * @brief
       */
      std::vector<std::vector<double> > getFeatureMatrix() const
      {
        return featureMatrix_;
      }

    protected:
      //! Vector containing the features extracted from a single image.
      std::vector<double> featureVector_;

      //! Matrix containing the feature vectors extracted from a set of images.
      std::vector<std::vector<double> > featureMatrix_;

  };
}// namespace pandora_vision
#endif  // PANDORA_VISION_VICTIM_FEATURE_EXTRACTION_H
