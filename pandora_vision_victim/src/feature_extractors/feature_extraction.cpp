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

#include "pandora_vision_victim/feature_extractors/feature_extraction.h"
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
  FeatureExtraction::FeatureExtraction()
  {
    if (!featureVector_.empty())
      featureVector_.clear();
    if (!featureMatrix_.empty())
      featureMatrix_.clear();
  }

  /**
   * @brief Default Destructor
   */
  FeatureExtraction::~FeatureExtraction()
  {
  }

  /**
   * @brief This function extracts features according to the predefined
   * feature extraction algorithms.
   * @param inImage [const cv::Mat&] Frame to extract features from.
   * @ return void
   */
  void FeatureExtraction::extractFeatures(const cv::Mat& inImage)
  {
  }

  /**
   * @brief This function constructs the features matrix, i.e. the feature
   * vectors of a set of images.
   * @param directory [const boost::filesystem::path&] The directory that
   * contains the set of images for the feature extraction.
   * @param annotationsFile [const std::string&] The name of the file that
   * contains the class attributes of the images to be processed.
   * @param featuresMat [cv::Mat*] The features matrix.
   * @param labelsMat [cv::Mat*] The matrix that contains the class attributes
   * for the processed set of images.
   * @return void
   */
  void FeatureExtraction::constructFeaturesMatrix(
      const boost::filesystem::path& directory,
      const std::string& annotationsFile,
      cv::Mat* featuresMat, cv::Mat* labelsMat)
  {
    cv::Mat image, imageROI;
    std::vector<std::string> annotatedImages;
    std::vector<cv::Rect> boundingBox;
    std::vector<int> classAttributes;

    bool successfulFileLoad = file_utilities::loadAnnotationsFromFile(
        annotationsFile, &boundingBox, &annotatedImages, &classAttributes);
    int rowIndex = 0;
    for (boost::filesystem::recursive_directory_iterator iter(directory);
         iter != boost::filesystem::recursive_directory_iterator();
         iter++)
    {
      if (is_directory(iter->status()))
        continue;
      std::string imageAbsolutePath = iter->path().string();
      std::string imageName = iter->path().filename().string();
      image = cv::imread(imageAbsolutePath);
      if (!image.data)
      {
        std::cout << "Error reading file " << imageName << std::endl;
        continue;
      }
      // cv::Size size(640, 480);
      // cv::resize(image, image, size);

      if (successfulFileLoad)
      {
        std::cout << "Read class attribute from annotation file." << std::endl;
        for (int ii = 0; ii < annotatedImages.size(); ii++)
        {
          if(annotatedImages[ii] == imageName)
          {
            imageROI = image(boundingBox[ii]);
                      annotatedImages[ii].clear();
            labelsMat->at<double>(rowIndex, 0) = classAttributes[ii];
            break;
          }
        }
        extractFeatures(imageROI);
      }
      else
      {
        std::cout << "Find class attrbitute from image name." << std::endl;
        extractFeatures(image);

        std::string checkIfPositive = "positive";
        if (boost::algorithm::contains(imageName, checkIfPositive))
          labelsMat->at<double>(rowIndex, 0) = 1.0;
        else
          labelsMat->at<double>(rowIndex, 0) = -1.0;
      }

      std::cout << "Feature vector of image " << imageName << " "
                << featureVector_.size() << std::endl;

      for (int jj = 0; jj < featureVector_.size(); jj++)
        featuresMat->at<double>(rowIndex, jj) = featureVector_[jj];

      rowIndex += 1;
    }
  }

  /**
   * @brief
   */
  std::vector<double> FeatureExtraction::getFeatureVector() const
  {
    return featureVector_;
  }

  /**
   * @brief
   */
  std::vector<std::vector<double> > FeatureExtraction::getFeatureMatrix() const
  {
    return featureMatrix_;
  }
}// namespace pandora_vision
