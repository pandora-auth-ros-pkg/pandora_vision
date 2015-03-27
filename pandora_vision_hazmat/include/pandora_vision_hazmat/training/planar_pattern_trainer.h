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
 * Authors: Choutas Vassilis 
 *********************************************************************/

#ifndef PANDORA_VISION_HAZMAT_TRAINING_PLANAR_PATTERN_TRAINER_H
#define PANDORA_VISION_HAZMAT_TRAINING_PLANAR_PATTERN_TRAINER_H

#include "pandora_vision_hazmat/training/utilities.h"


/** 
  @class PlanarPatternTrainer
  @brief Abstract class used for training the Hazmat detector
 **/

class PlanarPatternTrainer{

  public:
    /*
     * @brief: Function used to produce the necessary keypoints and their
     *          corresponding descriptors for an image. 
     * @param image[const cv::Mat&] : The image we want to process.
     * @param descriptors[cv::Mat*]: A pointer to the array that will be used to
     * store the descriptors of the current image.
     * @param keyPoints[std::vector<cv::KeyPoint>*] : A pointer to the vector
     * containing the Keypoints detected in the current image.
     * @param boundingBox[std::vector<cv::Point2f>*] : A pointer to the vector
     * containing the bounding box for the pattern in the current image.
     **/
    virtual void getFeatures(const cv::Mat& image,
        cv::Mat *descriptors, 
        std::vector<cv::KeyPoint>* keyPoints,
        std::vector<cv::Point2f>* boundingBox) = 0;

    /*
     * @brief: Function used to produce the necessary keypoints and their
     *          corresponding descriptors for an image. 
     * @param image[const cv::Mat&] : The image we want to process.
     * @param descriptors[cv::Mat*]: A pointer to the array that will be used to
     * store the descriptors of the current image.
     * @param keyPoints[std::vector<cv::KeyPoint>*] : A pointer to the vector
     * containing the Keypoints detected in the current image.
     **/
    virtual void getFeatures(const cv::Mat& image,
        cv::Mat *descriptors,
        std::vector<cv::KeyPoint>* keyPoints) = 0;

    virtual const std::string getFeatureType() = 0;

    /*
     * @brief: Function used to produce the necessary keypoints and their
     *          corresponding descriptors for an image. 
     * @param images[const std::vector<cv::Mat&>] : The image we want 
     * to process.
     * @param imageNames[const std::vector<std::string>&] : The vector
     * containing the image names.
     * @param descriptors[cv::Mat*]: A pointer to the vector that will be 
     * used to store the descriptors for each image.
     * @param keyPoints[std::vector<cv::KeyPoint>*] : A pointer to the vector
     * containing the Keypoints detected in every image.
    **/
    virtual void getFeatures(const std::vector<cv::Mat>& images,
        const std::vector<std::string>& imageNames,
        std::vector<cv::Mat>* descriptors,
        std::vector<std::vector<cv::KeyPoint> >* keyPoints) = 0;

    /**
      @brief Sets the current image that will be used to train the detector.
      @param img [const cv::Mat] : The image of the pattern that we want 
      to detect .
     **/            
    virtual void setCurrentImage(const cv::Mat &img)
    {
      currentImage_ = img;
    }


    /**
      @brief Main training function that reads input images and stores 
      the results in a corresponding xml file.
     **/ 
    void train();

    /*
     * @brief This method iterates over a directory, reads every
     * instance/synthetic view,calculates features and keypoints for every single
     * one of them and stores them in a corresponding xml file.
     * @param dirPath[const boost::filesystem::path&]: The path of the directory.
     */

    void singleViewTraining(const boost::filesystem::path& dirPath);

    /*
     * @brief This method iterates over a directory, reads every
     * instance/synthetic view,calculates features and keypoints for every single
     * one of them and stores them in a corresponding xml file.
     * @param dirPath[const boost::filesystem::path&]: The path of the directory.
     */
    bool getHomographyMatrices(
        const boost::filesystem::path& homographyFilePath,
        std::map<std::string, cv::Mat>* homographyMatrices);
    /*
     * @brief This method iterates over a directory, reads every
     * instance/synthetic view,calculates features and keypoints for every single
     * one of them and stores them in a corresponding xml file.
     * @param dirPath[const boost::filesystem::path&]: The path of the directory.
     */
    void directoryProcessor(const boost::filesystem::path& 
        dirPath);
    void multiViewTraining(const boost::filesystem::path& dirPath);
    /**
      @brief Saves the training data to a proper XML file.
      @param patternName [const std::string &] : The name of the pattern.
      @param descriptors [const cv::Mat &] : The descriptors of the pattern.
      @param keyPoints [const std::vector<cv::Keypoint>] : The key points
      detected on the pattern.
     **/
    void saveDataToFile(const boost::filesystem::path &patternName,
        const cv::Mat &descriptors ,
        const std::vector<cv::KeyPoint> &keyPoints,
        const std::vector<cv::Point2f> &boundingBox);
    /**
      @brief Default Object Constructor. 
     **/ 
    PlanarPatternTrainer()
    {
      // Get the path of the package.
      packagePath_ = ros::package::getPath("pandora_vision_hazmat");
    }
  protected:
    cv::Mat currentImage_; //!< Current image used for traing.
    std::string packagePath_;
};


#endif  // PANDORA_VISION_HAZMAT_TRAINING_PLANAR_PATTERN_TRAINER_H
