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

#include "pandora_vision_hazmat/training/planar_pattern_trainer.h"

/**
 @brief Main training function that reads input images and stores 
        the results in a corresponding xml file.
**/ 

void PlanarPatternTrainer::train()
{

  ROS_INFO("Starting Training!");
  const clock_t startingTime = clock();  
  
  // The number of images.
  int imgNum;

  // The temporary container for the images.
  cv::Mat img;

  // The vector of the training image set.
  std::vector<cv::Mat> images;
  std::string imgName;
  std::string inputFile;

  std::vector<std::string> inputDataNames;

  // Open the file with the different pattern names .
  std::string fileName;
  std::string line;
  
  //ROS_INFO("Reading the names of the patterns.");

  //while(std::getline(file, line))
  //{
    //if (line.empty())
      //continue;
    //inputDataNames.push_back(line);
  //}

  // Initialize the path to the data that will be used to train the system. 
  boost::filesystem::path trainingInputPath(packagePath_ +
        "/data/trainingInput/");
  
  // Create the container for all the files in the directory containing
  // the input of the training module.
  std::vector<boost::filesystem::path> trainingInputContents;

  // Iterate over the provided directory and store all
  // the files/paths it contains.
  copy(boost::filesystem::directory_iterator(trainingInputPath),
      boost::filesystem::directory_iterator(),
      std::back_inserter(trainingInputContents));

  // Sort the resulting data.
  sort(trainingInputContents.begin(), trainingInputContents.end());
  // Iterate over all the files/paths in the subdirectory.
  for (std::vector<boost::filesystem::path>::iterator 
      dirIterator(trainingInputContents.begin());
      dirIterator != trainingInputContents.end(); dirIterator++)
  {
    ROS_INFO("%s", dirIterator->c_str());
    try
    {
      // Check if the provided path exists.
      if (boost::filesystem::exists(*dirIterator))
      {
        // Check if it is a file.
        if (boost::filesystem::is_regular_file(*dirIterator))
        {
          singleViewTraining(*dirIterator);
        }
        // Check if it is a directory.
        else if(boost::filesystem::is_directory(*dirIterator))
        {
          ROS_INFO("%s is a directory !", dirIterator->c_str());
          ROS_INFO("Iterating over all the files in the directory!");
          // 
          std::vector<boost::filesystem::path> pathVector;

          // Iterate over the provided directory and store all
          // the paths it contains.
          copy(boost::filesystem::directory_iterator(*dirIterator),
              boost::filesystem::directory_iterator(),
              std::back_inserter(pathVector));

          // Sort the contained paths.
          sort(pathVector.begin(), pathVector.end());

          // Get the path to the image that represents the frontal view of
          // the pattern.
          std::vector<boost::filesystem::path>::iterator frontalViewPathIter;
          
          
          
          //if (boost::filesystem::is_regular_file(*frontalViewPathIter))
          //{
            
          //}

          // Get the path to the file containing the homographies that
          // created all the synthetic views.
          std::vector<boost::filesystem::path>::iterator
            homographyDataStoragePathIter;

          boost::filesystem::path homographyPath(dirIterator->string() +
              std::string("homography.txt"));

          homographyDataStoragePathIter = std::find(pathVector.begin(),
              pathVector.end(), homographyPath);
          if (homographyDataStoragePathIter == pathVector.end())
          {
            ROS_ERROR("Could not find the file containing the homography "
                "transformations that generated the new views!");
            continue;
          }
          int homographyDataStoragePathPos = std::distance(pathVector.begin(),
              homographyDataStoragePathIter);
          

          for (std::vector<boost::filesystem::path>::const_iterator it 
              (pathVector.begin()); 
              it != pathVector.end(); ++it)
          {
            ROS_INFO("Path %s ", it->c_str());
          }

        }
        else
        {
          ROS_INFO("Path %s exists but is neither a file nor a directory!",
              dirIterator->c_str());
        }
        
      }
      else
      {
        ROS_ERROR("Invalid path!");
        continue;
      }
      
  }
  catch(const boost::filesystem3::filesystem_error& ex)
  {
    ROS_ERROR("%s", ex.what());
  }


  }
  const clock_t endingTime = clock();
  ROS_INFO("Features calculated and saved.");
  ROS_INFO("Training is over !");
  ROS_INFO("Training time was : %f seconds", ( endingTime - startingTime ) /
      static_cast<double>(CLOCKS_PER_SEC));
}

/*
 * @brief This method iterates over a directory, reads every
 * instance/synthetic view,calculates features and keypoints for every single
 * one of them and stores them in a corresponding xml file.
 * @param dirPath[const boost::filesystem::path&]: The path of the directory.
*/
void PlanarPatternTrainer::multiViewTraining(const boost::filesystem::path& 
    dirPath)
{
  ROS_INFO("Processing directory : %s ", dirPath.c_str());
}

/*
 * @brief This method iterates over a directory, reads every
 * instance/synthetic view,calculates features and keypoints for every single
 * one of them and stores them in a corresponding xml file.
 * @param dirPath[const boost::filesystem::path&]: The path of the directory.
*/
void PlanarPatternTrainer::singleViewTraining(const boost::filesystem::path&
    dirPath)
{
  ROS_INFO("Reading image : %s ", dirPath.filename().c_str());
  cv::Mat descriptors;
  std::vector<cv::KeyPoint> keyPoints;
  std::vector<cv::Point2f> boundingBox; 

  cv::Mat img = cv::imread(dirPath.c_str());
  if (!img.data)
  {
    ROS_ERROR("Could not read image,proceeding to next pattern!");
    return;
  }
  //Calculate the image features.
  this->getFeatures(img, &descriptors, &keyPoints, &boundingBox );
  //Save the training data for the i-th image.
  
  this->saveDataToFile(dirPath.filename().string(), descriptors, keyPoints,
      boundingBox);
  return;
}

/**
 @brief Saves the training data to a proper XML file.
 @param patternName [const std::string &] : The name of the pattern.
 @param descriptors [const cv::Mat &] : The descriptors of the pattern.
 @param keyPoints [const std::vector<cv::Keypoint>] : The key points
        detected on the pattern.
**/                  
  
void PlanarPatternTrainer::saveDataToFile(const std::string &patternName,
  const cv::Mat &descriptors ,
  const std::vector<cv::KeyPoint> &keyPoints,
  const std::vector<cv::Point2f> &boundingBox) 
{

  // Create the file name where the results will be stored.
  std::string fileName(patternName);


  // Properly choose the name of the data file.
  std::string path("trainingData");

  path = path + "/" + this->getFeatureType() + "/";

  fileName = path + fileName;
  ROS_INFO("DEBUG MESSAGE : Saving fileName %s", fileName.c_str());
  // Opening the xml file for writing .
  cv::FileStorage fs( fileName + ".xml", cv::FileStorage::WRITE);
  
  if (!fs.isOpened())
  {
    ROS_ERROR("Could not open the file name: %s", fileName.c_str());
    return;
  }

  // Enter the name of the pattern.
  fs << "PatternName" << patternName;

  // Save the descriptors.
  fs << "Descriptors" << descriptors;

  // Calculate the number of keypoints found.
  int keyPointsNum = keyPoints.size();

  // Store the detected keypoints.
  fs << "PatternKeypoints" << "[";
  for (int i = 0 ; i < keyPointsNum ; i++ )
    fs << "{" << "Keypoint" <<  keyPoints[i].pt << "}";
  fs << "]";

  // Store the bounding box for the pattern.
  fs << "BoundingBox" << "[";
  for (int i = 0 ; i < boundingBox.size() ; i++ )
    fs << "{" << "Corner" <<  boundingBox[i] << "}";

  fs << "]";

  // Close the xml file .
  fs.release();

}


