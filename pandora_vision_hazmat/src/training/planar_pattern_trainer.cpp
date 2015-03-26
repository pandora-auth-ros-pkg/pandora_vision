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

  std::cout << "Starting Training! " << std::endl;
  const clock_t startingTime = clock() ;  
  // The number of images.
  int imgNum ;

  // The temporary container for the images.
  cv::Mat img;

  // The vector of the training image set.
  std::vector<cv::Mat> images;
  std::string imgName ;
  std::string inputFile;

  cv::Mat descriptors ;

  std::vector<cv::KeyPoint> keyPoints;
  std::vector<cv::Point2f> boundingBox ; 


  // Read the name of the file with the pattern titles.
  std::cout << "Please enter the name of the file with the pattern " <<
    "file names." << std::endl;
  std::getline(std::cin,inputFile);

  // Open the file with the different pattern names .
  cv::FileStorage fs( inputFile+".xml" ,cv::FileStorage::READ);


  // Check if the file was opened .
  if ( !fs.isOpened() )
  {
    std::cout << "File: " << inputFile << " could not be opened!" <<
      std::endl; 
    return ;
  }

  std::vector<std::string> input ;

  // Temporary string file.
  std::string inputName ;

  cv::FileNode inputNames = fs["PatternName"];

  // Check if the xml node is a string sequence .
  if ( inputNames.type() != cv::FileNode::SEQ)
  {
    std::cerr << "strings is not a sequence! FAIL" << std::endl;
    return ;
  }

  // Initialize the file Iterators .   
  cv::FileNodeIterator it = inputNames.begin() ;
  cv::FileNodeIterator itEnd = inputNames.end() ;

  std::cout << "Reading Pattern Names." << std::endl;

  // Read the name of the pattern files.
  for ( ; it != itEnd ; ++it ) 
  {

    inputName = (std::string)(*it)["name"]  ;
    input.push_back(inputName) ;
  }

  

  std::cout << "Calculating Pattern Features " << std::endl;

  cv::Mat hist;

  for (int i = 0 ; i < input.size() ; i++ )
  {
    // Name of the image to be read. 
    imgName = "patterns/" + input[i] + ".png" ; 
    img = cv::imread(imgName);
    if ( !img.data ) 
    {
      std::cout << "Error reading image : " << imgName << std::endl;
      std::cout << "Proceeding with the next image ! " << std::endl;
      continue;
    }
    

    // Calculate the Hue - Saturation histogram of the image.
    this->calculateHistogram(&hist);
    // Calculate the image features.
    this->getFeatures(img, &descriptors , &keyPoints , &boundingBox );
    // Save the training data for the i-th image.
    this->saveData( input[i] , descriptors , keyPoints , boundingBox 
        , hist);

    keyPoints.clear();
    boundingBox.clear();
  }

  const clock_t endingTime = clock();
  std::cout << "Features calculated and saved." << std::endl;
  std::cout << "Training is over ! " << std::endl;
  std::cout << "Training time was : " << ( endingTime - startingTime ) /
    static_cast<double>(CLOCKS_PER_SEC) << " seconds " << std::endl;
}

/**
 @brief Saves the training data to a proper XML file.
 @param patternName [const std::string &] : The name of the pattern.
 @param descriptors [const cv::Mat &] : The descriptors of the pattern.
 @param keyPoints [const std::vector<cv::Keypoint>] : The key points
        detected on the pattern.
**/                  
  
void PlanarPatternTrainer::saveData(const std::string &patternName,
  const cv::Mat &descriptors ,
  const std::vector<cv::KeyPoint> &keyPoints,
  const std::vector<cv::Point2f> &boundingBox , 
  const cv::Mat &histogram)
{

  // Create the file name where the results will be stored.
  std::string fileName(patternName);


  // Properly choose the name of the data file.
  std::string path("trainingData") ;

  path = path + "/" + this->getFeatureType() + "/";

  fileName = path + fileName ;
  std::cout << fileName << std::endl;
  // Opening the xml file for writing .
  cv::FileStorage fs( fileName + ".xml" ,cv::FileStorage::WRITE);
  
  if ( ! fs.isOpened() )
  {
    std::cout << "Error opening file : " << fileName << std::endl;
    std::cout << "Proceeding to next pattern !" << std::endl;
    return;
  }

  // Enter the name of the pattern.
  fs << "PatternName" << patternName ;

  // Save the descriptors.
  fs << "Descriptors" << descriptors ;

  // Save the histogram
  fs << "Histogram" << histogram ;

  // Calculate the number of keypoints found.
  int keyPointsNum = keyPoints.size() ;

  // Store the detected keypoints.
  fs << "PatternKeypoints" << "[" ;
  for (int i = 0 ; i < keyPointsNum ; i++ )
    fs << "{" << "Keypoint" <<  keyPoints[i].pt << "}" ;

  fs << "]" ;

  // Store the bounding box for the pattern.
  fs << "BoundingBox" << "[" ;
  for (int i = 0 ; i < boundingBox.size() ; i++ )
    fs << "{" << "Corner" <<  boundingBox[i] << "}" ;

  fs << "]" ;

  // Close the xml file .
  fs.release();


}


// The function that will calculate the histogram of the pattern.
void PlanarPatternTrainer::calculateHistogram(cv::Mat *hist)
{
  // The parameters will be read by an yml/xml file .
  int hbins = 15, sbins = 16;  
  int histSize[] = {hbins, sbins};
  float hranges[] = { 0, 180 };
  float sranges[] = { 0, 256 };
  const float* ranges[] = { hranges, sranges };
  int channels[] = {0, 1};


  cv::Mat hsv;

  cv::cvtColor( currentImage_ , hsv , CV_BGR2HSV );

  calcHist( &hsv , 1 , channels, cv::Mat() , // do not use mask
      *hist , 2 , histSize , ranges ,
      true , // the histogram is uniform
      false );

  cv::normalize( *hist , *hist , 255 , 0 , cv::NORM_L1 );   
}

