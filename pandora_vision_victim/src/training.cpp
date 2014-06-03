/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, P.A.N.D.O.R.A. Team.
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
* Author: Despoina Paschalidou
*********************************************************************/

#include "pandora_vision_victim/training.h"

namespace pandora_vision
{
/**
  @brief Constructor
**/
SvmTraining::SvmTraining() : _nh(), trainingNowON(false)

{
  ROS_INFO("[victim_node] : Created Svm training instance");
  
  //constructRgbTrainingMatrix();
  constructDepthTrainingMatrix();
  
}

/**
  @brief Destructor
*/
SvmTraining::~SvmTraining()
{
  ROS_DEBUG("[victim_node] : Destroying Svm training instance");
}


 /**
   * @brief This method constructs the rgb training matrix 
   * to be used for the training
   * @return void
  */
void SvmTraining::constructRgbTrainingMatrix()
{
  ROS_INFO("ENTER");
  /*if(!trainingNowON)
  {
    ROS_INFO("EXIT");
    return;
  }*/
    char img_name[50];
    cv::Mat img;
    std::vector<double> _rgbFeatureVector;
    int num_files = 9500;
    int num_feat = 121;
    cv::Mat training_mat(num_files,num_feat,CV_64FC1);
    cv::FileStorage fs("/home/marios/rgbtraining.yml", cv::FileStorage::WRITE);


    for (int i=0; i<num_files; i++)
    {
      if(i<3500)
      sprintf(img_name,"/home/marios/opencv_traincascade/data/Positive_Images/positive%d.jpg",i+1);
      else
      sprintf(img_name,"/home/marios/opencv_traincascade/data/Negative_Images/negative%d.jpg",i+1-3500);
      std::cout<<img_name<<std::endl;
      img=cv::imread(img_name);
      if (img.cols == 0) 
      {
       std::cout << "Error reading file " << img_name << std::endl;
       return;
      }
      cv::Size size(640,480);
      cv::resize(img,img,size);
      
      _rgbSystem.extractRgbFeatures(img);
      _rgbFeatureVector=_rgbSystem.getRgbFeatureVector();
      
      std::cout<<"rgbFeatureVector of image "<<img_name<<" "<<_rgbFeatureVector.size()<<std::endl;

      for (int j=0; j<_rgbFeatureVector.size(); j++)
      {
        training_mat.at<double>(i,j)=_rgbFeatureVector[j];
      }
    }
    if (!fs.isOpened())
    {
      fs.open("/home/marios/rgbtraining.yml", cv::FileStorage::WRITE);
    }
      fs << "training_mat" << training_mat;
      fs.release();
    
  
    std::cout << "training matrix = " << std::endl << " " << training_mat.size()<< std::endl;
}

 /**
   * @brief This method constructs the depth training matrix 
   * to be used for the training
   * @return void
  */

void SvmTraining::constructDepthTrainingMatrix()
{
  ROS_INFO("ENTER DEPTH");
  /*if(!trainingNowON)
  {
    ROS_INFO("EXIT");
    return;
  }*/
    char img_name[50];
    cv::Mat img;
    std::vector<double> _depthFeatureVector;
    int num_files = 9500;
    int num_feat = 121;
    cv::Mat training_mat(num_files,num_feat,CV_64FC1);
    cv::FileStorage fs("/home/marios/depthtraining.yml", cv::FileStorage::WRITE);


    for (int i=0; i<num_files; i++)
    {
      if(i<3500)
      sprintf(img_name,"/home/marios/opencv_traincascade/data/Positive_Images/positive%d.jpg",i+1);
      else
      sprintf(img_name,"/home/marios/opencv_traincascade/data/Negative_Images/negative%d.jpg",i+1-3500);
      img=cv::imread(img_name);
      if (img.cols == 0) 
      {
       std::cout << "Error reading file " << img_name << std::endl;
       return;
      }
      //if(img.channels()!=1)
      //continue;
      cv::Size size(640,480);
      cv::resize(img,img,size);

      _depthSystem.extractDepthFeatures(img);
      _depthFeatureVector=_depthSystem.getDepthFeatureVector();
      
      std::cout<<"depthFeatureVector of image "<<img_name<<" "<<_depthFeatureVector.size()<<std::endl;
     
      for (int j=0; j<_depthFeatureVector.size(); j++)
      {
        training_mat.at<double>(i,j)=_depthFeatureVector[j];
      }
    }
    
    
    if (!fs.isOpened())
    {
      fs.open("/home/marios/depthtraining.yml", cv::FileStorage::WRITE);
    }
    
      fs << "training_mat" << training_mat;
      fs.release();
    
    std::cout << "training matrix = " << std::endl << " " << training_mat.size()<< std::endl;
}

}// namespace pandora_vision

int main()
{
  pandora_vision::SvmTraining victim_trainer;
  return 0;
}
