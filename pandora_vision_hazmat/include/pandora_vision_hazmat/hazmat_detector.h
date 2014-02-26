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
* Authors:  Tsakalis Vasilis, Despoina Paschalidou
*********************************************************************/

#ifndef HAZMATEPSILONDETECTOR_H
#define HAZMATEPSILONDETECTOR_H

#include "sift.h"
#include "kdtree.h"
#include "xform.h"

struct HazmatEpsilon //the struct in which the results are stored
{ 
  int pattern_num;
  float x,y;
  int m;
  float MO;
  int votes;
  CvMat* H;
};

class HazmatEpsilonDetector
{
    
  private:
  
    //the maximum number of keypoint NN candidates to check during BBF search
    int KDTREE_BBF_MAX_NN_CHKS; 
    
    //threshold on squared ratio of distances between NN and 2nd NN
    float NN_SQ_DIST_RATIO_THR; 
    
    std::string patternIndexPath;
    
    int nPatterns;
    
    int* nFeats;
    
    int votes;
    
    float votingThreshold;
    
    float scale;
    
    float area;
    
    float minAreaThreshold;
    
    float maxAreaThreshold;
    
    float MOThreshold;
    
    struct feature** feats;
    
    struct kd_node** trees;
    
    std::string param_path;
    
    int sideLength,colorVariance,featureThreshold;
    
    int frameNum;
  
  
  public:
    CvPoint2D64f** upperPoints;
    CvPoint2D64f** lowerPoints;
    
    HazmatEpsilonDetector(std::string package_path); //our constructor
    
    ~HazmatEpsilonDetector(); //our destructor
    
    std::vector<CvPoint2D64f> center_of;
    
    void calcHistograms();
    
    //shows how many patterns were found in the screenshot
    int patterns_found_counter; 
    
    // Initialize class parameters. Can be tweaked to exchange \
    quality for speed.
    void setParameters(); 
    
    // Initialize hazmat detector. Loads hazmats from hard disk into memory.
    void initDetector(); 
    
    // Reads contents from file "contents" and stores into memory \
    the processed hazmats
    void preprocessHazmat(); 
    
    // The core of the hazmat detector. Detects hazmats in the screenshot
    std::vector<HazmatEpsilon> detectHazmat(cv::Mat hazmatFrame); 
    
    // The core of the Epsilon detector. NOT IMPLEMENTED
    int detectEpsilon(char* screenshot); 
    
    //our basic function called by programs
    int HazmatEpsilonDetect(); 
    
    // the fully functional detector of both hazmat and epsilon patterns
    std::vector<HazmatEpsilon> DetectHazmatEpsilon(cv::Mat img); 
    
    //upologizw embado tetrapleurou apo tis tesseris korufes tou
    float calculateRectangleArea(
      CvPoint2D64f pt1,CvPoint2D64f pt2, CvPoint2D64f pt3,CvPoint2D64f pt4); 
    
    void setHazmatParameters(int clrVariance,float votingThr,float minAreaThr,
      float maxAreaThr,int sideLgth,int featThr,float MOThr);
    
    std::vector <int> findFeature(int &m,int n,int testNum, 
      struct kd_node* kd_root);
    
    void calculateArea(CvMat* H,cv::Mat pattern_image);
    
    CvPoint2D64f defineVariance(float& SAD,float& SAD2,
      IplImage* img,CvMat* H,cv::Mat _pattern_image,int n);  
    
    cv::MatND patternHistog;
    
    void calcMinMax();
    
    float** minUV;
    
    float**maxUV;
    
    int rows,cols; 

    
};




#endif
