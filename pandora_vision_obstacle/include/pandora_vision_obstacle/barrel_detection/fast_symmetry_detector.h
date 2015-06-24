/**
 Based on algorithm from:
 1. Real Time Object Tracking using Reflectional Symmetry and Motion
 Wai Ho Li and Lindsay Kleeman
 Intelligent Robotics Research Centre Department of Electrical and Computer Systems Engineering Monash University, Clayton, Victoria 3800, Australia
 { Wai.Li, Lindsay.Kleeman } @eng.monash.edu.au
 
 2. Fast Global Reflectional Symmetry Detection for Robotic Grasping and Visual Tracking
 Wai Ho Li, Alan M. Zhang and Lindsay Kleeman
 Centre for Perceptive and Intelligent Machines in Complex Environments: Intelligent Robotics
 Monash University, Clayton
 Melbourne, Australia
 {Wai.Li, Alan.Zhang, Lindsay.Kleeman}@eng.monash.edu.au
 
**/

//
//  FastSymmetryDetector.h
//  FSD
//
//  Created by Saburo Okita on 02/05/14.
//  Copyright (c) 2014 Saburo Okita. All rights reserved.
//


#ifndef __FSD__FastSymmetryDetector__
#define __FSD__FastSymmetryDetector__

#include <iostream>
#include <opencv2/opencv.hpp>

class FastSymmetryDetector {
public:
    FastSymmetryDetector(const cv::Size imageSize, const cv::Size houghSize, const int rotResolution = 1);
    void vote(cv::Mat& image, int minPairDist, int maxPairDist);
    inline void rotateEdges(std::vector<cv::Point2f>& edges, int theta);
    
    cv::Mat getAccumulationMatrix(float thresh = 0.0);
    
    std::vector<std::pair<cv::Point, cv::Point> > getResult(int noOfPeaks, float threshold = -1.0f);
    std::pair<cv::Point, cv::Point> getLine(float rho, float theta);
    void getMaxDistance(float* maxDistance);
    void getYCoords(float* maxY, float* minY);
    
private:
    std::vector<cv::Mat> rotMatrices;
    cv::Mat rotEdges;
    std::vector<float*> reRows;
    cv::Mat accum;
    
    cv::Size imageSize;
    cv::Point2f center;
    float diagonal;
    int rhoDivision;
    int rhoMax;
    int thetaMax;
    float maxDistance;
    float maxY;
    float minY;
};

#endif /* defined(__FSD__FastSymmetryDetector__) */
