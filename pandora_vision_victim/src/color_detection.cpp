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
* Author: Marios Prwtopapas
*********************************************************************/

#include "pandora_vision_victim/color_detection.h"

namespace pandora_vision
{
  /**
    @brief Constructor
  **/
  ColorDetection::ColorDetection()
  {
   
   ROS_INFO("[victim_node] : Created color detection instance"); 
  }
  
  /**
    @brief Destructor
  */
   ColorDetection::~ColorDetection()
  {
    ROS_INFO("[victim_node] : Destroying Color Detection instance");
  }
  
 void ColorDetection::findColorFeatures()
{
  cv::Mat src;
  cv::Mat hsv;
  /// Read the image
  src = cv::imread("positive142.jpg", 1 );
  cv::imshow("Source Image",src);
  /// Transform it to HSV
  cvtColor( src, hsv, CV_BGR2HSV );
  
  /// Separate the image in 3 places (H,S,V) one for each channel
  std::vector<cv::Mat> hsv_planes;
  split( hsv, hsv_planes );

	
  /// Establish the number of bins

	
	int h_bins = 180; 
	int s_bins = 256;
	int v_bins = 256;
	
	
	
	/// Set the ranges ( for H,S,V) 
	float h_ranges[] = { 0, 180 };
	float s_ranges[] = { 0, 256 };
	float v_ranges[] = { 0, 256 };
	
	const float* h_histRange = { h_ranges };
    const float* s_histRange = { s_ranges };
	const float* v_histRange = { v_ranges };

	bool uniform = true; 
	bool accumulate = false;

  /// Creation of 3 Mat objects to save each histogramm
  cv::Mat h_hist, s_hist, v_hist;

  h_hist=computeHist(hsv_planes[0],h_bins,h_histRange);
  s_hist=computeHist(hsv_planes[1],s_bins,s_histRange);
  v_hist=computeHist(hsv_planes[2],v_bins,v_histRange);
  
   /// Compute mean and std value of every color component
  std::vector<double> meanStdHSV(6);
  meanStdHSV=computeMeanStdHSV(hsv_planes);

  std::cout<<"Mean and Standard Deviation of HSV"<<std::endl;
  for (int ii=0; ii<meanStdHSV.size(); ii++)
		std::cout<<meanStdHSV[ii]<<std::endl;
		
  /// Find the dominant color component and their density values
  std::vector<double>dominantVal(6);
  findDominantColor(src, h_hist, h_bins, dominantVal[0], dominantVal[1] );
  findDominantColor(src, s_hist, s_bins, dominantVal[2], dominantVal[3] );
  findDominantColor(src, v_hist, v_bins, dominantVal[4], dominantVal[5] );
   
  std::cout<<"Dominant values and Densities of every color component HSV"<<std::endl;
  for (int ii=0; ii<dominantVal.size(); ii++)
		std::cout<<dominantVal[ii]<<std::endl;
   
  ///Compute the modules of first 6 components of a Fourier transform of the image components H(hue) and S(saturation).
   
   std::vector<double>huedft(6);
   std::vector<double>satdft(6);
   huedft=computeDFT(hsv_planes[0]);
   satdft=computeDFT(hsv_planes[1]);
   
   std::cout<<"6 first Dft of Hue"<<std::endl;
     for (int ii=0; ii<huedft.size(); ii++)
		std::cout<<huedft[ii]<<std::endl;
   std::cout<<"6 first Dft of Sat"<<std::endl;
     for (int ii=0; ii<satdft.size(); ii++)
		std::cout<<satdft[ii]<<std::endl;
  
///compute the colour angles of rgb color components
std::vector<double>ColorAnglesAndStd=computeColorAngles(src);
std::cout<<"Color Angles and normalized intensity std"<<std::endl;
for (int ii=0; ii<ColorAnglesAndStd.size(); ii++)
std::cout<<ColorAnglesAndStd[ii]<<std::endl;
 

}

  
  cv::Mat ColorDetection::computeHist(cv::Mat planes,int histSize,const float* histRange)
{
	  bool uniform = true; 
	  bool accumulate = false;
	  cv::Mat hist;
	  /// Compute the histograms:	
	  cv::calcHist( &planes, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange, uniform, accumulate );
	  /// Normalize the result to [0, 255]
	  //cv::normalize(hist, hist, 0, 255, cv::NORM_MINMAX, -1, cv::Mat() );
	  return hist;

}

 std::vector<double> ColorDetection::computeMeanStdHSV( std::vector<cv::Mat> hsv_planes )
{
	   std::vector<double> meanStdHSV(6); 
	   cv::Scalar avg,st;
	   cv::meanStdDev(hsv_planes[0], avg, st);
	   meanStdHSV[0]=avg.val[0];
	   meanStdHSV[1]=st.val[0];
	   cv::meanStdDev(hsv_planes[1], avg, st);
	   meanStdHSV[2]=avg.val[0];
	   meanStdHSV[3]=st.val[0];
	   cv::meanStdDev(hsv_planes[2], avg, st);
	   meanStdHSV[4]=avg.val[0];
	   meanStdHSV[5]=st.val[0];
       return meanStdHSV;
}    

void ColorDetection::findDominantColor(cv::Mat img,cv::Mat hist,int bins,double& value,double& density )
{

	double maxVal = 0;
    double val = 0;
    

             for( int ii= 0; ii < bins; ii++ )
                {
                      double binVal = (double)hist.at<float>(ii);
                      if(binVal > maxVal)
                      {
                          maxVal = binVal;
                          val = ii;
                      }
                }
                
            
    
    value=val;
    density=maxVal/(img.rows*img.cols);
                
}

std::vector<double> ColorDetection::computeDFT(cv::Mat img)
{
	std::vector<double>temp(6);
	cv::Mat padded;                            //expand input image to optimal size
    int rows = cv::getOptimalDFTSize( img.rows );
    int cols = cv::getOptimalDFTSize( img.cols ); // on the border add zero values
    copyMakeBorder(img, padded, 0, rows - img.rows, 0, cols - img.cols, cv::BORDER_CONSTANT, cv::Scalar::all(0));
     cv::Mat planes[] = {cv::Mat_<float>(padded), cv::Mat::zeros(padded.size(), CV_32F)};
    cv::Mat complexI;
    merge(planes, 2, complexI);         // Add to the expanded another plane with zeros

    dft(complexI, complexI);            // this way the result may fit in the source matrix

    // compute the magnitude 
    split(complexI, planes);                   // planes[0] = Re(DFT(I), planes[1] = Im(DFT(I))
    magnitude(planes[0], planes[1], planes[0]);// planes[0] = magnitude
    cv::Mat magI = planes[0];
    temp[0]=(double)magI.at<float>(0,0);
    temp[1]=(double)magI.at<float>(0,1);
    temp[2]=(double)magI.at<float>(1,0);
    temp[3]=(double)magI.at<float>(0,2);
    temp[4]=(double)magI.at<float>(1,1);
    temp[5]=(double)magI.at<float>(2,0);
    return temp;
    
}

std::vector<double> ColorDetection::computeColorAngles(cv::Mat img)
{
	/// Separate the image in 3 places (R,G,B) one for each channel
  std::vector<cv::Mat> rgb_planes;
  split( img, rgb_planes );
  /// Compute the average pixel value of each r,g,b color component
  cv::Scalar bmean=mean(rgb_planes[0]);
  cv::Scalar gmean=mean(rgb_planes[1]);
  cv::Scalar rmean=mean(rgb_planes[2]);
  

  ///obtain zero-mean colour vectors r0, g0 and b0 by subtracting the corresponding average pixel value of each original colour vector
  cv::Mat r0=cv::Mat::zeros(img.rows,img.cols,CV_64FC1);
  cv::Mat b0=cv::Mat::zeros(img.rows,img.cols,CV_64FC1);
  cv::Mat g0=cv::Mat::zeros(img.rows,img.cols,CV_64FC1);
  for (int ii=0; ii<img.rows; ii++)
	  for (int jj=0; jj<img.cols; jj++)
	  b0.at<double>(ii,jj)=rgb_planes[0].at<uchar>(ii,jj)-bmean.val[0];
for (int ii=0; ii<img.rows; ii++)
	  for (int jj=0; jj<img.cols; jj++)
	  g0.at<double>(ii,jj)=rgb_planes[1].at<uchar>(ii,jj)-gmean.val[0];
for (int ii=0; ii<img.rows; ii++)
	  for (int jj=0; jj<img.cols; jj++)
	  r0.at<double>(ii,jj)=rgb_planes[2].at<uchar>(ii,jj)-rmean.val[0];


    double rgdot=r0.dot(g0);
    double gbdot=g0.dot(b0);
    double rbdot=r0.dot(b0);
    double rsum=0,bsum=0,gsum=0;
    
 //Compute the dot product of rgb color components
    for (int ii=0; ii<r0.rows; ii++)
		for(int jj=0; jj<r0.cols; jj++)
			{
			   rsum+=pow(r0.at<double>(ii,jj),2);
			   gsum+=pow(g0.at<double>(ii,jj),2);
			   bsum+=pow(b0.at<double>(ii,jj),2);
			}
	   
	
	double rlength=sqrt(rsum);
    double glength=sqrt(gsum);
	double blength=sqrt(bsum); 
    rgdot/=(rlength*glength);
    gbdot/=(glength*blength);
    rbdot/=(rlength*blength);


  //compute the color angles
    double rgAngle=acos(rgdot);
    double gbAngle=acos(gbdot);
    double rbAngle=acos(rbdot);
   
  ///normalised intensity standard deviation
   /// Transform the src image to grayscale
  cvtColor( img, img, CV_BGR2GRAY );
  //compute the mean intensity value
  cv::Scalar meanI=mean(img);
    //std::cout<<"meanI "<<meanI<<std::endl;
  // find the maximum intensity value
  double maxVal,std,sum=0;
  minMaxLoc( img,NULL, &maxVal );
      
  for(int ii=0; ii<img.rows; ii++)
	for(int jj=0; jj<img.cols; jj++)
	{
		sum+=pow((img.at<uchar>(ii,jj)-meanI.val[0]),2);
	}
  std=2.0/(maxVal*img.cols*img.rows)*sqrt(sum);
 
  ///construct the final feature vector
   std::vector<double> ColorAnglesAndStd(4);
   ColorAnglesAndStd[0]=rgAngle;
   ColorAnglesAndStd[1]=gbAngle;
   ColorAnglesAndStd[2]=rbAngle;
   ColorAnglesAndStd[3]=std;
   return ColorAnglesAndStd;
  
 }

}
