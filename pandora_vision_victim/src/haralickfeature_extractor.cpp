/*********************************************************************
*
* Software License Agreement (BSD License)
*
* Copyright (c) 2014, P.A.N.D.O.R.A. Team.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of the P.A.N.D.O.R.A. Team nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* Author: Victor Daropoulos
*********************************************************************/


#include "pandora_vision_victim/haralickfeature_extractor.h"

namespace pandora_vision
{
  //!<Constructor
  HaralickFeatures::HaralickFeatures()
  {
  }

  /**
    @brief Function for updating the values of the GLCM matrix.
    @param y1 [int] The y coordinate of the first pixel
    @param x1 [int] The x coordinate of the first pixel
    @param y2 [int] The y coordinate of the second pixel
    @param x2 [int] The x coordinate of the second pixel
    @param out [cv::Mat*] The updated GLCM matrix
    @param in [cv::Mat&] The GLCM matrix
    @return void
  **/  
  void HaralickFeatures::updateGLCM(int y1,int x1,int y2,int x2,cv::Mat* out,const cv::Mat& in)
  {
    if(y2<0 || y2>=in.rows || x2<0 || x2>=in.cols) return;
    
    out->at<float>(in.at<uchar>(y1,x1),in.at<uchar>(y2,x2))+=1.0;
    
  }

  /**
    @brief Function for calculating the normalized GLCM matrix, considering an
    horizontal relationship between pixels with offset equal to 1. The returned 
    matrix is symmetric with respect to the main diagonal.
    @param in [cv::Mat&] A grayscale image with 8 bit values.
    @return [cv::Mat] The GLCM matrix.
  **/
  cv::Mat HaralickFeatures::calculateGLCM(const cv::Mat& in)
  {
    cv::Mat out=cv::Mat::zeros(256,256,CV_32FC1);
    
    cv::Mat temp;
    
    int offset=1;
    
    for(int y=0;y<in.rows;y++)
    {
      for(int x=0;x<in.cols;x++)
      {
        updateGLCM(y,x,y+offset,x,&out,in);
      }
    }
    
    cv::transpose(out,temp);
    
    out=out+temp;  
    
    normalizeGLCM(&out);
    
    return out;
  }

  /**
    @brief Function for normalizing the values of the GLCM matrix.
    @param in [cv::Mat*] The matrix to be normalized
    @return [cv::Mat] The normalized matrix
  **/  
  cv::Mat HaralickFeatures::normalizeGLCM(cv::Mat* in)
  {
    cv::Scalar totalSum=cv::sum(*in);
    
    for(int y=0;y<in->rows;y++)
    {
      for(int x=0;x<in->cols;x++)
      {
        in->at<float>(y,x)=(float)(in->at<float>(y,x)/(float)totalSum[0]);
      }
    }
    return *in;
  }

  /**
    @brief Function for calculating the Angular Second Moment,
    http://murphylab.web.cmu.edu/publications/boland/boland_node26.html  
    @param in [cv::Mat&] The normalized GLCM matrix
    @return void
  **/    
  void HaralickFeatures::getAngularSecondMoment(const cv::Mat& in)
  {
    float sum=0;
    
    for(int y=0;y<in.rows;y++)
    {
      for(int x=0;x<in.cols;x++)
      {
        sum+=in.at<float>(y,x)*in.at<float>(y,x);
      }
    }
    
    _haralickFeatures.push_back(sum);
  }


  /**
    @brief Function for calculating entropy,
    http://murphylab.web.cmu.edu/publications/boland/boland_node26.html  
    @param in [cv::Mat&] The normalized GLCM matrix
    @return void
  **/      
  void HaralickFeatures::getEntropy(const cv::Mat& in)
  {
    float sum=0;
    
    for(int y=0;y<in.rows;y++)
    {
      for(int x=0;x<in.cols;x++)
      {
        if(in.at<float>(y,x)!=0)
        {
          sum+=in.at<float>(y,x)*log(in.at<float>(y,x));
        }
      }
    }
    
    sum=-sum;
    
    _haralickFeatures.push_back(sum);
  }

  /**
    @brief Function for calculating contrast,
    http://murphylab.web.cmu.edu/publications/boland/boland_node26.html  
    @param in [cv::Mat&] The normalized GLCM matrix
    @return void
  **/      
  void HaralickFeatures::getContrast(const cv::Mat& in)
  {
    float sum=0;
    
    for(int y=0;y<in.rows;y++)
    {
      for(int x=0;x<in.cols;x++)
      {
        sum+=pow((float)(y-x),2.0)*in.at<float>(y,x);
      }
    }
    
    _haralickFeatures.push_back(sum);
  }

  /**
    @brief Function for calculating variance,
    http://murphylab.web.cmu.edu/publications/boland/boland_node26.html  
    @param in [cv::Mat&] The normalized GLCM matrix
    @return void
  **/     
  void HaralickFeatures::getVariance(const cv::Mat& in)
  {
    float w_mean=0;
    
    for(int y=0;y<in.rows;y++)
    {
      for(int x=0;x<in.cols;x++)
      {
        w_mean+=y*in.at<float>(y,x);
      }
    }
    
    float variance=0;
    
    for(int y=0;y<in.rows;y++)
    {
      for(int x=0;x<in.cols;x++)
      {
        variance+=pow((float)(y-w_mean),2.0)*in.at<float>(y,x);
      }
    }
    
    _haralickFeatures.push_back(variance);
  }


  /**
    @brief Function for calculating correlation,
    http://murphylab.web.cmu.edu/publications/boland/boland_node26.html  
    @param in [cv::Mat&] The normalized GLCM matrix
    @return void
  **/      
  void HaralickFeatures::getCorrelation(const cv::Mat& in)
  {
    float mean=0;
    float std=0;
    float corr=0;
    
    for(int y=0;y<in.rows;y++)
    {
      for(int x=0;x<in.cols;x++)
      {
        mean+=y*in.at<float>(y,x);
      }
    }
    
    for(int y=0;y<in.rows;y++)
    {
      for(int x=0;x<in.cols;x++)
      {
        std+=pow((float)y-mean,2.0)*in.at<float>(y,x);
      }
    }
    
    std=sqrt(std);
    
    for(int y=0;y<in.rows;y++)
    {
      for(int x=0;x<in.cols;x++)
      {
        corr+=(in.at<float>(y,x)*(y-mean)*(x-mean))/pow(std,2.0);
      }
    }
    
    _haralickFeatures.push_back(corr);
  } 

  /**
    @brief Function for calculating homogeneity,
    http://murphylab.web.cmu.edu/publications/boland/boland_node26.html  
    @param in [cv::Mat&] The normalized GLCM matrix
    @return void
  **/     
  void HaralickFeatures::getHomogeneity(const cv::Mat& in)
  {
    float temp=0;
    
    for(int y=0;y<in.rows;y++)
    {
      for(int x=0;x<in.cols;x++)
      {
        temp+=(1./(1.+pow(float(y-x),2)))*in.at<float>(y,x);
      }
    }
    
    _haralickFeatures.push_back(temp);
  }    

  /**
    @brief Function for calculating sum average,
    http://murphylab.web.cmu.edu/publications/boland/boland_node26.html  
    @param in [cv::Mat&] The normalized GLCM matrix
    @return void
  **/    
  void HaralickFeatures::getSumAverage(const cv::Mat& in)
  {
    cv::Mat temp=cv::Mat::zeros(1,2*(in.rows-1)+1,CV_32FC1);
    
    for(int y=0;y<in.rows;y++)
    {
      for(int x=0;x<in.cols;x++)
      {
        temp.at<float>(x+y)+=in.at<float>(y,x);
      }
    }
    
    for(int i=0;i<temp.cols;i++)
    {
      temp.at<float>(i)=i*temp.at<float>(i);
    }
    
    cv::Scalar totalSum=cv::sum(temp);
    
    _haralickFeatures.push_back(totalSum[0]); 
    
    
  }  

  /**
    @brief Function for calculating sum variance,
    http://murphylab.web.cmu.edu/publications/boland/boland_node26.html  
    @param in [cv::Mat&] The normalized GLCM matrix
    @return void
  **/  
  void HaralickFeatures::getSumVariance(const cv::Mat& in)
  {
    cv::Mat temp=cv::Mat::zeros(1,2*(in.rows-1)+1,CV_32FC1);
    
    for(int y=0;y<in.rows;y++)
    {
      for(int x=0;x<in.cols;x++)
      {
        temp.at<float>(x+y)+=in.at<float>(y,x);
      }
    }
    
    float f7=_haralickFeatures[6];
    
    float sum=0;
    
    for(int i=0;i<temp.cols;i++)
    {
      sum+=pow(float(i)-f7,2.0)*temp.at<float>(i);
    }
    
    _haralickFeatures.push_back(sum);
  }

  /**
    @brief Function for calculating sum entropy,
    http://murphylab.web.cmu.edu/publications/boland/boland_node26.html  
    @param in [cv::Mat&] The normalized GLCM matrix
    @return void
  **/  
  void HaralickFeatures::getSumEntropy(const cv::Mat& in)
  {
    cv::Mat temp=cv::Mat::zeros(1,2*(in.rows-1)+1,CV_32FC1);
    
    for(int y=0;y<in.rows;y++)
    {
      for(int x=0;x<in.cols;x++)
      {
        temp.at<float>(x+y)+=in.at<float>(y,x);
      }
    }
    
    float sum=0;
    
    for(int i=0;i<temp.cols;i++)
    {
      if(temp.at<float>(i)!=0)
      {
        sum+=temp.at<float>(i)*log(temp.at<float>(i));
      }
    }
    
    sum=-sum;
    
    _haralickFeatures.push_back(sum);
  }

  /**
    @brief Function for calculating difference variance,
    http://murphylab.web.cmu.edu/publications/boland/boland_node26.html  
    @param in [cv::Mat&] The normalized GLCM matrix
    @return void
  **/   
  void HaralickFeatures::getDifferenceVariance(const cv::Mat& in)
  {
    cv::Mat temp=cv::Mat::zeros(1,in.rows,CV_32FC1);
    
    for(int y=0;y<in.rows;y++)
    {
      for(int x=0;x<in.cols;x++)
      {
        temp.at<float>(abs(y-x))+=in.at<float>(y,x);
      }
    }
    
    float f10_=0;
    
    for(int i=0;i<temp.cols;i++)
    {
      f10_+=i*temp.at<float>(i);
    }
    
    float f10=0;
    
    for(int i=0;i<temp.cols;i++)
    {
      f10+=pow((float)(i-f10_),2.0)*temp.at<float>(i);
    }
    
    _haralickFeatures.push_back(f10);
  }

  /**
    @brief Function for calculating difference entropy,
    http://murphylab.web.cmu.edu/publications/boland/boland_node26.html  
    @param in [cv::Mat&] The normalized GLCM matrix
    @return void
  **/   
  void HaralickFeatures::getDifferenceEntropy(const cv::Mat& in)
  {
    cv::Mat temp=cv::Mat::zeros(1,in.rows,CV_32FC1);
    
    for(int y=0;y<in.rows;y++)
    {
      for(int x=0;x<in.cols;x++)
      {
        temp.at<float>(abs(y-x))+=in.at<float>(y,x);
      }
    }
    
    float sum=0;
    
    for(int i=0;i<temp.cols;i++)
    {
      if(temp.at<float>(i)!=0)
      {
        sum+=temp.at<float>(i)*log(temp.at<float>(i));
      }
    }
    
    sum=-sum;
    
    _haralickFeatures.push_back(sum);
  }

  /**
    @brief Function for calculating Info Measure of Correlation 1 and 2,
    http://murphylab.web.cmu.edu/publications/boland/boland_node26.html  
    @param in [cv::Mat&] The normalized GLCM matrix
    @return void
  **/          
  void HaralickFeatures::getInfoMeasuresCorr(const cv::Mat& in)
  {
    cv::Mat px=cv::Mat::zeros(1,in.rows,CV_32FC1);
    
    cv::Mat py=cv::Mat::zeros(1,in.rows,CV_32FC1);
    
    for(int y=0;y<in.rows;y++)
    {
      for(int x=0;x<in.cols;x++)
      {
        px.at<float>(y)+=in.at<float>(y,x);
        py.at<float>(y)+=in.at<float>(x,y);
      }
    }
    
    float HX=0;
    float HY=0;
    
    for(int i=0;i<px.cols;i++)
    {
      if(px.at<float>(i)!=0)
      {
        HX+=px.at<float>(i)*log(px.at<float>(i));
      }
      if(py.at<float>(i)!=0)
      {
        HY+=py.at<float>(i)*log(py.at<float>(i));
      }
    }
    
    HX=-HX;
    HY=-HY;
    
    
    float HXY1=0;
    float HXY2=0;
    float HXY=0;
    
    for(int y=0;y<in.rows;y++)
    {
      for(int x=0;x<in.cols;x++)
      {
        if((px.at<float>(y)*py.at<float>(x))!=0)
        {
          HXY1+=in.at<float>(y,x)*log(px.at<float>(y)*py.at<float>(x));
          HXY2+=px.at<float>(y)*py.at<float>(x)*log(px.at<float>(y)*py.at<float>(x));
        }
        if(in.at<float>(y,x)!=0)
        {
          HXY+=in.at<float>(y,x)*log(in.at<float>(y,x));
        }
      }
    }
    
    HXY1=-HXY1;
    HXY2=-HXY2;
    HXY=-HXY;
    
    float f12=(HXY-HXY1)/std::max(HX,HY);
    
    _haralickFeatures.push_back(f12);
    
    float f13=sqrt(1-exp(-2*(HXY2-HXY)));
    
    _haralickFeatures.push_back(f13);
    
  }


  /**
    @brief Function returning the haralick feature vector,
    http://murphylab.web.cmu.edu/publications/boland/boland_node26.html  
    @return std::vector [float] The vector containing the haralick features
  **/     
  std::vector<float> HaralickFeatures::getFeatures()
  {
    return _haralickFeatures;
  }

  int main(int argc,char* argv[])
  {
    cv::Mat image=cv::imread(argv[1]);
    
    cv::Mat temp=cv::Mat(image.rows,image.cols,CV_8UC1);
    
    cv::cvtColor(image,temp,CV_BGR2GRAY);
   
    HaralickFeatures* features=new HaralickFeatures();
    
    //cv::Mat temp=cv::Mat::zeros(4,4,CV_8UC1);
    
    std::vector<float> myvec;
    
    /*
    
    temp.at<uchar>(0,0)=0;
    temp.at<uchar>(0,1)=0;
    temp.at<uchar>(0,2)=1;
    temp.at<uchar>(0,3)=1;
    temp.at<uchar>(1,0)=0;
    temp.at<uchar>(1,1)=0;
    temp.at<uchar>(1,2)=1;
    temp.at<uchar>(1,3)=1;
    temp.at<uchar>(2,0)=0;
    temp.at<uchar>(2,1)=2;
    temp.at<uchar>(2,2)=2;
    temp.at<uchar>(2,3)=2;
    temp.at<uchar>(3,0)=2;
    temp.at<uchar>(3,1)=2;
    temp.at<uchar>(3,2)=3;
    temp.at<uchar>(3,3)=3;
    
    */
    
    cv::Mat out=features->calculateGLCM(temp);
    
    features->getAngularSecondMoment(out);
    
    features->getContrast(out);
    
    features->getEntropy(out);
    
    features->getVariance(out);
    
    features->getCorrelation(out);
    
    features->getHomogeneity(out);
    
    features->getSumAverage(out);
    
    features->getSumVariance(out);
    
    features->getSumEntropy(out);
    
    features->getDifferenceVariance(out);
    
    features->getDifferenceEntropy(out);
    
    features->getInfoMeasuresCorr(out);
    
    myvec=features->getFeatures();
    
    for(int i=0;i<myvec.size();i++)
    {
      std::cout<<myvec.at(i)<<std::endl;
    }
    
    return 0;
    
  }
}    
  

