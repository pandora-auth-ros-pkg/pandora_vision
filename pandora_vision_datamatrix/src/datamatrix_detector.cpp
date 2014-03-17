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

#include "pandora_vision_datamatrix/datamatrix_detector.h"

namespace pandora_vision 
{
  
  /**
   *@brief Constructor
  **/
  DatamatrixDetector::DatamatrixDetector()
  {
    img = NULL;
    dec = NULL;
    reg = NULL;
    msg = NULL;
    
    datamatrix_qode.message = "";
    ROS_INFO("[Datamatrix_node] : Datamatrix_Detector instance created");
  }
  

  /**
    @brief Destructor
   */
  DatamatrixDetector::~DatamatrixDetector()
  {
    //!< Deallocate memory
    dmtxMessageDestroy(&msg);
    dmtxDecodeDestroy(&dec);
    dmtxImageDestroy(&img);  
    dmtxRegionDestroy(&reg);
    ROS_INFO("[Datamatrix_node] : Datamatrix_Detector instance destroyed");
  }
  
  
  /**
    @brief Detects datamatrixes and stores them in a vector. 
    @param frame [cv::Mat] The image in which the QRs are detected
    @return void
   */
  void DatamatrixDetector::detect_datamatrix(cv::Mat image)
  {
    img = NULL;
    dec = NULL;
    reg = NULL;
    msg = NULL;
    //!< creates and initializes a new DmtxImage structure using pixel 
    //!< data provided  by  the calling application. 
    img = dmtxImageCreate(image.data, image.cols, image.rows, 
        DmtxPack24bppBGR);
    ROS_ASSERT(img != NULL);    
    
    //!< creates  and  initializes a new DmtxDecode struct, which 
    //!< designates the image to be scanned and initializes the scan 
    //!< grid pattern.    
    dec = dmtxDecodeCreate(img, 1);
    ROS_ASSERT(dec != NULL);
    
    //!< searches  every  pixel location in a grid pattern looking 
    //!< for potential barcode regions. A DmtxRegion is returned 
    //!< whenever a potential  barcode region  is found, or if the final 
    //!< pixel location has been scanned.
    reg = dmtxRegionFindNext(dec, NULL);
    if(reg != NULL) 
    {
      msg = dmtxDecodeMatrixRegion(dec, reg, DmtxUndefined);
      if(msg != NULL) 
      {
        std::cout << msg->output <<std::endl;
        datamatrix_qode.message.assign((const char*) msg->output, msg->outputIdx);
        //!< Find datamatrixe's center exact position
        locate_datamatrix(image);
      }
     
    }

  }
  
  /**
    @brief Function that finds the position of datamatrixe's center
    @param frame [cv::Mat] The image in which the QRs are detected
    @return void
   */
  void DatamatrixDetector::locate_datamatrix(cv::Mat image)
  {
    DmtxVector2 p00, p10, p11, p01;
    std::vector<cv::Point2f> datamatrixVector;
    
    p00.X = p00.Y = p01.X = p10.Y = 0.0;
    p01.Y = p10.X = p11.X = p11.Y = 1.0;
		
    dmtxMatrix3VMultiplyBy(&p00, reg->fit2raw);
    dmtxMatrix3VMultiplyBy(&p10, reg->fit2raw);
    dmtxMatrix3VMultiplyBy(&p11, reg->fit2raw);
    dmtxMatrix3VMultiplyBy(&p01, reg->fit2raw);
		
    cv::Point2f corner1(p00.X, image.rows - p00.Y);
    cv::Point2f corner2(p10.X, image.rows - p10.Y);
    cv::Point2f corner3(p11.X, image.rows - p11.Y);
    cv::Point2f corner4(p01.X, image.rows - p01.Y);
    
    datamatrixVector.push_back(corner1);
    datamatrixVector.push_back(corner2);
    datamatrixVector.push_back(corner4);
    datamatrixVector.push_back(corner3);
    
    cv::line(image, corner1, corner2, cv::Scalar(0, 255, 0), 3);
    cv::line(image, corner2, corner3, cv::Scalar(255, 0, 0), 3);
    cv::line(image, corner3, corner4, cv::Scalar(0, 0, 255), 3);
    cv::line(image, corner4, corner1, cv::Scalar(255, 255, 0), 3);
    
    
    cv::imshow("Image window", image);
    cv::waitKey(30);

  }
}// namespace pandora_vision
