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
    
    ROS_INFO("[Datamatrix_node] : Datamatrix_Detector instance created");
  }
  

  /**
    @brief Destructor
   */
  DatamatrixDetector::~DatamatrixDetector()
  {
    ROS_INFO("[Datamatrix_node] : Datamatrix_Detector instance destroyed");
  }
  
  
  /**
    @brief Detects datamatrixes and stores them in a vector. 
    @param frame [cv::Mat] The image in which the QRs are detected
    @return void
   */
  void DatamatrixDetector::detect_datamatrix(cv::Mat image)
  {
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
      }
    }  
  
  }
}// namespace pandora_vision
