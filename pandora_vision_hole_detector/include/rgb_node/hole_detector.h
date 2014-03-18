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

#ifndef RGB_NODE_HOLE_DETECTOR_H 
#define RGB_NODE_HOLE_DETECTOR_H 

#include "rgb_node/rgb_constants.h"
#include "rgb_node/texture_filter.h"
#include "rgb_node/edge_detection.h"
#include "rgb_node/blob_extraction.h"
namespace pandora_vision
{
  class HoleDetector
  {
    //! Current frame after backprojection
    cv::Mat backprojectedFrame;
    //! Current frame after edge detection algorithm
    cv::Mat edgesFrame;
    
    //! Instance of class EdgeDetector, that finds edges in current frame
    EdgeDetector _edgeDetector;
    
    //! Instance of class TextureDetector, that applies texture in current 
    //! frame in order to isolate pixels of the image, where we have walls
    TextureDetector _textureDetector;
    
    //! Instance of class BlobDetector, that locates the centers of
    //! blobs in current image
    BlobDetector _blobDetector;
    
    //! Kyepoints of blobs in current frame
    std::vector<cv::KeyPoint> detectedkeyPoints;
    
    public:
    /**
     @brief Class constructor
    */ 
    HoleDetector();
    
    /**
     @brief Class destructor
    */ 
    virtual ~HoleDetector();
    
    /**
     @brief Function that locates the position of potentional holes
     in current frame.
     @param holeFrame [cv::Mat] current frame to be processed
     @return void
    */ 
    void findHoles(cv::Mat holeFrame);
  };
}// namespace pandora_vision
#endif  // RGB_NODE_HOLE_DETECTOR_H
