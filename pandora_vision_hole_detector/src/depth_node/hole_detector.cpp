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
* Authors: Alexandros Filotheou, Manos Tsardoulias
*********************************************************************/

#include "depth_node/hole_detector.h"

namespace pandora_vision
{
  /**
   @brief The HoleDetector constructor
   **/
  HoleDetector::HoleDetector(void) {}



  /**
    @brief Finds the holes provided a depth image in CV_32FC1 format
    @param[in] depthImage [const cv::Mat&] The depth image in CV_32FC1
    format
    @param[out] interpolatedDepthImage [cv::Mat*] The denoised
    depth image in CV_32FC1 format
    @return std::vector<cv::Point2f> Centers of the possible holes
   **/
  HoleFilters::HolesConveyor HoleDetector::findHoles(const cv::Mat& depthImage,
    cv::Mat* interpolatedDepthImage)
  {
    #ifdef DEBUG_TIME
    Timer::start("findHoles", "inputCloudCallback");
    #endif
    #ifdef DEBUG_SHOW
    std::vector<cv::Mat> imgs;
    std::vector<std::string> msgs;
    if(DepthParameters::debug_show_find_holes) // Debug
    {
      std::string msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
      msg += " : Initial Depth";
      msgs.push_back(msg);
      cv::Mat tmp = Visualization::scaleImageForVisualization(depthImage, 0);
      imgs.push_back(tmp);
    }
    #endif

    //!< Perform noise elimination (black pixels removed)
    NoiseElimination::performNoiseElimination(depthImage,
        interpolatedDepthImage);

    #ifdef DEBUG_SHOW
    if(DepthParameters::debug_show_find_holes) // Debug
    {
      std::string msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
      msg += " : Interpolated depth image";
      msgs.push_back(msg);
      cv::Mat tmp = Visualization::scaleImageForVisualization(
        *interpolatedDepthImage, 0);
      imgs.push_back(tmp);
    }
    #endif

    //!< Edge computation
    cv::Mat denoisedDepthImageEdges;
    EdgeDetection::computeEdges(*interpolatedDepthImage,
        &denoisedDepthImageEdges);

    #ifdef DEBUG_SHOW
    if(DepthParameters::debug_show_find_holes) // Debug
    {
      std::string msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
      msg += STR(" : Edges after denoise");
      msgs.push_back(msg);
      cv::Mat tmp;
      denoisedDepthImageEdges.copyTo(tmp);
      imgs.push_back(tmp);
    }
    #endif

    //!< Find blobs in the edges image. Each blob is represented as
    //!< a keypoint which is the center of the blob found
    std::vector<cv::KeyPoint> keyPoints;
    BlobDetection::detectBlobs(denoisedDepthImageEdges, &keyPoints);

    #ifdef DEBUG_SHOW
    if(DepthParameters::debug_show_find_holes) // Debug
    {
      std::string msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
      msg += STR(" : Initial keypoints");
      msgs.push_back(msg);
      imgs.push_back(
        Visualization::showKeypoints(msg, denoisedDepthImageEdges, -1,
          keyPoints)
      );
    }
    #endif

    //!< The final vectors of keypoints, rectangles and blobs' outlines.
    struct HoleFilters::HolesConveyor conveyor;

    /**
      Get me blobs that their center point is inside the image,
      their bounding box is also inside the image, and their area is
      greater than DepthParameters::bounding_box_min_area_threshold.
      Each keypoint is associated with exactly one rectangle.
      The end product here is a set of keypoints, a set of rectangles that
      enclose them and a set of the outlines of the blobs found, all tightly
      packed in the conveyor object.
     **/
    HoleFilters::validateBlobs(
      keyPoints,
      &denoisedDepthImageEdges,
      DepthParameters::bounding_box_detection_method,
      &conveyor);

    #ifdef DEBUG_SHOW
    if(DepthParameters::debug_show_find_holes) // Debug
    {
      std::string msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
      msg += STR(" : Blobs");
      msgs.push_back(msg);
      imgs.push_back(
        Visualization::showHoles(
          msg,
          *interpolatedDepthImage,
          -1,
          conveyor.keyPoints,
          conveyor.rectangles,
          std::vector<std::string>(),
          conveyor.outlines)
        );
    }
    #endif

    //!< Since not all blobs are holes, sift blobs according to known properties
    //!< of holes in 3D space
    /*
     *HoleFilters::checkHoles(
     *    interpolatedDepthImage,
     *    initialPointCloud,
     *    conveyor);
     */


    #ifdef DEBUG_SHOW
    if(DepthParameters::debug_show_find_holes) // Debug
    {
      std::string msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
      msg += STR(" : Keypoints sent to hole_fusion");
      msgs.push_back(msg);
      imgs.push_back(
        Visualization::showKeypoints(
          msg,
          *interpolatedDepthImage,
          -1,
          conveyor.keyPoints)
      );
    }
    if(DepthParameters::debug_show_find_holes)
    {
      Visualization::multipleShow("findHoles function", imgs, msgs,
        DepthParameters::debug_show_find_holes_size, 1);
    }
    #endif

    //-------------------------------------------//
    //~ cv::Mat tempMat;
    //~ interpolatedDepthImage.copyTo(tempMat);
    //~ pf.update(conveyor.keyPoints);
    //~ pf.show(tempMat);  // Just for testing
    //~ Visualization::showScaled("PF",tempMat,1);
    //--------------------------------------------//

    #ifdef DEBUG_TIME
    Timer::tick("findHoles");
    Timer::printAllMeansTree();
    #endif

    return conveyor;
  }

} // namespace pandora_vision

