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

#ifndef PANDORA_VISION_VICTIM_FACE_DETECTOR_H
#define PANDORA_VISION_VICTIM_FACE_DETECTOR_H 

#include <math.h>
#include <iostream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/objdetect/objdetect.hpp"

namespace pandora_vision
{
  class FaceDetector
  {
    private:
   
    std::vector<std::vector<cv::Rect_<int> > > faces;

    std::vector<cv::Rect_<int> > faces_total;

    int _bufferSize;

    int now;

    //!< Total probability of face found in a frame
    float probability;

    //!< Image buffer used to store frames
    std::vector<cv::Mat> frame_buffer;

    //!< Vector with partial probabilities that are used to
    //!< calculate total probability of face according to
    //!< consistency in last _bufferSize frames
    std::vector<float> probability_buffer;

    //!< Cascade classifier for face detection
    cv::CascadeClassifier trained_cascade;

    //!<Trained model for face detection
    cv::Ptr<cv::FaceRecognizer> trained_model;

    /**
      @brief Initializes frame and probability buffer
      @param frame [cv::Mat] The current frame
      @return void
    */
    void initFrameProbBuffers(cv::Mat frame);

    /**
      @brief Calls detectMultiscale to scan frame for faces and drawFace
        to create rectangles around the faces found in each frame
      @param frame [cv::Mat] the frame to be scaned.
      @return [int] the number of faces found in each frame
    */
    int detectFace(cv::Mat frame);
    
    /**
    @brief Crate rectangles to current frame according to the positions
      of faces found in previous frames
    @param tmp [cv::Mat] The frame to be scanned for faces
    @return void
   */
    void createRectangles(cv::Mat *tmp);
    

  public:

    //! The Constructor
    FaceDetector(std::string cascade_path, std::string model_path,
        int bufferSize);

    //! The Destructor
    ~FaceDetector();

    /**
      @brief Searches for faces in current frame.
      @param frame [cv::Mat] The current frame
      @return number [int] of faces found in current frame
    **/
    int findFaces(cv::Mat frame);

    /**
      @brief Creates the continuous table of faces found that contains
      information for each face in every set of 4 values.
      @return int[] table of face positions and sizes
    */
    int* getFacePositionTable();

    /**
      @brief Returns the size of the table with the positions of the
      faces found
      @return [int] size of table
    */
    int getFaceTableSize();

    /**
      @brief Returns the probability of the faces detected in the frame
      @return [float] probability value
    */
    float getProbability();

  };
}// namespace pandora_vision
#endif  // PANDORA_VISION_VICTIM_FACE_DETECTOR_H
