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

#ifndef PANDORA_VISION_VICTIM_VICTIM_VJ_DETECTOR_H
#define PANDORA_VISION_VICTIM_VICTIM_VJ_DETECTOR_H

#include "pandora_vision_victim/victim_parameters.h"

namespace pandora_vision
{
  class VictimVJDetector : public Processor<VictimCVMatStamped, POIsStamped>
  {
    public:
      //! The Constructor
      VictimVJDetector(const std::string& ns, sensor_processor::AbstractHandler* handler);
      //! Default constructor
      VictimVJDetector(void){}

      //! The Destructor
      ~VictimVJDetector();

      /**
      @brief Searches for faces in current frame.
      @param frame [cv::Mat] The current frame
      @return number [int] of faces found in current frame
      **/
      std::vector<DetectedVictim> findFaces(cv::Mat frame);

      /**
      @brief Creates the continuous table of faces found that contains
      information for each face in every set of 4 values.
      @return int[] table of face positions and sizes
      **/
      std::vector<BoundingBox> getAlertKeypoints();

      /**
      @brief Returns the probability of the faces detected in the frame
      @return [float] probability value
      **/
      std::vector<float> predictionToProbability(std::vector<float> predictions);
      
      /**
      @brief
      **/
      virtual bool process(const CVMatStampedConstPtr& input, const POIsStampedPtr& output);
    
    private:
      std::vector<std::vector< cv::Rect_<int> > > faces;

      std::vector<cv::Rect_<int> > faces_total;

      //!< Cascade classifier for face detection
      cv::CascadeClassifier trained_cascade;

      //!<Trained model for face detection
      cv::Ptr<cv::FaceRecognizer> trained_model;

      /**
      @brief Calls detectMultiscale to scan frame for faces and drawFace
      to create rectangles around the faces found in each frame
      @param frame [cv::Mat] the frame to be scaned.
      @return [int] the number of faces found in each frame
      **/
      std::vector<float> detectFace(cv::Mat frame);
      
    
    
      //----------------------------FROM DETECTION - TO DO-----------------------------------------------
      
      /**
      @brief This method check in which state we are, according to
      the information sent from hole_detector_node
      @return void
      **/
      void detectVictims(  // TO DO
        bool depthEnabled, 
        bool holesEnabled,
        const cv::Mat& rgbImage,
        const cv::Mat& depthImage,
        const pandora_vision_msgs::EnhancedHolesVectorMsg& msg
      );
      
      VictimParameters params_;
      
      //! Debug purposes
      // The image_transport nodehandle
      image_transport::ImageTransport imageTransport_;  // TO DO ALL
      image_transport::Publisher _debugVictimsPublisher;
      image_transport::Publisher _interpolatedDepthPublisher;
      cv::Mat debugImage;
      std::vector<cv::KeyPoint> rgb_vj_keypoints;
      std::vector<cv::KeyPoint> rgb_svm_keypoints;
      std::vector<cv::KeyPoint> depth_vj_keypoints;
      std::vector<cv::KeyPoint> depth_svm_keypoints;
      std::vector<cv::Rect> rgb_vj_bounding_boxes;
      std::vector<cv::Rect> rgb_svm_bounding_boxes;
      std::vector<cv::Rect> depth_vj_bounding_boxes;
      std::vector<cv::Rect> depth_svm_bounding_boxes;
      std::vector<cv::Rect> holes_bounding_boxes;
      std::vector<float> rgb_vj_p;
      std::vector<float> rgb_svm_p;
      std::vector<float> depth_vj_p;
      std::vector<float> depth_svm_p;
      
      //----------------------------------------------------------------------//
      DetectionImages dImages;  // TO DELETE

      /**
      @brief Function that enables suitable subsystems, according
      to the current State 
      @param [std::vector<cv::Mat>] vector of images to be processed. Size of
      vector can be either 2 or 1, if we have both rgbd information or not
      @return void
      **/ 
      std::vector<DetectedVictim> victimFusion(  // TO DO
        DetectionImages imgs, 
        DetectionMode detectionMode 
      );
      
      //********************************************************************************************
      
  };
}// namespace pandora_vision
#endif  // PANDORA_VISION_VICTIM_VICTIM_VJ_DETECTOR_H
