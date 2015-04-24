/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, P.A.N.D.O.R.A. Team.
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
 * Authors: Choutas Vassilis 
 *********************************************************************/

#include "pandora_vision_hazmat/detection/hazmat_processor.h"

namespace pandora_vision
{
  namespace pandora_vision_hazmat
  {
    HazmatProcessor::HazmatProcessor(const std::string& ns, 
      sensor_processor::Handler* handler) : VisionProcessor(ns, handler)
    {
      ROS_INFO("Starting Hazmat Detection Node!\n");

      debugMsgFlag_ = false;
      execTimerFlag_ = false;

      // Get the path of the current package.
      std::string packagePath = ros::package::getPath("pandora_vision_hazmat");   

      PlanarObjectDetector::setFileName(packagePath);

      // Initiliaze the object detector
      std::string featureType;

      if (!this->accessPublicNh()->getParam(ns + "/feature_type", featureType))
      {
        ROS_ERROR("Could not get the feature type parameter!\n");
        ROS_INFO("Initializing default detector!\n");
        featureType = "SIFT"; 
      }

      detector_ = factory_.createDetectorObject(featureType);

      // Check if the detector was initialized.
      if (detector_ == NULL)
      {
        ROS_FATAL("Could not create a detector object!\n");
        ROS_FATAL("The node will now exit!\n");
        ROS_BREAK();
      }
      ROS_INFO("Created the detector object!\n");

      dynamicReconfServer_.setCallback(boost::bind(
            &HazmatProcessor::dynamicReconfigCallback, this, _1, _2));
    }
    
    HazmatProcessor::HazmatProcessor() : VisionProcessor() {}
    
    HazmatProcessor::~HazmatProcessor()
    {
      ROS_INFO("Destroying the detector object!");
      delete detector_;
    }
    
    void HazmatProcessor::dynamicReconfigCallback(
        const ::pandora_vision_hazmat::DisplayConfig& config, uint32_t level)
    {
      if (detector_ == NULL)
      {
        ROS_ERROR("No detector object created!");
        return;
      }
      // Check if the features type has changed. If yes create a new detector.
      if (detector_->getFeaturesName().compare(config.Feature_Type) != 0)
      {
        delete detector_;
        ROS_INFO("[PANDORA_VISION_HAZMAT] : Create new %s detector",
            config.Feature_Type.c_str());
        detector_ = factory_.createDetectorObject(config.Feature_Type);
      }

      detector_->setDisplayResultsFlag(config.Display_Results);
      detector_->setFeatureTimerFlag(config.Feature_Timer);
      detector_->setMaskDisplayFlag(config.Mask_Display);
      this->execTimerFlag_ = config.Execution_Timer;
      this->debugMsgFlag_ = config.Debug_Messages;
    }
    
    bool HazmatProcessor::process(const CVMatStampedConstPtr& input, 
      const POIsStampedPtr& output)
    {
      output->header = input->header;
      
      std::stringstream ss;
      const clock_t begin_time = clock();
      PlanarObjectDetector::setDims(input->image);
      
      ROS_INFO("Detecting...");
      
      bool found = detector_->detect(input->image, &output->pois);
      
      ROS_INFO("Detection done!!");
      double execTime = ( clock () - begin_time ) /  
        static_cast<double>(CLOCKS_PER_SEC );
        
      // Check if the debug message printing is enabled.
      if (found && debugMsgFlag_)
      {
        for (int i = 0 ; i < output->pois.size() ; i++)
        {
          boost::shared_ptr<HazmatPOI> hazmatPOIPtr(
            boost::dynamic_pointer_cast<HazmatPOI>(output->pois[i]));
          ROS_INFO("[PANDORA_VISION_HAZMAT] : Found Hazmat : %d",
              hazmatPOIPtr->getPattern());
        }
        ROS_INFO("[PANDORA_VISION_HAZMAT] : Number of Hazmats Found = %d .",
            static_cast<int>(output->pois.size()));
      }

      if (execTimerFlag_)
        ROS_INFO("[PANDORA_VISION_HAZMAT] : Detection Execution Time is : %f!",
            execTime);

      //~ ss << execTime * 1000;

      //~ cv::putText(input->image, "Exec Time : " + ss.str() + "ms", 
          //~ cv::Point(0, input->image.rows) ,
          //~ CV_FONT_HERSHEY_PLAIN, 3, cv::Scalar(0 , 0 , 255), 3);

      // TO DO : Add visualization flag
      cv::imshow("Input Image", input->image);
      char key = cv::waitKey(10);
      if ( key == 27)
      {
        ROS_INFO("Goodbye! \n");
        ros::shutdown();
      }
      return found;
    }
  }  // namespace pandora_vision_hazmat
}  // namespace pandora_vision
