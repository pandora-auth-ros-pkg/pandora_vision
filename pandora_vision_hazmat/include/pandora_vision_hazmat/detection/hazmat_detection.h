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

#ifndef PANDORA_VISION_HAZMAT_DETECTION_HAZMAT_DETECTION_H
#define PANDORA_VISION_HAZMAT_DETECTION_HAZMAT_DETECTION_H

#include <dynamic_reconfigure/server.h>
#include "pandora_vision_hazmat/DisplayConfig.h"
#include "pandora_vision_hazmat/detection/detector_factory.h"
#include "pandora_vision_msgs/HazmatAlertsVectorMsg.h"

namespace pandora_vision
{
  namespace pandora_vision_hazmat
  {
    class HazmatDetectionNode
    {
      public:

        /*
         * @brief : Default Constructor that initiliazes the hazmat detector
         * and the necessary ROS objects for communication.
         */
        HazmatDetectionNode();

        /*
         * @brief : Class destructor that destroys the current detector. 
         */
        ~HazmatDetectionNode()
        {
          ROS_INFO("Destroying the detector object!");
          delete detector_;
        }

        /*
         * @brief : Class method that is used by the dynamic reconfigure
         * server to change object parameters.
         * @param config[const pandora_vision_hazmat::DisplayConfig&] :
         *  The message containing the new configuration for the node.
         * @param level[uint32_t]: Flag used by the dynamic reconfigure
         * server.
         *
         */
        void dynamicReconfigCallback(
            const ::pandora_vision_hazmat::DisplayConfig& config, 
            uint32_t level);

        /*
         * @brief : Receives an image message and detects the patterns on the 
         * image, if any are present.
         * @param imageMsg[const sensor_msgs::Image&] : The image message. 
         */
        void imageCallback(const sensor_msgs::Image& imageMsg);

      private:
        ros::NodeHandle nodeHandle_; //<! The node handle object for the node.

        ros::Subscriber imageSubscriber_; //<! Camera Subscriber 

        ros::Publisher hazmatPublisher_; //<! The publisher of hazmat alerts.

        dynamic_reconfigure::Server< ::pandora_vision_hazmat::DisplayConfig> 
          dynamicReconfServer_; //<! Reconfigure server for 
                                //<! changing object params

        bool execTimerFlag_; //<! Flag that toggles the execution time
                             //<! printing.

        bool debugMsgFlag_; //<! Flag that toggles debug messages that contain
        //<! for the detected patterns.

        std::string imageTopic_; //<! Image topic variable. 

        std::string hazmatTopic_; //<! Alert topic variable. 

        DetectorFactory factory_; //<! The factory that produces the detectors.

        PlanarObjectDetector *detector_; //<! Pointer to the detector used.
    };

} // namespace pandora_vision_hazmat
} // namespace pandora_vision

#endif  // PANDORA_VISION_HAZMAT_DETECTION_HAZMAT_DETECTION_H
