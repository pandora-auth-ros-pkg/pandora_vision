/*********************************************************************
 *
 * Hardware License Agreement (BSD License)
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
 *     from this hardware without specific prior written permission.
 *
 *  THIS HARDWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS HARDWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors:
 *  Choutas Vassilis <vasilis4ch@gmail.com>
 *********************************************************************/

#include <string>
#include "pandora_vision_obstacle/hard_obstacle_detection/hard_obstacle_postprocessor.h"

namespace pandora_vision
{
namespace pandora_vision_obstacle
{
  HardObstaclePostProcessor::HardObstaclePostProcessor() :
    sensor_processor::PostProcessor<CVMatStamped, nav_msgs::OccupancyGrid>()
  {
  }

  /**
   * @brief Converts an OpenCV matrix that represents a traversability map
   * to a Occupancy Grid Format.
   * @param inputImage[const CVMatStampedConstPtr&] The input elevation map that will
   * converted.
   * @param outputOccupancyGrid[const nav_msgs::OccupancyGridPtr&] The resulting elevation
   * map in Occupancy Grid format.
   * @return bool True if the conversion was successful, false otherwise.
  */
  // bool CvMatToOccupancyGrid(const CVMatStampedConstPtr& inputImage,
  //     const nav_msgs::OccupancyGridPtr& outputOccupancyGrid)
  // {
  //   return true;
  // }

  bool HardObstaclePostProcessor::postProcess(const CVMatStampedConstPtr& input,
      const nav_msgs::OccupancyGridPtr& output)
  {
    // return CvMatToOccupancyGrid(input, output);
    return true;
  }

}  // namespace pandora_vision_obstacle
}  // namespace pandora_vision
