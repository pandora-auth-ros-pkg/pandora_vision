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
 * Authors:
 *   Chatzieleftheriou Eirini <eirini.ch0@gmail.com>
 *********************************************************************/

#ifndef PANDORA_VISION_OBSTACLE_BARREL_DETECTION_BARREL_PROCESSOR_H
#define PANDORA_VISION_OBSTACLE_BARREL_DETECTION_BARREL_PROCESSOR_H

#include <string>
#include <cv_bridge/cv_bridge.h>
#include "sensor_processor/processor.h"
#include "pandora_vision_common/pois_stamped.h"
#include "pandora_vision_common/images_stamped.h"
#include "pandora_vision_obstacle/obstacle_poi.h"

namespace pandora_vision
{
  class BarrelProcessor : public sensor_processor::Processor<ImagesStamped, POIsStamped>
  {
    public:
      typedef boost::shared_ptr<ObstaclePOI> ObstaclePOIPtr;

      BarrelProcessor(const std::string& ns, sensor_processor::Handler* handler);
      BarrelProcessor();

      virtual ~BarrelProcessor() {}

      virtual bool process(const ImagesStampedConstPtr& input,
        const POIsStampedPtr& output);
  };
}  // namespace pandora_vision

#endif  // PANDORA_VISION_OBSTACLE_BARREL_DETECTION_BARREL_PROCESSOR_H
