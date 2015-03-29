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
 * Authors:
 *   Tsirigotis Christos <tsirif@gmail.com>
 *   Chatzieleftheriou Eirini <eirini.ch0@gmail.com>
 *********************************************************************/

#ifndef PANDORA_VISION_QRCODE_QRCODE_POSTPROCESSOR_H
#define PANDORA_VISION_QRCODE_QRCODE_POSTPROCESSOR_H

#include "pandora_common_msgs/GeneralAlertInfoVector.h"
#include "pandora_vision_msgs/QRAlertsVectorMsg.h"
#include "pandora_vision_common/vision_postprocessor.h"
#include "pandora_vision_qrcode/qrcode_poi.h"

namespace pandora_vision
{
  class QrCodePostProcessor : public VisionPostProcessor<pandora_vision_msgs::QRAlertsVectorMsg>
  {
    public:
      typedef boost::shared_ptr<pandora_vision_msgs::QRAlertsVectorMsg> QRAlertsVectorMsgPtr;
      
      QrCodePostProcessor(const std::string& ns, AbstractHandler* handler);
      ~QrCodePostProcessor() {}
      
    virtual bool
      postProcess(const POIsStampedConstPtr& input, const QRAlertsVectorMsgPtr& output);
  };
  
  QrCodePostProcessor::QrCodePostProcessor(const std::string& ns, AbstractHandler* handler) :
    VisionPostProcessor<pandora_vision_msgs::QRAlertsVectorMsg>(ns, handler)
  {
  }
  
  bool QrCodePostProcessor::postProcess(const POIsStampedConstPtr& input, const QRAlertsVectorMsgPtr& output)
  {
    pandora_common_msgs::GeneralAlertInfoVector alertVector = getGeneralAlertInfo(input);
    output->header = alertVector.header;
    
    for (int ii = 0; ii < alertVector.generalAlerts.size(); ii++)
    {
      output->qrAlerts[ii].yaw = alertVector.generalAlerts[ii].yaw;
      output->qrAlerts[ii].pitch = alertVector.generalAlerts[ii].pitch;
      
      boost::shared_ptr<QrCodePOI> qrCodePOI(boost::dynamic_pointer_cast<QrCodePOI>(input->pois[ii]));
      
      output->qrAlerts[ii].QRcontent = qrCodePOI->getContent();
    }
  }
}  // namespace pandora_vision

#endif  // PANDORA_VISION_COMMON_QRCODE_POSTPROCESSOR_H
