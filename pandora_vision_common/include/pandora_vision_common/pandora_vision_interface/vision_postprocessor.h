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
 *   Christos Tsirigotis <tsirif@gmail.com>
 *********************************************************************/

#ifndef PANDORA_VISION_COMMON_VISION_POSTPROCESSOR_H
#define PANDORA_VISION_COMMON_VISION_POSTPROCESSOR_H

#include <map>
#include <string>
#include <utility>

#include <boost/shared_ptr.hpp>
#include <urdf_parser/urdf_parser.h>

#include "vision_common/postprocessor.h"
#include "vision_common/abstract_handler.h"
#include "pandora_common_msgs/GeneralAlertInfo.h"

#include "pandora_vision_common/pois_stamped.h"

namespace pandora_vision
{
  template <class VisionAlertMsg>
  class VisionPostProcessor : public PostProcessor<POIsStamped, VisionAlertMsg>
  {
  private:
    typedef boost::shared_ptr<VisionAlertMsg> VisionAlertMsgPtr;

  public:
    VisionPostProcessor(const std::string& ns, AbstractHandler* handler);

    virtual bool
      postProcess(const POIsStampedConstPtr& input, const std_msgs::Int32Ptr& output);

  protected:
    std::vector<GeneralAlertInfo>
      getGeneralAlertInfo(const POIsStampedConstPtr& input);

  private:
    template <class T>
      T
      findParam(std::map<std::string, T>* dict, const std::string& key);
    std::string
      findParentFrameId(std::map<std::string, std::string>* dict,
        const std::string& key,
        const std::string& model_param_name);

  protected:
    std::map<std::string, std::string> parentFrameDict_;
    std::map<std::string, double> hfovDict_;
    std::map<std::string, double> vfovDict_;
  };

  template <class VisionAlertMsg>
    VisionPostProcessor<VisionAlertMsg>::
    VisionPostProcessor(const std::string& ns, AbstractHandler* handler) :
      PostProcessor<POIsStamped, VisionAlertMsg>(ns, handler)
    {
      //TODO
    }

  template <class VisionAlertMsg>
    VisionPostProcessor<VisionAlertMsg>::
    std::vector<GeneralAlertInfo>
    getGeneralAlertInfo(const POIsStampedConstPtr& input)
    {
      //TODO
    }

  template <class T>
    template <class VisionAlertMsg>
      VisionPostProcessor<VisionAlertMsg>::
      T
      findParam(std::map<std::string, T>* dict, const std::string& key)
      {
        std::map<std::string, T>::iterator iter;
        if ((iter = dict->find(key)) != dict->end())
        {
          return *iter;
        }
        else
        {
          std::string param;

          if (!this->accessPublicNh()->getParam(key, param))
          {
            ROS_ERROR_NAMED(this->getName(),
                "Params couldn't be retrieved for %s", key);
            // throw an exception
            return T();
          }

          dict->insert(std::make_pair(key, param));
          return param;
        }
      }

  template <class VisionAlertMsg>
    VisionPostProcessor<VisionAlertMsg>::
    std::string
    findParentFrameId(std::map<std::string, std::string>* dict,
        const std::string& key,
        const std::string& model_param_name)
    {
      std::map<std::string, std::string>::iterator iter;
      if ((iter = dict->find(key)) != dict->end())
      {
        return *iter;
      }
      else
      {
        std::string robot_description;

        if (!this->accessPublicNh()->getParam(model_param_name, robot_description))
        {
          ROS_ERROR_NAMED(this->getName(),
              "Robot description couldn't be retrieved from the parameter server.");
          // throw an exception
          return "";
        }

        std::string parent_frame_id;

        boost::shared_ptr<urdf::ModelInterface> model(
          urdf::parseURDF(robot_description));
        // Get current link and its parent
        boost::shared_ptr<const urdf::Link> currentLink = model->getLink(key);
        boost::shared_ptr<const urdf::Link> parentLink = currentLink->getParent();
        // Set the parent frame_id to the parent of the frame_id
        parent_frame_id = parentLink->name;
        dict->insert(std::make_pair(key, parent_frame_id));
        return parent_frame_id;
      }
    }
}  // namespace pandora_vision

#endif  // PANDORA_VISION_COMMON_VISION_POSTPROCESSOR_H
