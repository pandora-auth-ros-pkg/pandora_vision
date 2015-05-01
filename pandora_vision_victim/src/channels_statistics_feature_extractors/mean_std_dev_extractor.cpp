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
*   Marios Protopapas <protopapas_marios@hotmail.com>
*   Kofinas Miltiadis <mkofinas@gmail.com>
*********************************************************************/

#include <vector>

#include "pandora_vision_victim/channels_statistics_feature_extractors/mean_std_dev_extractor.h"

namespace pandora_vision
{
  /**
   * @brief Constructor
   */
  MeanStdDevExtractor::MeanStdDevExtractor(cv::Mat* img)
    :ChannelsStatisticsFeatureExtractor(img)
  {
  }

  /**
   * @brief Destructor
   */
  MeanStdDevExtractor::~MeanStdDevExtractor()
  {
  }

  /**
   * @brief This function extracts the mean and standard deviation value of
   * a color component.
   * @return [std::vector<double>] The mean and standard deviation vector.
   */
  std::vector<double> MeanStdDevExtractor::extract(void)
  {
    std::vector<double> meanStdDevVector;
    cv::Scalar mu, sigma;
    cv::meanStdDev(*img_, mu, sigma);
    for (unsigned int ii = 0; ii < img_->channels(); ii++)
    {
      meanStdDevVector.push_back(mu.val[ii]);
      meanStdDevVector.push_back(sigma.val[ii]);
    }
    return meanStdDevVector;
  }
}  // namespace pandora_vision
