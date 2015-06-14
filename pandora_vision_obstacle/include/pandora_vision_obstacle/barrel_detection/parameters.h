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
 * Authors: Vasilis Bosdelekidis
 *********************************************************************/

#include <pandora_vision_obstacle/barrel_nodeConfig.h>
#include <dynamic_reconfigure/server.h>

/**
  @namespace pandora_vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
  class BarrelParametersHandler
  {
    public:
      //the constructor
      BarrelParametersHandler();

    private:
      // The dynamic reconfigure (Barrel) parameters' server
      dynamic_reconfigure::Server<pandora_vision_obstacle::barrel_nodeConfig> serverBarrel;

      // The dynamic reconfigure (Barrel) parameters' callback
      dynamic_reconfigure::Server<pandora_vision_obstacle::barrel_nodeConfig>::CallbackType fBarrel;

      /**
        @brief The function called when a parameter is changed
        @param[in] configBarrel [const pandora_vision_hole::barrel_nodeConfig&]
        @param[in] level [const uint32_t]
        @return void
       **/
      void parametersCallbackBarrel(
          const pandora_vision_obstacle::barrel_nodeConfig& configBarrel,
          const uint32_t& level);
  };
  /**
    @struct BarrelDetection
    @brief Provides flexibility by parameterizing variables needed by the
    barrel detection package
   **/
  struct BarrelDetection
  {
    static bool show_respective_barrel;
    static bool show_valid_barrel;
    static int fsd_canny_thresh_1;
    static int fsd_canny_thresh_2;
    static int fsd_min_pair_dist;
    static int fsd_max_pair_dist;
    static int fsd_no_of_peaks;
    static float roi_variance_thresh;
  };

}  // namespace pandora_vision

