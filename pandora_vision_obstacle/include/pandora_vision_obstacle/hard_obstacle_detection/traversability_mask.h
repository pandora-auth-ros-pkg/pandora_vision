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
 *********************************************************************/

#ifndef PANDORA_VISION_OBSTACLE_HARD_OBSTACLE_DETECTION_TRAVERSABILITY_MASK_H
#define PANDORA_VISION_OBSTACLE_HARD_OBSTACLE_DETECTION_TRAVERSABILITY_MASK_H

#include <boost/shared_ptr.hpp>

#include "ros/ros.h"
#include "opencv2/core/core.hpp"
#include "pandora_vision_obstacle/hard_obstacle_detection/RobotGeometryMaskDescription.h"

namespace pandora_vision
{
namespace pandora_vision_obstacle
{
  /**
   * @class TraversabilityMask TODO
   */
  class TraversabilityMask
  {
   public:
    typedef boost::shared_ptr<TraversabilityMask> Ptr;
    typedef boost::shared_ptr<TraversabilityMask const> ConstPtr;

    typedef boost::shared_ptr<cv::Mat> MatPtr;
    typedef boost::shared_ptr<cv::Mat const> MatConstPtr;

   public:
    TraversabilityMask();
    virtual ~TraversabilityMask();

    /**
     * Find trinary traversability of point in elevation map
     * Values can be free, obstacle or unknown
     */
    int8_t
    findTraversability(const cv::Point& center);

    /**
     * Sets elevation map to be used
     */
    void
    setElevationMap(const boost::shared_ptr<cv::Mat const>& map);

    /**
     * load robot's geometry mask from yaml (matrix A)
     */
    void
    loadGeometryMask(const ros::NodeHandle& nh);

   private:
    /**
     * Finds B matrices, input is respective A matrix, heights and wheel
     * distance. Should compare wheel heights and make the proper
     * transformations
     */
    void
    findElevatedLeft(MatPtr al, double hForward, double hBack, double d);
    void
    findElevatedRight(MatPtr ar, double hForward, double hBack, double d);
    void
    findElevatedTop(MatPtr at, double hLeft, double hRight, double d);
    void
    findElevatedBottom(MatPtr ab, double hLeft, double hRight, double d);

    /**
     * @brief: find mean and stdev of height on wheel, false if wheel has
     * unknown values underneath it
     */
    bool
    findHeightOnWheel(const cv::Point& wheelPos, double* meanHeight, double* stdDevHeight);

    /**
     * Fills up wheel with a mask which corresponds to the wheel, performs a
     * check whether wheel is on unknown terrain. Returns false if it is
     */
    bool
    cropToWheel(const cv::Point& wheelPos, const MatPtr& wheel);

    /**
     * Crops elevationMap_ to Ar according to center point and geometric params
     */
    MatPtr
    cropToRight();

    /**
     * Crops elevationMap_ to Al according to center point and geometric params
     */
    MatPtr
    cropToLeft();

    /**
     * Crops elevationMap_ to At according to center point and geometric params
     */
    MatPtr
    cropToTop();

    /**
     * Crops elevationMap_ to Ab according to center point and geometric params
     */
    MatPtr
    cropToBottom();

    inline int metersToSteps(double meters)
    {
      return static_cast<int>(meters / description_->RESOLUTION) + 1;
    }
   private:
    //!< This is A matrix
    MatPtr robotGeometryMask_;
    //!< This is geometrical description of robot's base
    RobotGeometryMaskDescriptionPtr description_;
    //!< Local elevation map created
    MatConstPtr elevationMapPtr_;

    //!< center of matrix A
    cv::Point center_;
  };

  typedef TraversabilityMask::Ptr TraversabilityMaskPtr;
  typedef TraversabilityMask::ConstPtr TraversabilityMaskConstPtr;
}  // namespace pandora_vision_obstacle
}  // namespace pandora_vision

#endif  // PANDORA_VISION_OBSTACLE_HARD_OBSTACLE_DETECTION_TRAVERSABILITY_MASK_H
