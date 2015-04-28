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
 * Authors: Angelos Triantafyllidis, Manos Tsardoulias
 *********************************************************************/

#include "utils/image_matching.h"


/**
  @namespace pandora_vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{

  /**
    @brief Converts Conveyors which represent each hole and include all 
    necessary information about it to match with rgb and depth images.
    @param[in] conveyor [HolesConveyor*]
    The input conveyor to be converted and sent back as output.
    @param[in] x_th [double] The x coordinate of thermal image in rgb image
    @param[in] y_th [double] The y coordinate of thermal image in rgb image
    @param[in] c_x [double] The c factor on x-axis.
    Found as matchedthermal_x/rgb_x
    @param[in] c_y [double] The c factor on y-axis.
    Found as matchedthermal_y/rgb_y
    @return void
   **/
  void ImageMatching::conveyorMatching(HolesConveyor* conveyor, double x_th,
    double y_th, double c_x, double c_y)
  {
    for(unsigned int i = 0; i < conveyor->size(); i++)
    {
      // Match the keypoint of each Hole found
      conveyor->holes[i].keypoint.pt.x = matchingFunction(
        conveyor->holes[i].keypoint.pt.x, x_th, c_x);
      conveyor->holes[i].keypoint.pt.y = matchingFunction(
        conveyor->holes[i].keypoint.pt.y, y_th, c_y); 

      // Match the bounding box
      for(int j = 0; j < conveyor->holes[i].rectangle.size(); j++)
      {
        conveyor->holes[i].rectangle[j].x = matchingFunction(
          conveyor->holes[i].rectangle[j].x, x_th, c_x);
        conveyor->holes[i].rectangle[j].y = matchingFunction(
          conveyor->holes[i].rectangle[j].y, y_th, c_y);
      }

      // Match the blobs outline points
      for(int o = 0; o < conveyor->holes[i].outline.size(); o++)
      {
        conveyor->holes[i].outline[o].x = matchingFunction(
          conveyor->holes[i].outline[o].x, x_th, c_x);
        conveyor->holes[i].outline[o].y = matchingFunction(
          conveyor->holes[i].outline[o].y, y_th, c_y);
      }

    }
  }

  /**
    @brief The function that converts each point coordinates.
    @param[in] point [double]. The input point coordinates to be converted.
    @return float. The final point in Rgb or Depth image.
    @param[in] arg1 [double] The x or y coordinate of thermal image in rgb image
    @param[in] arg2 [double] The c factor on x-axis or y-axis.
    Found as matchedthermal_x/rgb_x same for y.
    return double. The final point coordinate in Rgb or Depth image.
  **/
  double ImageMatching::matchingFunction(double point, double arg1, double arg2)
  {
    // The value of thermal image point.x or point.y in Rgb image
    double coordinates_rgb = arg1 + arg2 * point;

    return coordinates_rgb;
  }

  /**
    @brief Set up the variables needed to convert the points.Takes variables
    from yaml file so that if one (or both) camera's position is changed
    on the robot they can be changed too.
    @param[in] nh [ros::Nodehandle&] The nodehandle of the class that calls 
    the function.
    @param[in] x_th [double] The x coordinate of thermal image in rgb image
    @param[in] y_th [double] The y coordinate of thermal image in rgb image
    @param[in] c_x [double] The c factor on x-axis.
    Found as matchedthermal_x/rgb_x
    @param[in] c_y [double] The c factor on y-axis. 
    Found as matchedthermal_y/rgb_y
    @return void
  **/
  void ImageMatching::variableSetUp(ros::NodeHandle& nh, double* x_th, 
    double* y_th, double* c_x, double* c_y)
  {
    // The namespace dictated in the launch file
    std::string ns = nh.getNamespace();

    // Read values of each variable.
    // x_thermal variable.
    if(nh.getParam(ns + "/thermal_camera_node/matching_values/x_therm", *x_th))
    {
      ROS_INFO("[Thermal_node], x_th variable is loaded for matching");
    }
    else
    {
      ROS_ERROR("[Thermal_node], Could not find x_th variable");
    }

    // y_thermal variable.
    if(nh.getParam(ns + "/thermal_camera_node/matching_values/y_therm", *y_th))
    {
      ROS_INFO("[Thermal_node], y_th variable is loaded for matching");
    }
    else
    {
      ROS_ERROR("[Thermal_node], Could not find y_th variable");
    }

    // C factor on x directions.
    if(nh.getParam(ns + "/thermal_camera_node/matching_values/c_x", *c_x))
    {
      ROS_INFO("[Thermal_node], c_x variable is loaded for matching");
    }
    else
    {
      ROS_ERROR("[Thermal_node], Could not find c_x variable");
    }

    // C factor on y directions.
    if(nh.getParam(ns + "/thermal_camera_node/matching_values/c_y", *c_y))
    {
      ROS_INFO("[Thermal_node], c_y variable is loaded for matching");
    }
    else
    {
      ROS_ERROR("[Thermal_node], Could not find c_y variable");
    }
  }


} // namespace pandora_vision
