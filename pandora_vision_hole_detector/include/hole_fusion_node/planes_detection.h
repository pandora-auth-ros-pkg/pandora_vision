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
* Authors: Alexandros Filotheou, Manos Tsardoulias
*********************************************************************/

#ifndef PLANES_DETECTION_H
#define PLANES_DETECTION_H

#include "hole_fusion_node/hole_fusion_parameters.h"

/**
  @namespace vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
  /**
    @class PlanesDetection
    @brief Provides methods for plane extraction
   **/
  class PlanesDetection
  {
    public:

      /**
        @brief Identify the planes in a point cloud and return the number of
        detected planes.
        @param[in] inputCloud [const PointCloudXYZPtr&] The point cloud whose
        planes we wish to locate
        @param[in] applyVoxelFilter [const bool&] Apply the voxel filter or not
        on the input cloud.
        @param[out] inliersVector [std::vector<pcl::PointIndices::Ptr>*]
        A vector of pointers to a pcl::PointIndices struct where point indices
        are stored. In pandora's context, it is used to calculate the maximum
        ratio of points lying on a plane
        @return The number of planes detected in inputCloud.
       **/
      static int locatePlanes(const PointCloudXYZPtr& inputCloud,
          const bool& applyVoxelFilter,
          std::vector<pcl::PointIndices::Ptr>* inliersVector);


      /**
        @brief Identify the planes in a point cloud and return a vector
        cointaining pointers to them.
        @param[in] inputCloud [const PointCloudXYZPtr&] The point cloud whose
        planes we wish to locate
        @param[in] applyVoxelFilter [const bool&] Apply the voxel filter or not
        on the input cloud.
        @param[out] planesVectorOut
        [std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>*]
        the output vector of pointers to planes
        @param[out] coefficientsVectorOut [std::vector<pcl::ModelCoefficients>*]
        the output vector of coefficients of each plane detected
        @return void
       **/
      static void locatePlanes(const PointCloudXYZPtr& inputCloud,
          const bool& applyVoxelFilter,
          std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>* planesVectorOut,
          std::vector<pcl::ModelCoefficients>* coefficientsVectorOut);

      /**
        @brief Locates planes using the SACS segmentation
        (as stated in http://www.pointclouds.org/documentation/tutorials/
        extract_indices.php#extract-indices)
        @param[in] cloudIn [const PointCloudXYZPtr&] The point cloud whose
        planes we wish to locate
        @param[out] planesVectorOut
        [std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>*]
        the output vector of pointers to planes
        @param[out] coefficientsVectorOut [std::vector<pcl::ModelCoefficients>*]
        The output vector of coefficients of each plane detected
        @param[out] inliersVectorOut [std::vector<pcl::PointIndices::Ptr>*]
        The inliers for each plane
        @return void
       **/
      static void locatePlanesUsingSACSegmentation
        (const PointCloudXYZPtr& cloudIn,
         std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>* planesVectorOut,
         std::vector<pcl::ModelCoefficients>* coefficientsVectorOut,
         std::vector<pcl::PointIndices::Ptr>* inliersVectorOut);

      /**
        @brief Locates planes using the normals SACS segmentation
        (http://pi-robot-ros-pkg.googlecode.com/svn-history/r303/trunk/pi_pcl/
        src/cylinder.cpp)
        @param[in] cloudIn [const PointCloudXYZPtr&] The point cloud whose
        planes we wish to locate
        @param[out] planesVectorOut
        [std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>*]
        The output vector of pointers to planes
        @param[out] coefficientsVectorOut [std::vector<pcl::ModelCoefficients>*]
        The output[out] vector of coefficients of each plane detected
        @param[out] inliersVectorOut [std::vector<pcl::PointIndices::Ptr>*]
        The inliers for each plane
        @return void
       **/
      static void locatePlanesUsingNormalsSACSegmentation
        (const PointCloudXYZPtr& cloudIn,
         std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>* planesVectorOut,
         std::vector<pcl::ModelCoefficients>* coefficientsVectorOut,
         std::vector<pcl::PointIndices::Ptr>* inliersVectorOut);

      /**
        @brief Applies a voxel grid filtering
        (http://pointclouds.org/documentation/tutorials/voxel_grid.php)
        @param[in] cloudIn [const PointCloudXYZPtr&] The point cloud to filter
        @return PointCloudXYZPtr A pointer to the filtered cloud
       **/
      static PointCloudXYZPtr applyVoxelGridFilter
        (const PointCloudXYZPtr& cloudIn);
  };

}

#endif
