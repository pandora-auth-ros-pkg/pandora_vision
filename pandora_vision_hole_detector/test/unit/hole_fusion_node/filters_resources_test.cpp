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
 * Author: Alexandros Philotheou
 *********************************************************************/

#include "hole_fusion_node/filters_resources.h"
#include "gtest/gtest.h"

namespace pandora_vision
{
  /**
    @class FiltersResourcesTest
    @brief Tests the integrity of methods of class FiltersResources
   **/
  class FiltersResourcesTest : public ::testing::Test
  {
    protected:

      FiltersResourcesTest() {}

      /**
        @brief Constructs the internals of a rectangular hole
        of width @param x and height of @param y
        @param[in] upperLeft [const cv::Point2f&] The upper left vertex of the
        rectangle to be created
        @param[in] x [const int&] The recgangle's width
        @param[in] y [const int&] The rectangle's height
        return [HolesConveyor] A struct containing the elements of one hole
       **/
      HolesConveyor getConveyor (
        const cv::Point2f& upperLeft,
        const int& x,
        const int& y );

      virtual void SetUp ()
      {
        WIDTH = 640;
        HEIGHT = 480;

        // An image needed only for its size
        squares_ = cv::Mat::zeros( HEIGHT, WIDTH, CV_32FC1 );

        HolesConveyorUtils::append(
          getConveyor( cv::Point2f ( WIDTH - 100, HEIGHT - 100 ),
            100,
            100 ),
          &conveyor);

        HolesConveyorUtils::append(
          getConveyor( cv::Point2f ( WIDTH - 103, 3 ),
            100,
            100 ),
          &conveyor);

        HolesConveyorUtils::append(
          getConveyor( cv::Point2f ( 100, 100 ),
            100,
            100 ),
          &conveyor);

      }

      // An image needed only for its size
      cv::Mat squares_;

      // Dimensions of the squares_ image
      int WIDTH;
      int HEIGHT;

      // The overall conveyor holding the holes
      HolesConveyor conveyor;

  };



  /**
    @brief Constructs the internals of a rectangular hole
    of width @param x and height of @param y
    @param[in] upperLeft [const cv::Point2f&] The upper left vertex of the
    rectangle to be created
    @param[in] x [const int&] The recgangle's width
    @param[in] y [const int&] The rectangle's height
    return [HolesConveyor] A struct containing the elements of one hole
   **/
  HolesConveyor FiltersResourcesTest::getConveyor (
    const cv::Point2f& upperLeft,
    const int& x,
    const int& y )
  {
    // What will be returned: the internal elements of one hole
    HolesConveyor conveyor;

    // The hole's keypoint
    cv::KeyPoint k (  upperLeft.x + x / 2, upperLeft.y + y / 2 , 1 );

    conveyor.keyPoints.push_back(k);


    // The four vertices of the rectangle
    cv::Point2f vertex_1( upperLeft.x, upperLeft.y );

    cv::Point2f vertex_2( upperLeft.x, upperLeft.y + y - 1 );

    cv::Point2f vertex_3( upperLeft.x + x - 1, upperLeft.y + y - 1 );

    cv::Point2f vertex_4( upperLeft.x + x - 1, upperLeft.y );

    std::vector<cv::Point2f> rectangle;
    rectangle.push_back(vertex_1);
    rectangle.push_back(vertex_2);
    rectangle.push_back(vertex_3);
    rectangle.push_back(vertex_4);

    conveyor.rectangles.push_back(rectangle);


    // The outline points of the hole will be obtained through the depiction
    // of the points consisting the rectangle
    cv::Mat image = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC1 );

    cv::Point2f a[] = {vertex_1, vertex_2, vertex_3, vertex_4};

    for(unsigned int j = 0; j < 4; j++)
    {
      cv::line(image, a[j], a[(j + 1) % 4], cv::Scalar(255, 0, 0), 1, 8);
    }


    std::vector<cv::Point2f> outline;
    for ( int rows = 0; rows < image.rows; rows++ )
    {
      for ( int cols = 0; cols < image.cols; cols++ )
      {
        if ( image.at<unsigned char>( rows, cols ) != 0 )
        {
          outline.push_back( cv::Point2f ( cols, rows ) );
        }
      }
    }

    conveyor.outlines.push_back(outline);

    return conveyor;

  }



  // Test FiltersResources::createHolesMasksSetVector
  TEST_F ( FiltersResourcesTest, CreateHolesMasksSetVector )
  {
    // The indices of points inside the holes in conveyor
    std::vector<std::set<unsigned int> > holesMasksSetVector;

    // Run FiltersResources::createHolesMasksSetVector
    FiltersResources::createHolesMasksSetVector(
      conveyor,
      squares_,
      &holesMasksSetVector );

    for ( int h = 0; h < HolesConveyorUtils::size( conveyor ); h++ )
    {
      EXPECT_EQ ( 10000, holesMasksSetVector[h].size() );
    }
  }

} // namespace pandora_vision
