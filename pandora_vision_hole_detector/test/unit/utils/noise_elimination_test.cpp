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

#include "utils/noise_elimination.h"
#include "gtest/gtest.h"

namespace pandora_vision
{
  /**
    @class NoiseEliminationTest
    @brief Tests the integrity of methods of class NoiseElimination
   **/
  class NoiseEliminationTest : public ::testing::Test
  {
    protected:

      NoiseEliminationTest () {}

      /**
        @brief Constructs a filled rectangle of width @param x
        and height of @param y
        @param[in] upperLeft [const cv::Point2f&] The upper left vertex of the
        rectangle to be created
        @param[in] x [const int&] The recgangle's width
        @param[in] y [const int&] The rectangle's height
        @param[in] value [const float&] The value with which the inside of the
        rectangle will be filled
        @param[out] image [cv::Mat*] The image on which the rectangle will be
        imprinted on. Its size must be set before calling this method
        return void
       **/
      void generateFilledRectangle (
        const cv::Point2f& upperLeft,
        const int& x,
        const int& y,
        const float& value,
        cv::Mat* image );

      //! Sets up images needed for testing
      virtual void SetUp ()
      {
        WIDTH = 640;
        HEIGHT = 480;

        // Construct interpolationMethod0
        interpolationMethod0 = cv::Mat::zeros( HEIGHT, WIDTH, CV_32FC1 );

        generateFilledRectangle ( cv::Point2f ( 10, 10 ),
          WIDTH - 10,
          HEIGHT - 10,
          1.0,
          &interpolationMethod0);

        // Construct interpolationMethod1
        interpolationMethod1 = cv::Mat::zeros( HEIGHT, WIDTH, CV_32FC1 );

        generateFilledRectangle ( cv::Point2f ( 10, 10 ),
          WIDTH - 10,
          HEIGHT - 10,
          0.6,
          &interpolationMethod1);


        // Construct interpolationMethod2
        interpolationMethod2 = cv::Mat::zeros( HEIGHT, WIDTH, CV_32FC1 );

        generateFilledRectangle ( cv::Point2f ( 10, 10 ),
          WIDTH - 10,
          HEIGHT - 10,
          0.0,
          &interpolationMethod2);

      }

      int WIDTH;
      int HEIGHT;

      // Three images that will be used to test
      // methods of class NoiseElimination

      // An image with minimal noise (black pixels)
      cv::Mat interpolationMethod0;

      // An image with considerable amount of noise (black pixels)
      cv::Mat interpolationMethod1;

      // An image heavily noisy (black pixels)
      cv::Mat interpolationMethod2;
  };



  /**
    @brief Constructs a filled rectangle of width @param x
    and height of @param y
    @param[in] upperLeft [const cv::Point2f&] The upper left vertex of the
    rectangle to be created
    @param[in] x [const int&] The recgangle's width
    @param[in] y [const int&] The rectangle's height
    @param[in] value [const float&] The value with which the inside of the
    rectangle will be filled
    @param[out] image [cv::Mat*] The image on which the rectangle will be
    imprinted on. Its size must be set before calling this method
    return void
   **/
  void NoiseEliminationTest::generateFilledRectangle (
    const cv::Point2f& upperLeft,
    const int& x,
    const int& y,
    const float& value,
    cv::Mat* image )
  {
    if ( image->type() != CV_32FC1 )
    {
      std::cerr << "Image of invalid type. Please use CV_32FC1" << std::endl;

      return;
    }

    // Fill @param image with black pixels
    *image = cv::Mat::zeros( image->size(), CV_32FC1 );

    // The four vertices of the rectangle
    cv::Point2f vertex_1( upperLeft.x, upperLeft.y );

    cv::Point2f vertex_2( upperLeft.x, upperLeft.y + y - 1 );

    cv::Point2f vertex_3( upperLeft.x + x - 1, upperLeft.y + y - 1 );

    cv::Point2f vertex_4( upperLeft.x + x - 1, upperLeft.y );

    cv::Point2f a[] = { vertex_1, vertex_2, vertex_3, vertex_4 };

    // Fill the inside of the desired rectangle with the @param value provided
    for( int rows = 0; rows < y; rows++ )
    {
      for ( int cols = 0; cols < x; cols++ )
      {
        image->at< float >( rows, cols ) = value;
      }
    }
  }



  //! Test NoiseElimination::chooseInterpolationMethod
  TEST_F ( NoiseEliminationTest, ChooseInterpolationMethodTest )
  {
    // On interpolationMethod0, Parameters::Depth::interpolation_method
    // should be equal to 0

    NoiseElimination::chooseInterpolationMethod ( interpolationMethod0 );

    ASSERT_EQ ( 0, Parameters::Depth::interpolation_method );


    // On interpolationMethod1, Parameters::Depth::interpolation_method
    // should be equal to 1

    NoiseElimination::chooseInterpolationMethod ( interpolationMethod1 );

    ASSERT_EQ ( 1, Parameters::Depth::interpolation_method );


    // On interpolationMethod2, Parameters::Depth::interpolation_method
    // should be equal to 2

    NoiseElimination::chooseInterpolationMethod ( interpolationMethod2 );

    ASSERT_EQ ( 2, Parameters::Depth::interpolation_method );

  }

} // namespace pandora_vision
