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
        and height of @param y. All points inside the rectangle have a value of
        0.0
        @param[in] upperLeft [const cv::Point2f&] The upper left vertex of the
        rectangle to be created
        @param[in] x [const int&] The recgangle's width
        @param[in] y [const int&] The rectangle's height
        @param[out] image [cv::Mat*] The image on which the rectangle will be
        imprinted on. Its size must be set before calling this method
        return void
       **/
      void generateFilledRectangle (
        const cv::Point2f& upperLeft,
        const int& x,
        const int& y,
        cv::Mat* image );

      //! Sets up images needed for testing
      virtual void SetUp ()
      {
        WIDTH = 640;
        HEIGHT = 480;

        // Construct interpolationMethod0
        interpolationMethod0 = cv::Mat::zeros( HEIGHT, WIDTH, CV_32FC1 );

        // Fill interpolationMethod0 with a value of 1.0
        for ( int rows = 0; rows < HEIGHT; rows++ )
        {
          for ( int cols = 0; cols < WIDTH; cols++)
          {
            interpolationMethod0.at< float >( rows, cols ) = 1.0;
          }
        }

        // Insert noise
        generateFilledRectangle ( cv::Point2f ( 0, 0 ),
          WIDTH - 200,
          HEIGHT - 200,
          &interpolationMethod0);

        generateFilledRectangle ( cv::Point2f ( WIDTH - 50, HEIGHT - 50 ),
          10,
          10,
          &interpolationMethod0);

        // Uncomment for visual inspection
        /*
         *Visualization::showScaled
         *  ( "interpolationMethod0", interpolationMethod0, 0);
         */

        // Construct interpolationMethod1
        interpolationMethod1 = cv::Mat::zeros( HEIGHT, WIDTH, CV_32FC1 );

        // Fill interpolationMethod1 with a value of 0.5
        for ( int rows = 0; rows < HEIGHT; rows++ )
        {
          for ( int cols = 0; cols < WIDTH; cols++)
          {
            interpolationMethod1.at< float >( rows, cols ) = 0.5;
          }
        }

        // Insert noise
        generateFilledRectangle ( cv::Point2f ( 0, 0 ),
          WIDTH - 50,
          HEIGHT - 200,
          &interpolationMethod1);

        generateFilledRectangle ( cv::Point2f ( WIDTH - 50, HEIGHT - 50 ),
          10,
          10,
          &interpolationMethod1);

        // Uncomment for visual inspection
        /*
         *Visualization::showScaled
         *  ( "interpolationMethod1", interpolationMethod1, 0);
         */


        // Construct interpolationMethod2
        interpolationMethod2 = cv::Mat::zeros( HEIGHT, WIDTH, CV_32FC1 );

        // Fill interpolationMethod2 with a value of 1.0
        for ( int rows = 0; rows < HEIGHT; rows++ )
        {
          for ( int cols = 0; cols < WIDTH; cols++)
          {
            interpolationMethod2.at< float >( rows, cols ) = 1.0;
          }
        }

        generateFilledRectangle ( cv::Point2f ( 10, 10 ),
          WIDTH - 20,
          HEIGHT - 20,
          &interpolationMethod2);

        // Uncomment for visual inspection
        /*
         *Visualization::showScaled
         *  ( "interpolationMethod2", interpolationMethod2, 0);
         */

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
    and height of @param y. All points inside the rectangle have a value of 0.0
    @param[in] upperLeft [const cv::Point2f&] The upper left vertex of the
    rectangle to be created
    @param[in] x [const int&] The rectangle's width
    @param[in] y [const int&] The rectangle's height
    @param[out] image [cv::Mat*] The image on which the rectangle will be
    imprinted on. Its size must be set before calling this method
    return void
   **/
  void NoiseEliminationTest::generateFilledRectangle (
    const cv::Point2f& upperLeft,
    const int& x,
    const int& y,
    cv::Mat* image )
  {
    if ( image->type() != CV_32FC1 )
    {
      std::cerr << "Image of invalid type. Please use CV_32FC1" << std::endl;

      return;
    }

    // Fill the inside of the desired rectangle with the @param value provided
    for( int rows = upperLeft.y; rows < upperLeft.y + y; rows++ )
    {
      for ( int cols = upperLeft.x; cols < upperLeft.x + x; cols++ )
      {
        image->at< float >( rows, cols ) = 0.0;
      }
    }
  }



  //! Test NoiseElimination::brushfireNearStep
  TEST_F ( NoiseEliminationTest, BrushfireNearStepTest )
  {

    // Uncomment for visual inspection
    /*
     *Visualization::showScaled
     *  ( "before brushfireNearStep", interpolationMethod1, 0);
     */

    // Run NoiseElimination::brushfireNearStep on image interpolationMethod1
    NoiseElimination::brushfireNearStep
      ( &interpolationMethod1, 200 * WIDTH + 100 );

    // Uncomment for visual inspection
    /*
     *Visualization::showScaled
     *  ( "after brushfireNearStep", interpolationMethod1, 0);
     */

    // All pixels of interpolationMethod1 should now have a value of 0.5
    float sum = 0.0;
    for ( int rows = 0; rows < interpolationMethod1.rows; rows++ )
    {
      for ( int cols = 0; cols < interpolationMethod1.cols; cols++ )
      {
        sum += interpolationMethod1.at< float >( rows, cols );
      }
    }

    ASSERT_EQ ( 0.5 * ( static_cast< float >( WIDTH * HEIGHT ) - 100 ), sum );
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
