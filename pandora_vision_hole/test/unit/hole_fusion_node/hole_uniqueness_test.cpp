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

#include "hole_fusion_node/hole_uniqueness.h"
#include "utils/visualization.h"
#include "gtest/gtest.h"

namespace pandora_vision
{
  /**
    @class HoleUniquenessTest
    @brief Tests the integrity of methods of class HoleUniqueness
   **/
  class HoleUniquenessTest : public ::testing::Test
  {
    protected:

      HoleUniquenessTest(){}

      virtual void SetUp(){}
  };

  //! Tests HoleUniqueness::makeHolesUnique (a)
  TEST_F ( HoleUniquenessTest, makeHolesUniqueA )
  {
    // The container of holes
    BlobVector conveyor;

    // A container of one hole
    pandora_vision_msgs::Blob hole_1;

    // Construct hole_1
    hole_1.areaOfInterest.center.x = 100.0;
    hole_1.areaOfInterest.center.y = 100.0;

    hole_1.areaOfInterest.width = 41.0;
    hole_1.areaOfInterest.height = 41.0;

    hole_1.outline.push_back(MessageConversions::cvToMsg(cv::Point2f(80.0, 80.0)));
    hole_1.outline.push_back(MessageConversions::cvToMsg(cv::Point2f(80.0, 120.0)));
    hole_1.outline.push_back(MessageConversions::cvToMsg(cv::Point2f(120.0, 80.0)));
    hole_1.outline.push_back(MessageConversions::cvToMsg(cv::Point2f(120.0, 120.0)));

    // A container of another hole
    pandora_vision_msgs::Blob hole_2;

    // Construct hole_2
    hole_2.areaOfInterest.center.x = 90.0;
    hole_2.areaOfInterest.center.y = 90.0;

    hole_2.areaOfInterest.width = 61.0;
    hole_2.areaOfInterest.height = 61.0;

    hole_2.outline.push_back(MessageConversions::cvToMsg(cv::Point2f(70.0, 70.0)));
    hole_2.outline.push_back(MessageConversions::cvToMsg(cv::Point2f(70.0, 130.0)));
    hole_2.outline.push_back(MessageConversions::cvToMsg(cv::Point2f(130.0, 70.0)));
    hole_2.outline.push_back(MessageConversions::cvToMsg(cv::Point2f(130.0, 130.0)));

    // A container of a third hole
    pandora_vision_msgs::Blob hole_3;

    // Construct hole_3
    hole_3.areaOfInterest.center.x = 80.0;
    hole_3.areaOfInterest.center.y = 80.0;

    hole_3.areaOfInterest.width = 61.0;
    hole_3.areaOfInterest.height = 61.0;

    hole_3.outline.push_back(MessageConversions::cvToMsg(cv::Point2f(60.0, 60.0)));
    hole_3.outline.push_back(MessageConversions::cvToMsg(cv::Point2f(60.0, 120.0)));
    hole_3.outline.push_back(MessageConversions::cvToMsg(cv::Point2f(120.0, 60.0)));
    hole_3.outline.push_back(MessageConversions::cvToMsg(cv::Point2f(120.0, 120.0)));

    // Holes 1 and 2 will be multiple, in a random order.
    conveyor.append( hole_1 );
    conveyor.append( hole_2 );
    conveyor.append( hole_3 );
    conveyor.append( hole_2 );
    conveyor.append( hole_2 );
    conveyor.append( hole_1 );

    // Run HoleUniqueness::makeHolesUnique (a)
    HoleUniqueness::makeHolesUnique( &conveyor );

    // There should be only three unique holes inside the container
    EXPECT_EQ ( 3, conveyor.size() );

    EXPECT_EQ ( 80.0, conveyor.getBlob(0).areaOfInterest.center.x );
    EXPECT_EQ ( 80.0, conveyor.getBlob(0).areaOfInterest.center.y );

    EXPECT_EQ ( 100.0, conveyor.getBlob(1).areaOfInterest.center.x );
    EXPECT_EQ ( 100.0, conveyor.getBlob(1).areaOfInterest.center.y );

    EXPECT_EQ ( 90.0, conveyor.getBlob(2).areaOfInterest.center.x );
    EXPECT_EQ ( 90.0, conveyor.getBlob(2).areaOfInterest.center.y );
  }

  //! Tests HoleUniqueness::makeHolesUnique (b)
  TEST_F ( HoleUniquenessTest, makeHolesUniqueB )
  {
    // What one needs to do here, is to construct a swarm of holes
    // around some point and give each hole a validity probability.

    // First swarm; swarm a
    pandora_vision_msgs::Blob hole_a1;
    pandora_vision_msgs::Blob hole_a2;
    pandora_vision_msgs::Blob hole_a3;

    hole_a1.areaOfInterest.center.x = 100.0;
    hole_a2.areaOfInterest.center.x = 100.0;
    hole_a3.areaOfInterest.center.x = 100.0;
    
    hole_a1.areaOfInterest.center.y = 100.0;
    hole_a2.areaOfInterest.center.y = 100.0;
    hole_a3.areaOfInterest.center.y = 100.0;

    hole_a1.areaOfInterest.width = 41.0;
    hole_a1.areaOfInterest.height = 41.0;

    hole_a1.outline.push_back(MessageConversions::cvToMsg(cv::Point2f(80.0, 80.0)));
    hole_a1.outline.push_back(MessageConversions::cvToMsg(cv::Point2f(120.0, 80.0)));
    hole_a1.outline.push_back(MessageConversions::cvToMsg(cv::Point2f(80.0, 120.0)));
    hole_a1.outline.push_back(MessageConversions::cvToMsg(cv::Point2f(120.0, 120.0)));


    hole_a2.areaOfInterest.width = 51.0;
    hole_a2.areaOfInterest.height = 51.0;

    hole_a2.outline.push_back(MessageConversions::cvToMsg(cv::Point2f(70.0, 70.0)));
    hole_a2.outline.push_back(MessageConversions::cvToMsg(cv::Point2f(120.0, 70.0)));
    hole_a2.outline.push_back(MessageConversions::cvToMsg(cv::Point2f(70.0, 120.0)));
    hole_a2.outline.push_back(MessageConversions::cvToMsg(cv::Point2f(120.0, 120.0)));


    hole_a3.areaOfInterest.width = 61.0;
    hole_a3.areaOfInterest.height = 61.0;

    hole_a3.outline.push_back(MessageConversions::cvToMsg(cv::Point2f(70.0, 70.0)));
    hole_a3.outline.push_back(MessageConversions::cvToMsg(cv::Point2f(130.0, 70.0)));
    hole_a3.outline.push_back(MessageConversions::cvToMsg(cv::Point2f(70.0, 130.0)));
    hole_a3.outline.push_back(MessageConversions::cvToMsg(cv::Point2f(130.0, 130.0)));

    // Second swarm; swarm b
    pandora_vision_msgs::Blob hole_b1;
    pandora_vision_msgs::Blob hole_b2;
 
    hole_b1.areaOfInterest.center.x = 200.0;
    hole_b2.areaOfInterest.center.x = 200.0;

    hole_b1.areaOfInterest.center.y = 200.0;
    hole_b2.areaOfInterest.center.y = 200.0;

    hole_b1.areaOfInterest.width = 41.0;
    hole_b1.areaOfInterest.height = 41.0;

    hole_b1.outline.push_back(MessageConversions::cvToMsg(cv::Point2f(180.0, 180.0)));
    hole_b1.outline.push_back(MessageConversions::cvToMsg(cv::Point2f(220.0, 180.0)));
    hole_b1.outline.push_back(MessageConversions::cvToMsg(cv::Point2f(180.0, 220.0)));
    hole_b1.outline.push_back(MessageConversions::cvToMsg(cv::Point2f(220.0, 220.0)));


    hole_b2.areaOfInterest.width = 131.0;
    hole_b2.areaOfInterest.height = 131.0;

    hole_b2.outline.push_back(MessageConversions::cvToMsg(cv::Point2f(150.0, 150.0)));
    hole_b2.outline.push_back(MessageConversions::cvToMsg(cv::Point2f(150.0, 280.0)));
    hole_b2.outline.push_back(MessageConversions::cvToMsg(cv::Point2f(280.0, 150.0)));
    hole_b2.outline.push_back(MessageConversions::cvToMsg(cv::Point2f(280.0, 280.0)));

    // Add another, unique, hole
    pandora_vision_msgs::Blob hole;

    hole.areaOfInterest.center.x = 300.0;
    hole.areaOfInterest.center.y = 300.0;

    hole.areaOfInterest.width = 41.0;
    hole.areaOfInterest.height = 41.0;

    hole.outline.push_back(MessageConversions::cvToMsg(cv::Point2f(280.0, 280.0)));
    hole.outline.push_back(MessageConversions::cvToMsg(cv::Point2f(320.0, 280.0)));
    hole.outline.push_back(MessageConversions::cvToMsg(cv::Point2f(280.0, 320.0)));
    hole.outline.push_back(MessageConversions::cvToMsg(cv::Point2f(320.0, 320.0)));

    // Push all the holes back into a container
    BlobVector conveyor;
    conveyor.append( hole_a1 );
    conveyor.append( hole_a2 );
    conveyor.append( hole_a3 );
    conveyor.append( hole_b1 );
    conveyor.append( hole_b2 );
    conveyor.append( hole );

    // Construct a validity map
    std::map< int, float > validityMap;
    validityMap[0] = 0.9;
    validityMap[1] = 0.8;
    validityMap[2] = 0.91;
    validityMap[3] = 0.9;
    validityMap[4] = 0.8;
    validityMap[5] = 0.9;

    // Run HoleUniqueness::makeHolesUnique (b)
    HoleUniqueness::makeHolesUnique( &conveyor, &validityMap );

    // There should be three unique holes
    ASSERT_EQ( 3, conveyor.size() );

    std::map< int, float >::iterator it = validityMap.begin();

    EXPECT_NEAR( 0.91, it->second, 0.01 );

    it++;
    EXPECT_NEAR( 0.9, it->second, 0.01 );

    it++;
    EXPECT_NEAR( 0.9, it->second, 0.01 );

    // Inquire about the internals of the unique holes
    EXPECT_NEAR( 100.0, conveyor.getBlob(0).areaOfInterest.center.x, 0.01 );
    EXPECT_NEAR( 100.0, conveyor.getBlob(0).areaOfInterest.center.y, 0.01 );

    std::vector<cv::Point2f> vec = MessageConversions::areaToVec(
      conveyor.getBlob(0).areaOfInterest);

    EXPECT_NEAR( 70.0, vec[0].x, 0.01 );
    EXPECT_NEAR( 70.0, vec[0].y, 0.01 );
    EXPECT_NEAR( 70.0, vec[1].x, 0.01 );
    EXPECT_NEAR( 130.0, vec[1].y, 0.01 );
    EXPECT_NEAR( 130.0, vec[2].x, 0.01 );
    EXPECT_NEAR( 130.0, vec[2].y, 0.01 );
    EXPECT_NEAR( 130.0, vec[3].x, 0.01 );
    EXPECT_NEAR( 70.0, vec[3].y, 0.01 );


    EXPECT_NEAR( 200.0, conveyor.getBlob(1).areaOfInterest.center.x, 0.01 );
    EXPECT_NEAR( 200.0, conveyor.getBlob(1).areaOfInterest.center.y, 0.01 );

    vec = MessageConversions::areaToVec(conveyor.getBlob(1).areaOfInterest);

    EXPECT_NEAR( 180.0, vec[0].x, 0.01 );
    EXPECT_NEAR( 180.0, vec[0].y, 0.01 );
    EXPECT_NEAR( 180.0, vec[1].x, 0.01 );
    EXPECT_NEAR( 220.0, vec[1].y, 0.01 );
    EXPECT_NEAR( 220.0, vec[2].x, 0.01 );
    EXPECT_NEAR( 220.0, vec[2].y, 0.01 );
    EXPECT_NEAR( 220.0, vec[3].x, 0.01 );
    EXPECT_NEAR( 180.0, vec[3].y, 0.01 );


    EXPECT_NEAR( 300.0, conveyor.getBlob(2).areaOfInterest.center.x, 0.01 );
    EXPECT_NEAR( 300.0, conveyor.getBlob(2).areaOfInterest.center.y, 0.01 );

    vec = MessageConversions::areaToVec(conveyor.getBlob(2).areaOfInterest);

    EXPECT_NEAR( 280.0, vec[0].x, 0.01 );
    EXPECT_NEAR( 280.0, vec[0].y, 0.01 );
    EXPECT_NEAR( 280.0, vec[1].x, 0.01 );
    EXPECT_NEAR( 320.0, vec[1].y, 0.01 );
    EXPECT_NEAR( 320.0, vec[2].x, 0.01 );
    EXPECT_NEAR( 320.0, vec[2].y, 0.01 );
    EXPECT_NEAR( 320.0, vec[3].x, 0.01 );
    EXPECT_NEAR( 280.0, vec[3].y, 0.01 );
  }

}  // namespace pandora_vision
