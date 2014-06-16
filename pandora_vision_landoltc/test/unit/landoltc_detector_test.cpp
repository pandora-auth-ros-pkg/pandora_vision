#include "pandora_vision_landoltc/landoltc_detector.h"
#include "gtest/gtest.h"

namespace pandora_vision {

// The fixture for testing class LandoltC
class LandoltCTest : public ::testing::Test {
 protected:
  // You can remove any or all of the following functions if its body
  // is empty.
  
  
  
  

  //~ LandoltCTest() {
    //~ // You can do set-up work for each test here.
  //~ }
//~ 
  //~ virtual ~LandoltCTest() {
    //~ // You can do clean-up work that doesn't throw exceptions here.
  //~ }

  // If the constructor and destructor are not enough for setting up
  // and cleaning up each test, you can define the following methods:
  
  virtual void SetUp() {
    
  frame = cv::Mat(640,480,CV_8UC3,cv::Scalar(255,255,255));
  
  cv::circle(frame, cv::Point(320, 240), 10, cv::Scalar(0, 0, 0), 3);
  
  cv::cvtColor(frame, gray, CV_BGR2GRAY);
  
  cv::Sobel(gray, gradX, CV_32F, 1, 0, 3);
  
  cv::Sobel(gray, gradY, CV_32F, 0, 1, 3);
  
  float* gradXF = reinterpret_cast<float*>(gradX.data);
  
  float* gradYF = reinterpret_cast<float*>(gradY.data);    
  }
  
  cv::Mat frame,gray, gradX, gradY;
  float* gradXF;
  float* gradYF;
  
  // Objects declared here can be used by all tests in the test case for Foo.
};

TEST_F(LandoltCTest, findCentersTest) {
  
  //~ LandoltCDetector landoltc;
  
  
}

}

