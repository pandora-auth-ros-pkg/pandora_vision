#include "pandora_vision_hazmat/hazmat_detector.h"
#include "gtest/gtest.h"

class HazmatFilterTest : public ::testing::Test
{
  protected:
  HazmatFilterTest () {}
  
  
  //~ testMatrixDivision(cv::Mat)
  };
  
  
  TEST( HistogramMask , normImage )
  {
    Detector *detector = new HazmatDetector();
    HistogramMask mask(detector);
    // Create a 3-channel white image
    cv::Mat whiteImage( 20 , 20 , CV_8UC3 );
    // Set every element to its maximum possible value.
    whiteImage.setTo( 255 );
    
    // Convert the image to YUV format.
    cv::Mat yuvWhite;
    cvtColor( whiteImage , yuvWhite , CV_BGR2YCrCb );
    
    // Declare the normalized white image matrix.
    cv::Mat normWhiteImage;
    
    // Call the function and normalized the white image according to the
    // Y channel.
    HistogramMask::normalizeImage( yuvWhite , &normWhiteImage , 1 );
    
    //~ // For every Channel.
    //~ for ( ch = 0 ; ch < normWhiteImage.channels() ; ch++ )
    //~ {
      //~ // For every row
      //~ for ( int i = 0 ; i < normWhiteImage.rows ; i++ )
      //~ {
        //~ // For every column o
        //~ for ( int j = 0 ; j < normWhiteImage.cols ; j++ )
        //~ {
          //~ if ( whiteImage.at<)
        //~ }
      //~ }
    //~ }
    
    
  }
