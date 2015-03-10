#include "pandora_vision_hazmat/image_signature.h"
#include "pandora_vision_hazmat/hazmat_detector.h"
#include "gtest/gtest.h"

 
  TEST( ImageSignatureTest , imageSign )
  {
    // Create a small image with positive elements .
    cv::Mat testImage( 20 , 20 , CV_32SC3 );
    testImage.setTo(100);
    
    cv::Mat signs;
        
    ImageSignature::signFunction(testImage , &signs );
    
    // Since we pass a multi channel matrix the function must return
    // an empty matrix.
    ASSERT_TRUE(signs.data == NULL);
    
    // Create a matrix with positive elements.
    testImage = cv::Mat(20,20, CV_32FC1 );
    testImage.setTo(100);
    
    ImageSignature::signFunction(testImage , &signs );
    
    // Check that every value is positive.

    for (int i = 0 ; i < testImage.rows ; i++)
    {
      for (int j = 0 ; j < testImage.cols ; j++ )
      {
        int val = signs.at<float>(i,j) ;
        ASSERT_TRUE(  val > 0  ) ;
      }
    }
    
    
    // Create a matrix with positive elements.
    testImage = cv::Mat(20,20, CV_32FC1 );
    testImage.setTo(-100);
    
    ImageSignature::signFunction(testImage , &signs );
    
    // Check that every value is negative.

    for (int i = 0 ; i < testImage.rows ; i++)
    {
      for (int j = 0 ; j < testImage.cols ; j++ )
      {
        int val = signs.at<float>(i,j) ;
        ASSERT_TRUE(  val <  0  ) ;
      }
    }
    
    // Create an array with random numbers.
    //~ cv::randu(testImage , -100 , 100 );
    
    ImageSignature::signFunction(testImage , &signs );
   
    cv::Mat result( testImage.size() , testImage.type() );
    
    // Multiply the signs with the original image.
    // If there is a sign mismatch then the sign function did not 
    // return the correct sign since :
    // positive * positive = positive 
    // negative * negative = negative 
    // 0 * anything = 0 .
    
    result = signs.mul( testImage  );
    
    for (int i = 0 ; i < testImage.rows ; i++)
    {
      for (int j = 0 ; j < testImage.cols ; j++ )
      {
        int val = result.at<float>(i,j) ;
        ASSERT_GE(  val  ,   0  ) ;
      }
    }
    
  }
  
