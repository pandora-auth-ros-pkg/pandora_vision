#include "pandora_vision_hazmat/filters.h"
#include "pandora_vision_hazmat/HazmatDetector.h"
#include "gtest/gtest.h"


  
  
  TEST( HazmatHistogramTest , imageNormalization )
  {
    
    
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
    HistogramMask::normalizeImage( yuvWhite , &normWhiteImage , 0 );
    //~ HistogramMask::calcNormYUVHist( yuvWhite , &normWhiteImage );
    
    // Test if the input image and the output image are of the same 
    // type/depth .
    ASSERT_EQ( whiteImage.type() , normWhiteImage.type() ) ;
    
    // Check if the input and the output image have the same number 
    // of channels.
    ASSERT_EQ( whiteImage.channels() , normWhiteImage.channels() ) ;
    
    // Check if the input image has the same number of rows as the 
    // output
    ASSERT_EQ( whiteImage.rows , normWhiteImage.rows ) ;
    
    
    // Check if the input image has the same number of cols as the 
    // output
    ASSERT_EQ( whiteImage.cols , normWhiteImage.cols ) ;
    
    // Check if the output matrix has the correct elements.
    // Since this is a white image , it's Cr and Cb values are equal to
    // 128 (see OpenCV cvtColor Documentation ) and it's Y value is
    // equal to 255.
    
    
    
    for (int ch = 0 ; ch < whiteImage.channels() ; ch++)    
    {
      for (int i = 0 ; i < whiteImage.rows ; i++)
      {
        for (int j = 0 ; j < whiteImage.cols ; j++ )
        {
          int value ;
          switch (ch)
          {
            case 0:
              value = 255;
              break;
            case 1:
            case 2:
              value = 128;
              break;
          }
          
          ASSERT_EQ( value , static_cast<int> ( 
            normWhiteImage.at<cv::Vec3b>(i,j)[ch] ) ) ;
        }
      }
    }
    
    
    
  }
  
  TEST( HazmatHistogramTest , normalizedHistogramTest)
  {
    // Create a 3-channel white image
    cv::Mat whiteImage( 20 , 20 , CV_8UC3 );
    // Set every element to its maximum possible value.
    whiteImage.setTo( 255 );
    
    // Convert the image to YUV format.
    cv::Mat yuvWhite;
    cvtColor( whiteImage , yuvWhite , CV_BGR2YCrCb );
    
    cv::Mat hist ;
    
    // Create the Normalized YUV histogram of the image.
    HistogramMask::calcNormYUVHist( yuvWhite , &hist );
    
    // Since this is a white image all of its elements belong to 
    // the final bin of the histogram.
    int matElements = whiteImage.rows * whiteImage.cols;
    
    // Check that every element of the histogram is zero except
    // the one that corresponds to a white color.
    
    // For every row :
    for (int i = 0 ; i < hist.rows ; i++)
    {
      // For every column : 
      for ( int j = 0 ; j < hist.cols ; j++ )
      {
        // Get the value 
        int value = static_cast<int>( hist.at<uchar>(i,j) ) ;
        if (j != hist.cols  || i != hist.rows  )
          ASSERT_EQ( 0 ,  value )  ;
        else
          ASSERT_EQ( matElements , value ) ;
          
      }
    }
    
    }
