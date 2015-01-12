#include "pandora_vision_hazmat/hazmat_detector.h"
#include "pandora_vision_hazmat/sift_hazmat_detector.h"
//#include "pandora_vision_hazmat/multiple_flann_matcher.h"
#include "pandora_vision_hazmat/surf_hazmat_detector.h"
#include "pandora_vision_hazmat/orb_hazmat_detector.h"
int main(int argc , char **argv )
{
  SiftHazmatDetector detectorObj ;
  // SurfHazmatDetector detectorObj ;
  // OrbHazmatDetector detectorObj ; 
  //~ Detector *detector = new HistogramMask(detectorObj);
  //~ HistogramMask detector(&sign);
  //~ HistogramMask detector(&detectorObj);
//  MultipleFlannMatcher detector(&detectorObj);
  
  cv::VideoCapture camera(1);
  
  if ( !camera.isOpened() )
  {
    std::cerr << "Error Opening the camera ! " << std::endl;
    return -1;
  }
  
  
  cv::Mat frame;
  cv::Mat hist;
  cv::Mat mask;
  cv::Mat maskedFrame;
  std::stringstream ss;
  float x , y;
  
  int count = 1 ;
  
  while(true)
  {
    
    camera.grab();
    camera.retrieve(frame);
    
    
    if (count == 1 )
    {
      count++;
      HazmatDetector::setDims(frame); 
    }
    
    
    const clock_t begin_time = clock();

    bool found = detectorObj.detect(frame , &x, &y);
    // bool found = detector.detect(frame , &x, &y);
    double execTime = ( clock () - begin_time ) /  
      static_cast<double>(CLOCKS_PER_SEC );
    std::cout <<"Time to execute : " << execTime << std::endl; 
    if (found && x > 0 && y > 0  )
    {
      //~ cv::line( frame, scene_corners[0] , scene_corners[1] , Scalar(0, 255, 0), 4 );
      //~ cv::line( frame, scene_corners[1] , scene_corners[2] , Scalar( 0, 255, 0), 4 );
      //~ cv::line( frame, scene_corners[3] , scene_corners[0] , Scalar( 0, 255, 0), 4 );
      //~ cv::line( frame, scene_corners[2] , scene_corners[3] , Scalar( 0, 255, 0), 4 );
      cv::circle( frame , cv::Point2f(x,y)  , 4.0 , cv::Scalar(0,0,255) , -1 , 8 );
      
    }
    if ( !frame.data )
    {
      std::cout << "Invalid Frame. Continuing to next iteration!" 
        << std::endl;
      continue;
    }
    
    //~ HistogramMask::normalizeImage(frame , &frame , 0);
    //~ cv::cvtColor(frame,saliency,CV_BGR2Lab);
    //~ cv::cvtColor(frame,saliency,CV_BGR2GRAY);

    //~ detector.createMask( frame , &mask);
    //~ detector.createMask( frame , &mask , hist );
    
    //~ if ( ! mask.data )
    //~ {
      //~ std::cerr << "Invalid Mask " << std::endl;
      //~ // Release the camera .
      //~ camera.release();
      //~ break;
    //~ }
    
    //~ std::cout <<"TIme to execute : " << ( clock () - begin_time ) /  
      //~ static_cast<double>(CLOCKS_PER_SEC )<< std::endl;
    ss << 1 / execTime ;

    cv::putText( frame , "fps : " + ss.str() , cv::Point( 0 , frame.rows ) ,
        CV_FONT_HERSHEY_PLAIN , 3 , cv::Scalar( 0 , 0 , 255 ) , 3 ) ;


    // Clear the string stream.
    ss.str( std::string() ) ;
    
    cv::imshow("Frame",frame);
    //~ cv::imshow("Mask" , mask);
    //~ frame.copyTo(maskedFrame,mask);
    //~ cv::imshow("Segmented Frame",maskedFrame);

    if (cv::waitKey(30)>=0)
    { 
      // Release the camera.
      camera.release();
      break;
    }
    
    // Reset the contents of the mask and the Segmented Frame.
    //~ mask.setTo(0);
    //~ maskedFrame.setTo(0);
  }

  
  
  }
