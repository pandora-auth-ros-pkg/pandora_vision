#include "pandora_vision_hazmat/sift_hazmat_detector.h"

/** 
 * Sift detector 
**/

// SIFT hazmat detector constructor
SiftHazmatDetector::SiftHazmatDetector() : 
  SimpleHazmatDetector("SIFT") 
{
  int patternNum = this->getPatternsNumber();

  // Initialize the matchers that will be used for the 
  // detection of the pattern.
  matchers_ = new cv::Ptr<cv::DescriptorMatcher>[patternNum];
  
  // A temporary container for the descriptors of each pattern.
  std::vector<cv::Mat> descriptors;

  for (int i = 0 ; i < patternNum ; i++ )
  {
    matchers_[i] = cv::DescriptorMatcher::create("FlannBased");
    // Add the descriptors of the i-th pattern to the 
    // container.
    descriptors.push_back( (*patterns_ )[i].descriptors );
    // Add the descriptors to the matcher and train it.
    matchers_[i]->add( descriptors );
    matchers_[i]->train();

    // Clear the container for the next iteration.
    descriptors.clear();
  }

  // Initialize the keypoint detector and the feature extractor
  // that will be used.
  s_ = cv::SIFT() ; 
}

void SiftHazmatDetector::getFeatures( const cv::Mat &frame , 
  const cv::Mat &mask , cv::Mat *descriptors , 
  std::vector<cv::KeyPoint> *keyPoints ) 
{
  #ifdef FEATURES_CHRONO
  gettimeofday( &startwtime , NULL );
  #endif
  // Detect the Keypoints on the image.
  s_.detect( frame ,  *keyPoints ,  mask );
  #ifdef FEATURES_CHRONO
  gettimeofday( &endwtime, NULL );
  double keyPointTime = (double)((endwtime.tv_usec - startwtime.tv_usec)
      /1.0e6 + endwtime.tv_sec - startwtime.tv_sec);
  std::cout << "Keypoint Extraction time : " << keyPointTime << std::endl;
  #endif
  // Extract descriptors for the the detected keypoints.
  #ifdef FEATURES_CHRONO
  gettimeofday( &startwtime , NULL );
  #endif
  s_.compute( frame, *keyPoints , *descriptors);
  #ifdef FEATURES_CHRONO
  gettimeofday( &endwtime, NULL );
  double descriptorsTime = (double)((endwtime.tv_usec - startwtime.tv_usec)
      /1.0e6 + endwtime.tv_sec - startwtime.tv_sec);
  std::cout << "Descriptors Computation time : " << descriptorsTime 
    << std::endl;
  #endif
  }


