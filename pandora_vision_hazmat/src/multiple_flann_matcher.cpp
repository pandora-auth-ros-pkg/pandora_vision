#include "pandora_vision_hazmat/multiple_flann_matcher.h"

MultipleFlannMatcher::MultipleFlannMatcher(HazmatDetector* detectorPtr)
  : detector(detectorPtr)
{
  // Get the matcher used by the single matcher
  cv::FlannBasedMatcher baseMatcher = detector->getMatcher();
  
  // Request the number of patterns we want to detect.
  int patternNumber = detector->getPatternsNumber() ;
  
  // Get a pointer to the data
  const std::vector<Pattern> data = detector->getPattersRef();
  
  // Create the array of the matchers.
  matchers = new cv::DescriptorMatcher*[patternNumber] ;
  
  // Temporary container for the descriptors of each pattern.
  std::vector<cv::Mat> descriptors;
  
  // Create a matcher for every 
  for (int i = 0 ; i < patternNumber ; i++ )
  {
    // Create a matcher of the same type as the base matcher.
    matchers[i] = baseMatcher.clone(true);
    
    // Convert the array of descriptors to a std::vector of cv 
    // matrices for the training function.
    for (int j = 0 ; j < data[i].descriptors.rows ; i++ )
      descriptors.push_back( data[i].descriptors.row(j) );
    
    // Add the descriptors to the matcher.
    matchers[i]->add( descriptors );
    
    // Train the matcher for the data of the i-th pattern.
    matchers[i]->train();
    
    // Clear the descriptor vector for the next iteration.
    descriptors.clear();
  }
  
  
  }
