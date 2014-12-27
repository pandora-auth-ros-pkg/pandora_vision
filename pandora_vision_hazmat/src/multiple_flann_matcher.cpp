#include "pandora_vision_hazmat/multiple_flann_matcher.h"

MultipleFlannMatcher::MultipleFlannMatcher(HazmatDetector* detectorPtr)
  : detector(detectorPtr)
{
  std::cout << "Starting to construct the multiple FLANN matcher"
    << std::endl;
    
  // Get the matcher used by the single matcher
  cv::FlannBasedMatcher baseMatcher = detector->getMatcher();
  
  
  
  
  // Request the number of patterns we want to detect.
  int patternNumber = detector->getPatternsNumber() ;
  
  // Get a pointer to the data
  const std::vector<Pattern> *data = detector->getPatternsPtr();
  
  // Create the array of the matchers.
  matchers_ = new cv::Ptr<cv::FlannBasedMatcher>[patternNumber] ;

  // Temporary container for the descriptors of each pattern.
  std::vector<cv::Mat> descriptors;
  
  // Create a matcher for every 
  for (int i = 0 ; i < patternNumber ; i++ )
  {
    // Create a matcher of the same type as the base matcher.
    matchers_[i] = baseMatcher.clone(true);
   
    // Convert the array of descriptors to a std::vector of cv 
    // matrices for the training function.

    if ( !matchers_[i]->empty() ) 
    {
      std::cout << "Matcher " << i << " is not empty !" << std::endl;
      matchers_[i]->clear();
    }
 
    descriptors.push_back( (*data)[i].descriptors ) ;
   
    // Add the descriptors to the matcher.
    matchers_[i]->add( descriptors );

    // Train the matcher for the data of the i-th pattern.
    matchers_[i]->train();
    
    // Clear the descriptor vector for the next iteration.
    descriptors.clear();
  }
  
  std::cout << "Finished construncting multiple FLANN matcher!" 
    << std::endl ;
  
  } // End of Multiple FLANN constructor.


// Find the keypoint matches between the frame and the training set.
bool MultipleFlannMatcher::findKeypointMatches(
      const cv::Mat &frameDescriptors ,
      const cv::Mat &patternDescriptors , 
      const std::vector<cv::Point2f> patternKeyPoints ,
      const std::vector<cv::KeyPoint> sceneKeyPoints ,
      std::vector<cv::Point2f> *matchedPatternKeyPoints , 
      std::vector<cv::Point2f> *matchedSceneKeyPoints  , 
      const int &patternID )
{
  std::cout << "hi \n";
  exit(-1);
  
  // Define the vector that will contain the matches.
  std::vector< std::vector<cv::DMatch> > matches;
  
  // Perfom the matching using the matcher that corresponds
  // to the i-th pattern.
  
  matchers_[patternID]->knnMatch( frameDescriptors , matches , 2 );
  
  std::vector< cv::DMatch > goodMatches ;
  
  
  if ( matches.size() > 0 )
  {

    // Keep only the matches that are below a certain distance 
    // threshold
    // TO DO : READ THE THRESHOLD FROM FILE.
    
    float ratio = 0.8 ;
    
    for( int i = 0; i <  matches.size() ; i++ )
    { 
        if( matches[i][0].distance < ratio*matches[i][1].distance )
        { 
           goodMatches.push_back( matches[i][0]); 
        }
    }
    
  }
  // No matches found.
  else
    return false;
  
  // Add the keypoints of the matches found to the corresponding
  // vectors for the pattern and the scene.
  for( int i = 0; i < goodMatches.size(); i++ )
  {
    
    // Pattern key points .
    matchedPatternKeyPoints->push_back( 
      patternKeyPoints[ goodMatches[i].queryIdx ] );
      
    // Scene key points .
    matchedSceneKeyPoints->push_back( 
      sceneKeyPoints[ goodMatches[i].trainIdx ].pt );
  }
  
  // If we have less than 4 matches then we cannot find the Homography
  // and this is an invalid pattern.
  
  if ( goodMatches.size() < 4 )
    return false;
    
  
  goodMatches.clear();
  matches.clear();
  
  // Matches have been successfully found and keypoints correctly 
  // assigned.
  return true;  
  
  
  }
