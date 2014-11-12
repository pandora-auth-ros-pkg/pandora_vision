#include "pandora_vision_hazmat/hazmat_detector.h"

/**
  @brief Creates a mask that defines the region of the frame where
        features will be extracted.
  @param frame [const cv::Mat &] : The input frame.
  @param mask [cv::Mat *] : The output mask.
  @param data [cv::Mat &] : An array with extra data.  
  
**/
  
void HazmatDetector::createMask(const cv::Mat &frame , cv::Mat *mask , 
      const cv::Mat &data  )
{
  if ( mask->data)
    // If another mask has already been created do not change it.
    return ;
  else
    // If not then initialize an empty matrix so that the feature
    // search is performed on the entire frame. 
    *mask = cv::Mat( frame.size() , frame.type() );
  }

// Allocating space for the static member variable.
std::string HazmatDetector::fileName_ = std::string("input") ;


// Data input function . 

void HazmatDetector::readData( void )
{
  // Open the file for reading .
  cv::FileStorage fs( fileName_ + ".xml" ,cv::FileStorage::READ);
  std::cout << fileName_ + ".xml" << std::endl;
  // Check if the file was opened succesfully .
  if ( !fs.isOpened() )
  {
    std::cerr << "XML file could not be opened!" << std::endl;
    return ;
  }
  
  // Go to the xml node that contains the pattern names.
  cv::FileNode inputNames = fs["PatternName"];
  
  // Check if the node has a sequence.
  if ( inputNames.type() != cv::FileNode::SEQ)
    {
        std::cerr << "Input data  is not a string sequence! FAIL" 
          << std::endl;
        return ;
    }
  
  // Initialize File iterator.
  cv::FileNodeIterator it = inputNames.begin() ;
  cv::FileNodeIterator itEnd = inputNames.end() ;
  std::string inputName ;
  std::vector<std::string> input;
  
  // Iterate over the node and get the names.
  for ( ; it != itEnd ; ++it ) 
  {
   inputName = (std::string)(*it)["name"]  ;
   input.push_back(inputName) ;
  }
  
  // Close the file with the pattern names.
  fs.release() ;
  
  // For every pattern name read the necessary training data.
  std::string trainingDataDir = "trainingData//" + this->getFeaturesName() ;
  std::string fileName ;
  
  for (int i = 0 ; i < input.size() ; i++) 
  { 
    // Open the training file associated with image #i .
    fileName = trainingDataDir + "//" + input[i] + ".xml" ;
    cv::FileStorage fs2( fileName ,cv::FileStorage::READ);
    
    // Check if the file was properly opened.
    if ( !fs2.isOpened() )
    {
      std::cerr << "File " << fileName << "could not be opened! " 
        << std::endl;
      continue;
    }
    
    std::vector<cv::Point2f> keyPoints;
    std::vector<cv::Point2f> boundingBox;
    cv::Mat descriptors; 
    
    // Read the pattern's descriptors.
    fs2["Descriptors"] >> descriptors;
        
    // Read the pattern' keypoints.
    cv::FileNode keyPointsNode = fs2["PatternKeypoints"];
    
    // Initialize node iterators.
    cv::FileNodeIterator it = keyPointsNode.begin();
    cv::FileNodeIterator itEnd = keyPointsNode.end();
    
    cv::Point2f tempPoint ;
    
    // Iterate over the node to get the keypoints.
    for ( ; it != itEnd ; ++it ) 
    {
      (*it)["Keypoint"] >> tempPoint ;
      keyPoints.push_back(tempPoint);
    }
    
    // Read the pattern's bounding box.
    cv::FileNode boundingBoxNode = fs2["BoundingBox"];
    
    // Initialize it's iterator.
    cv::FileNodeIterator bbIt = boundingBoxNode.begin();
    cv::FileNodeIterator bbItEnd = boundingBoxNode.end();
    
    
    for ( ; bbIt != bbItEnd ; ++bbIt ) 
    {
      (*bbIt)["Corner"] >> tempPoint ;
      boundingBox.push_back(tempPoint);
    }
    
    // Add the pattern to the pattern vector.
    Pattern p;
    p.name = input[i] ;
    p.boundingBox = boundingBox ;
    p.keyPoints = keyPoints;
    p.descriptors = descriptors ;
    
    patterns_.push_back(p);
    
    // Clear the data vectors.
    keyPoints.clear();
    boundingBox.clear(); 
    
    // Close the xml file .
    fs2.release() ;
    
  }    
  
}

// Detect the pattern in the frame.

bool HazmatDetector::detect( const cv::Mat &frame , float *x , 
  float *y ) 
{
  // Check if the frame is not an empty matrix.
  if ( !frame.data )
  {
    std::cerr << "The provided frame is empty!" << std::endl;
    return false;
  }
  
  
  // Set the pattern center to NULL .
  *x = NULL ;
  *y = NULL ;
  
  
  cv::Mat frameDescriptors ;
  std::vector<cv::KeyPoint> frameKeyPoints;
  
  // Calculate the keypoints and extract the descriptors from the 
  // frame.
  getFeatures( frame , &frameDescriptors , &frameKeyPoints ) ; 

  double minDist ; 
  double maxDist ;
  
  // For every pattern in the list : 
  for (int i = 0 ; i < patterns_.size() ; i++ )
  {
    
  }
  
  
  // Find the pattern that matches best the current frame.
  int bestMatchIndex = getBestMatches( frame , frameDescriptors , 
    &minDist , &maxDist );
    
  //~ std::cout << patterns_[bestMatchIndex].name << std::endl;
    
  
  // Vector the contains the matches for whom the distance between
  // the frame descriptors and the pattern descriptors is smaller that
  // 3 * minDist ;
  std::vector< cv::DMatch > goodMatches;
  
  
  // Fill the above vector.
  //~ std::cout << minDist << std::endl;
  //~ std::cout << bestMatches_.size() << std::endl;
  
  if (bestMatches_.size() > 0 )
  {
    for( int i = 0; i < frameDescriptors.rows; i++ )
    { 
      if( bestMatches_[i].distance < 3*minDist )
      { 
         goodMatches.push_back( bestMatches_[i]); 
      }
    }
  }

  
  std::vector<cv::Point2f> patternKeyPoints;
  std::vector<cv::Point2f> sceneKeyPoints;
  
  // Get the keypoints from the good matches

  for( int i = 0; i < goodMatches.size(); i++ )
  {
    
    // Pattern key points .
    patternKeyPoints.push_back( 
      patterns_[bestMatchIndex].keyPoints[ goodMatches[i].queryIdx ] );
      
    // Scene key points .
    sceneKeyPoints.push_back( 
      frameKeyPoints[ goodMatches[i].trainIdx ].pt );
  }
  
  
  // Homography Matrix.
  cv::Mat H ;
  
  // Scene bounding box.
  std::vector<cv::Point2f> sceneBB ;
  //~ std::cout << patternKeyPoints.size() << std::endl;
  
  // If there are sufficient keypoint matches calculate the homography
  // between the pattern and the detected object.
  if ( patternKeyPoints.size() > 4 &&  sceneKeyPoints.size() > 4 )
  {
    // Calculate the homography matrix.
    H = cv::findHomography( patternKeyPoints , sceneKeyPoints, 
      CV_RANSAC );
    // Transfer the bounding box to the frame coordinates.
    cv::perspectiveTransform( patterns_[bestMatchIndex].boundingBox , 
      sceneBB , H );
    
    // Testing Frame
    //~ line( trackingFrame, scene_corners[0] , scene_corners[1] , Scalar(0, 255, 0), 4 );
    //~ line( trackingFrame, scene_corners[1] , scene_corners[2] , Scalar( 0, 255, 0), 4 );
    //~ line( trackingFrame, scene_corners[3] , scene_corners[0] , Scalar( 0, 255, 0), 4 );
    //~ line( trackingFrame, scene_corners[2] , scene_corners[3] , Scalar( 0, 255, 0), 4 );
    //~ circle( trackingFrame , scene_corners[4]  , 4.0 , Scalar(0,0,255) , -1 , 8 );
    
    // Copy the 4 corners of the bounding box.
    std::vector<cv::Point2f> bbCorners ;
    // Iterate over all elements of the bounding box except the last 
    // one which is its center.
    for (int i = 0 ; i < sceneBB.size() - 1 ; i++ )
      bbCorners.push_back( sceneBB[i] );
    if ( !cv::isContourConvex(bbCorners) )
      return false;
      
    int width = frame.cols;
    int height = frame.rows;
      
    // Check if every point of the bounding box is inside the image.
    for (int i = 0 ; i < sceneBB.size() ; i++ )
    {
      if ( ( sceneBB[i].x < 0 ) 
        || ( sceneBB[i].x > width )
        || ( sceneBB[i].y < 0) 
        || ( sceneBB[i].y > height ) )
          return false ;
    }
    
    // If all these conditions are met return the coordinates of the 
    // center of the detected pattern . 
    *x = sceneBB[ sceneBB.size() - 1 ].x ;
    *y = sceneBB[ sceneBB.size() - 1 ].y ;

    return true ;
    
    //~ trackingFrame.release();
  }
  else
    return false;
  
    
    //~ imshow("Tracker",trackingFrame);

    //~ char key =  waitKey(30);
    
}

/**
  * @brief Find the homography between the scene and the pattern keypoints
  * , check if it is valid and return the bounding box of the detected
  * pattern .
  * @param patternKeyPoints [std::vector<cv::KeyPoint> &] : Input 
  * keypoints from detected descriptor matches on the pattern.
  * @param sceneKeyPoints [std::vector<cv::KeyPoint> &] : Input 
  * keypoints from detected descriptor matches in the scene.
  * @param patternBB [std::vector<cv::Point2f *] : Vector of 2D float
  * Points that containes the bounding box and the center of the 
  * pattern.

 **/
  
bool virtual findBoundingBox(std::vector<cv::KeyPoint> &patternKeyPoints , 
      std::vector<cv::KeyPoint> &sceneKeyPoints , 
      std::vector<cv::Point2f> *patternBB) 
{
  // Check if we have enough points to find the homography between
  // the pattern and the scene.
  if ( patternKeyPoints.size() > 4 &&  sceneKeyPoints.size() > 4 )
  {
    // Calculate the homography matrix.
    H = cv::findHomography( patternKeyPoints , sceneKeyPoints, 
      CV_RANSAC );
    // Transfer the bounding box to the frame coordinates.
    cv::perspectiveTransform( patterns_[bestMatchIndex].boundingBox , 
      sceneBB , H );
    
    // Testing Frame
    //~ line( trackingFrame, scene_corners[0] , scene_corners[1] , Scalar(0, 255, 0), 4 );
    //~ line( trackingFrame, scene_corners[1] , scene_corners[2] , Scalar( 0, 255, 0), 4 );
    //~ line( trackingFrame, scene_corners[3] , scene_corners[0] , Scalar( 0, 255, 0), 4 );
    //~ line( trackingFrame, scene_corners[2] , scene_corners[3] , Scalar( 0, 255, 0), 4 );
    //~ circle( trackingFrame , scene_corners[4]  , 4.0 , Scalar(0,0,255) , -1 , 8 );
    
    // Copy the 4 corners of the bounding box.
    std::vector<cv::Point2f> bbCorners ;
    // Iterate over all elements of the bounding box except the last 
    // one which is its center.
    for (int i = 0 ; i < sceneBB.size() - 1 ; i++ )
      bbCorners.push_back( sceneBB[i] );
    if ( !cv::isContourConvex(bbCorners) )
      return false;
      
    int width = frame.cols;
    int height = frame.rows;
      
    // Check if every point of the bounding box is inside the image.
    for (int i = 0 ; i < sceneBB.size() ; i++ )
    {
      if ( ( sceneBB[i].x < 0 ) 
        || ( sceneBB[i].x > width )
        || ( sceneBB[i].y < 0) 
        || ( sceneBB[i].y > height ) )
          return false ;
    }
    
    // If all these conditions are met return the coordinates of the 
    // center of the detected pattern . 
    *x = sceneBB[ sceneBB.size() - 1 ].x ;
    *y = sceneBB[ sceneBB.size() - 1 ].y ;

    return true ;
    
    //~ trackingFrame.release();
  }
  else
    return false;
  
}

// Get the best matches between the scene and pattern descriptors.
int HazmatDetector::getBestMatches( const cv::Mat &frame ,
     const cv::Mat &features , double *minDist , double *maxDist )
{
  // Initialize total min and max distances.
  *minDist = std::numeric_limits<double>::max();
  //~ *maxDist = std::numeric_limits<double>::max();
  
  int bestMatch ;
  
  // Max and Min dist for every pattern.
  double tempMaxDist;
  double tempMinDist;
  
  
  
  cv::Mat *descriptorsObj;
  
  for (int i = 0 ; i < patterns_.size() ; i++ )
  {
    descriptorsObj = &patterns_[i].descriptors;
    // Find the matches.
    this->matcher_.match( *descriptorsObj , 
      features , matches );
    
    
    // Calculate the min and max distance between the descriptors
    // of the frame and the candidate pattern.
    tempMinDist = std::numeric_limits<double>::max();
    //~ tempMaxDist = std::numeric_limits<double>::min();
    
    // Calculation of max and min distances between keypoints for 
    // the patter #i .
    for( int j = 0; j < (*descriptorsObj).rows; j++ )
    { 
      double dist = matches[j].distance;
      if( dist < tempMinDist ) tempMinDist = dist;
      //~ if( dist > tempMaxDist ) tempMaxDist = dist;
    } 

    if (tempMinDist < *minDist )
    {
      //~ *maxDist = tempMaxDist ;
      *minDist = tempMinDist ;
      bestMatch = i ;
      bestMatches_ = matches ;
    }
    
  }
  
  return bestMatch;
  
}



