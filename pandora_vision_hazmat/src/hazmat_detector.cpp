#include "pandora_vision_hazmat/hazmat_detector.h"

int HazmatDetector::width = 640;
int HazmatDetector::height = 480;


// Allocating space for the static member variable.
std::string HazmatDetector::fileName_ = std::string("/home/vchoutas/Desktop/hazmatTraining/input") ;


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
  std::string trainingDataDir = "/home/vchoutas/Desktop/hazmatTraining/trainingData/" + this->getFeaturesName() ;

  std::string fileName ;
  
  for (int i = 0 ; i < input.size() ; i++) 
  { 
    // Open the training file associated with image #i .
    fileName = trainingDataDir + "/" + input[i] + ".xml" ;
    cv::FileStorage fs2( fileName ,cv::FileStorage::READ);
    
    // Check if the file was properly opened.
    if ( !fs2.isOpened() )
    {
      std::cerr << "File " << fileName << " could not be opened! " 
        << std::endl;
      continue;
    }
    
    std::vector<cv::Point2f> keyPoints;
    std::vector<cv::Point2f> boundingBox;
    cv::Mat descriptors; 
    cv::Mat histogram;
    
    // Read the pattern's descriptors.
    fs2["Descriptors"] >> descriptors;
    
    fs2["Histogram"] >> histogram;
        
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
    p.histogram = histogram ;
    patterns_->push_back(p);
    
    // Clear the data vectors.
    keyPoints.clear();
    boundingBox.clear(); 
    
    // Close the xml file .
    fs2.release() ;
    
  }
  
  std::cout << patterns_->size() << std::endl; 
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
  if ( patterns_ == NULL )
  {
    std::cout << "Error NULL pattern Pointer " << std::endl;
    exit(-1);
  }
  // Check if that patterns have been read succesfully.
  // TO DO : Produce Fatal Error on Failure.
  if ( patterns_->size() < 1 )
  {
    std::cerr << "No patterns read . Detection cannot continue " << 
      std::endl;
    *x = -1 ;
    *y = -1 ;
    return false;
  }
  
  
  // Set the pattern center to NULL .
  *x = -1 ;
  *y = -1 ;
  
  // The matrix that contains the descriptors of the frame.
  cv::Mat frameDescriptors ;
  // The mask that will be applied to the frame so as to limit the
  // the search space.
  cv::Mat mask ;
  // The detected keypoints of the frame that will be matched to the 
  // input pattern so as to find the correspondence from the training
  // set to the query frame.
  std::vector<cv::KeyPoint> frameKeyPoints;
  
  #ifdef CHRONO
  gettimeofday(&startwtime,NULL);
  #endif

  // Create the mask that will be used to extract regions of interest
  // on the image based on the Image Signature Saliency Map .
  ImageSignature::createSaliencyMapMask( frame , &mask );
  
  #ifdef CHRONO
  gettimeofday(&endwtime,NULL);
  double maskTime = (double)((endwtime.tv_usec - startwtime.tv_usec)
      /1.0e6 + endwtime.tv_sec - startwtime.tv_sec);
  std::cout << "Calculation time for creating the mask for frame  is " 
    << maskTime << std::endl;
  #endif
  
  #ifdef DEBUG
  cv::Mat maskedFrame;
  frame.copyTo(maskedFrame , mask);
  cv::imshow("Segmented Frame",maskedFrame);
  #endif

  // Calculate the keypoints and extract the descriptors from the 
  // frame.
  
  #ifdef CHRONO
  gettimeofday(&startwtime,NULL);
  #endif
  
  getFeatures( frame , mask , &frameDescriptors , &frameKeyPoints ) ; 
  
  #ifdef CHRONO
  gettimeofday(&endwtime,NULL);
  double featuresTime = (double)((endwtime.tv_usec - startwtime.tv_usec)
      /1.0e6 + endwtime.tv_sec - startwtime.tv_sec);
  std::cout << "Calculation time for the features for frame  is " 
    << featuresTime << std::endl;
  #endif
  
  bool matchesFound;
  bool boundingBoxFound;
  
  std::vector<cv::Point2f> patternKeyPoints;
  std::vector<cv::Point2f> sceneKeyPoints;
  std::vector<cv::Point2f> sceneBB ;
  
  int found ;
  
  // For every pattern in the list : 
  for (int i = 0 ; i < patterns_->size() ; i++ )
  {
    #ifdef CHRONO
    gettimeofday (&startwtime, NULL); 
    #endif
    
    // Test it before adding it to the system.
    //~ HistogramMask::createBackProjectionMask(frame , &mask , 
      //~ patterns_[i].histogram );
     
    
      
    // Try to find key point matches between the i-th pattern
    // and the descriptors and the keypoints extracted from the frame.
    matchesFound = findKeypointMatches(frameDescriptors , 
      (*patterns_)[i].descriptors , (*patterns_)[i].keyPoints , 
      frameKeyPoints , 
      &patternKeyPoints , 
      &sceneKeyPoints , i );
      
    #ifdef CHRONO
    gettimeofday(&endwtime,NULL);
    double keyPointTime = (double)((endwtime.tv_usec - startwtime.tv_usec)
				/1.0e6 + endwtime.tv_sec - startwtime.tv_sec);
    std::cout << "Calculation time for keypoint matches for pattern "
      << i << " is " << keyPointTime << std::endl;
    #endif
    
    // If we have succesfully found the matches.
    if (matchesFound)
    {
     
      #ifdef CHRONO
      gettimeofday(&startwtime,NULL);
      #endif
      
      // Find the bounding box for this query pattern .
      boundingBoxFound = findBoundingBox( patternKeyPoints , 
        sceneKeyPoints ,  (*patterns_)[i].boundingBox , &sceneBB );
        
      #ifdef CHRONO
      gettimeofday(&endwtime,NULL);
      double boundingBoxTime = (double)((endwtime.tv_usec - startwtime.tv_usec)
          /1.0e6 + endwtime.tv_sec - startwtime.tv_sec);
      std::cout << "Calculation time for the bounding box for "
        "the pattern " << i  << " is " << boundingBoxTime << std::endl;
      #endif      
      
      
      //~ std::cout << "Bounding Box flag : " << boundingBoxFound << std::endl;
        
      // If this flag is true then a valid match has been found and
      // we have detected the pattern.
      if (boundingBoxFound)
        
      {
        #ifdef DEBUG
        cv::Mat trackingFrame ;
        frame.copyTo(trackingFrame,mask);
        cv::line( trackingFrame, sceneBB[0] , sceneBB[1] , cv::Scalar(0, 255, 0), 4 );
        cv::line( trackingFrame, sceneBB[1] , sceneBB[2] , cv::Scalar( 0, 255, 0), 4 );
        cv::line( trackingFrame, sceneBB[3] , sceneBB[0] , cv::Scalar( 0, 255, 0), 4 );
        cv::line( trackingFrame, sceneBB[2] , sceneBB[3] , cv::Scalar( 0, 255, 0), 4 );
        cv::circle( trackingFrame , cv::Point2f(sceneBB[4].x,sceneBB[4].y)  , 4.0 , cv::Scalar(0,0,255) , -1 , 8 );
        imshow("Tracking Frame",trackingFrame);
       
        #endif
        
        
        //~ std::cout <<"Pattern " <<   i << std::endl;
        cv::Point2f base = sceneBB[0] - sceneBB[1];
        cv::Point2f h = sceneBB[1] - sceneBB[2];
        double sideA = cv::norm(base) ;
        double sideB = cv::norm(h) ;
        double surface = sideA * sideB ;
        
        
        
        //~ std::cout << "Surface " << surface << std::endl;
        //~ std::cout << "SideA " << sideA << std::endl;
        //~ std::cout << "SideB " << sideB << std::endl;
        

        
        //~ if ( sideA / sideB < 0.75 || sideA/sideB > 1.25)
          //~ continue;
        if (surface < 10 || surface > frame.rows*frame.cols)
        { 
          patternKeyPoints.clear();
          sceneBB.clear();
          sceneKeyPoints.clear();

          continue;
        }
        found = i;
        
      }
      
    }
    patternKeyPoints.clear();
    sceneBB.clear();
    sceneKeyPoints.clear();
    
  }
  
  if ( !boundingBoxFound )
  {
    *x = -1 ;
    *y = -1 ;
    patternKeyPoints.clear();
    sceneBB.clear();
    sceneKeyPoints.clear();
    return false;
  }
  
  // If all these conditions are met return the coordinates of the 
    // center of the detected pattern . 
  if ( sceneBB.empty() )
  {
    *x = -1 ;
    *y = -1 ;
    patternKeyPoints.clear();
    sceneBB.clear();
    sceneKeyPoints.clear();
  
    return false;
  }
  *x = sceneBB[ sceneBB.size() - 1 ].x ;
  *y = sceneBB[ sceneBB.size() - 1 ].y ;
  
  patternKeyPoints.clear();
  sceneBB.clear();
  sceneKeyPoints.clear();
  
  return true;

    
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
  
bool HazmatDetector::findBoundingBox( 
      const std::vector<cv::Point2f> &patternKeyPoints , 
      const std::vector<cv::Point2f> &sceneKeyPoints , 
      const std::vector<cv::Point2f> &patternBB , 
      std::vector<cv::Point2f> *sceneBB) 
{
  // Check if we have enough points to find the homography between
  // the pattern and the scene.
  if ( patternKeyPoints.size() > 4 &&  sceneKeyPoints.size() > 4 )
  {
//    std::cout << patternKeyPoints.size() << std::endl;

    // Calculate the homography matrix using RANSAC algorithm to 
    // eliminate outliers.
    cv::Mat H = cv::findHomography( patternKeyPoints , sceneKeyPoints, 
      CV_RANSAC , 4 );
/*
    cv::Mat invH = cv::findHomography( sceneKeyPoints , patternKeyPoints ,
        CV_RANSAC ) ; 

    cv::Mat res = H * invH ;
    for (int i = 0 ; i < 3 ; i++)
    {
      for (int j = 0 ; j < 3 ; j++)
      {
        std::cout << res.at<double>(i,j) << " ";
      }
      std::cout << std::endl;
    }
    std::cout << std::endl;
*/
    //cv::Mat H = cv::findHomography( patternKeyPoints , sceneKeyPoints, 
    //    CV_LMEDS ); 
    // Transform the bounding box to the frame coordinates.
    cv::perspectiveTransform( patternBB , 
      *sceneBB , H );
      
    
    
    // Check if every point of the bounding box is inside the image.
    // If not then the correspondences are invalid and these keypoints
    // are rejected.
    for (int i = 0 ; i < sceneBB->size() ; i++ )
    {
      if ( ( (*sceneBB)[i].x < 0 ) 
        || ( (*sceneBB)[i].x > HazmatDetector::width )
        || ( (*sceneBB)[i].y < 0) 
        || ( (*sceneBB)[i].y > HazmatDetector::height ) )
          {
            //~ std::cerr << "Bounding Box out of bounds " << std::endl;
            sceneBB->clear();
            return false ;
          }
    }
    
    // Check if the Bounding box is Convex 
    // If not the resulting homography is incorrect due to false
    // matching between the descriptors.
    std::vector<cv::Point2f> boundingBox = *sceneBB ;
    boundingBox.pop_back();
    if ( !cv::isContourConvex(boundingBox) )
    {
      boundingBox.clear();
      //~ std::cerr << "Contour not convex! " << std::endl;
      return false;
    }
    
    // Clear the bounding box vector .
    boundingBox.clear();
    return true ;
    
    //~ trackingFrame.release();
  }
  else
    return false;
  
}

