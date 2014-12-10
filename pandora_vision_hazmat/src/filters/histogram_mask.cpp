#include "pandora_vision_hazmat/histogram_mask.h"


// YUV histogram back projection filter constructor
HistogramMask::HistogramMask(HazmatDetector *baseDetector) :
  detector_(baseDetector)
{
  histogramEnabled_ = true ;
  }
  
cv::Mat HistogramMask::normImage = cv::Mat();

bool HistogramMask::histogramEnabled_ = false;

/**
 @brief Creates a mask based on the histogram ratio of the pattern 
        and the input frame.
 @param frame [const cv::Mat &] : The input frame .
 @param mask [cv::Mat *] : The resulting mask.
 @param keyPoints [const Mat &] : .
**/   
void HistogramMask::createMask(const cv::Mat &frame , cv::Mat *mask , 
      const cv::Mat &data  )
{
  
  if ( !frame.data )
  {
    std::cerr << "Invalid frame data " << std::endl;
    // If an error has occured create an empty mask and thus the entire
    // frame will be scanned.
    *mask = cv::Mat( frame.size() , frame.type() );
    return ;
  }
  
  cv::Mat frameHist ;
  
  calcNormYUVHist( frame , &frameHist );
  
  if ( !frameHist.data )
  {
    std::cerr << "Invalid frame histogram data " << std::endl;
    // If an error has occured create an empty mask and thus the entire
    // frame will be scanned.
    *mask = cv::Mat( frame.size() , frame.type() );
    return ;
  }
  
  if ( !data.data )
  {
    std::cerr << "Invalid extra data " << std::endl;
    // Allocate the mask and make it compatible with the input frame.
    *mask = cv::Mat( frame.size() , CV_8UC1 );
    mask->setTo(255);
    return;
  }
  
  cv::Mat ratioHist;
  
  // Create the ratio histogram . 

  cv::divide( data , frameHist , ratioHist );
  
  
  if ( !ratioHist.data )
  {
    std::cerr << "Invalid Ratio Histogram data " << std::endl;
    mask->data = NULL;
    return ;
  }
  
  static double epsilon = 0.05 ;
  
  cv::Scalar mean =  cv::mean( data )[0]  ;
  double t =   mean[0] ;
  
  cv::Mat yuvFrame;
  
  
  cv::cvtColor(frame,yuvFrame,CV_BGR2YCrCb);
  
  double thresh = cv::threshold( ratioHist , ratioHist , t , 255 
    , cv::THRESH_TOZERO );
  //~ double thresh = cv::threshold(  , ratioHist , t , 255 
    //~ , cv::THRESH_TOZERO );
  //~ cv::normalize( ratioHist, ratioHist, 0, 255, CV_MINMAX);  

  normalizeImage( yuvFrame , 0 );

  int histDims = 2 ;
  
  int UBins = 256 ;
  int VBins = 256 ;
  int histSize[]  = { UBins , VBins };
  int histChannels[] = { 1 , 2 } ;
  
  // U / Cb : blue difference chroma component .

  float blueDiffRange[] = { 0 , 256 } ;
  
  // V / Cr : red difference chroma component .
  
  float redDiffRange[] = { 0 , 256 } ; 
  
  const float *ranges[] = { blueDiffRange , redDiffRange } ;
  
  
  cv::Mat backProj ;
  
  // Calculate the backprojection of the ratio histogram on the
  // normalized image.
  
  

  //~ cv::calcBackProject( &normImage , 1 , histChannels ,  ratioHist , 
    //~ backProj , ranges );
  cv::calcBackProject( &normImage , 1 , histChannels ,  frameHist , 
    backProj , ranges );
  
  
  
  std::vector<cv::Mat> Ychannels ;
  std::vector<cv::Mat> channels ;
  
  // Split the image to its channels.
  cv::split( yuvFrame , Ychannels );
  //~ cv::split( normImage , channels );
    
  
  //~ cv::multiply( backProj , Ychannels[0] , backProj );
  
  
  //~ backProj.convertTo(backProj,CV_8UC3,255);
  cv::imshow("NormImage",normImage);
  cv::imshow("BackProj",backProj);
  if ( !mask->data )
  {
    *mask = cv::Mat( backProj.size() , backProj.type() );
    mask->setTo(255);
  }
  
  cv::bitwise_or( *mask , backProj , *mask );
  detector_->createMask( frame , mask , data );
  
  //~ backProj.convertTo(backProj,CV_8UC1,255);
  //~ cv::imshow("BackProj",backProj);
  
  Ychannels.clear();
  channels.clear();
  
  return ;
}


// Calculate the normalized YUV histogram.
void HistogramMask::calcNormYUVHist(const cv::Mat &image , 
  cv::Mat *hist)
{
  // YUV destination image.
  cv::Mat yuv;
  
  // Convert the input image to YUV format.
  cv::cvtColor( image , yuv , CV_BGR2YCrCb );
  
  // Normalize the image according to its Y component.
  normalizeImage(yuv , 0 );
  
  int histDims = 2 ;
  
  int UBins = 256 ;
  int VBins = 256 ;
  int histSize[]  = { UBins , VBins };
  const int histChannels[] = { 1 , 2 } ;
  
  // U / Cb : blue difference chroma component .

  
  float blueDiffRange[] = { 0 , 256 } ;
  
  // V / Cr : red difference chroma component .
  
  float redDiffRange[] = { 0 , 256 } ; 
  
  const float *ranges[] = { blueDiffRange , redDiffRange } ;
  

  
  // Calculate the histogram of the normalized YUV image from the 
  // u & v channels.
  cv::calcHist( &normImage , 1 , histChannels , cv::Mat() , *hist , 
    histDims , histSize , ranges ) ;
  
  cv::normalize( *hist , *hist , 1 , 0, cv::NORM_L1);
  
  }


// Normalizes an image by its #ch channel.

void HistogramMask::normalizeImage(cv::Mat &image , int ch)
{
  // Check if the image is valid.
  if ( !image.data )
  {
      std::cerr << " The image array is empty! " << std::endl;
      normImage.data =  NULL;
      return ;
  }
  
  // Check if the image has enough channels.
  if ( image.channels() <= 1 )
  {
    std::cerr << "The input image has only one channel! " << std::endl;
    normImage.data =  NULL;
    return;
  }
  
  
  float maxVal ;
  
  // Check the image type and find its maximum possible value.
  switch ( image.type() )
  {
    case CV_8UC3 : 
      maxVal = 255.f ;
      break;
    case CV_16UC3 : 

      maxVal = 65535.f ;
      break;
    case CV_32FC3 : 

      maxVal = 1.f ;
      break;
  }
  
  cv::Mat floatImg ;
  // Convert the input image to a 3-channel float image to prevent
  // information loss due to saturation casts.
  // May change to be made more generic for arbitraty number of 
  // channels.
  image.convertTo( floatImg , CV_32FC3 , 1.f/maxVal );
  
  std::vector<cv::Mat> channels ;
  std::vector<cv::Mat> normChannels( image.channels() ) ;
  
  // Split the image to its channels.
  cv::split( floatImg , channels );
  
  
  for (int i = 0 ; i < floatImg.channels() ; i++ )
  {
    cv::divide( channels[i] , channels[ch] , normChannels[i] );
  }

  // Merge the normalized image channels into a single image.
  cv::merge(normChannels , normImage );
  
  // Return an image that is of the same type as the input image.
  normImage.convertTo( normImage , image.type() , maxVal );
  
  
  
  channels.clear();
  normChannels.clear();
  return;
  }
