#include "pandora_vision_hazmat/filters.h"

HistogramMask::HistogramMask

// Calculate the normalized YUV histogram.
void HistogramMask::calcNormYUVHist(const cv::Mat &image , 
  cv::Mat *hist)
{
  // YUV destination image.
  cv::Mat yuv;
  
  // Normalized YUV image.
  cv::Mat normYUV;
  
  // Convert the input image to YUV format.
  cv::cvtColor( image , yuv , CV_BGR2YCrCb );
  
  
  // Normalize the image according to its Y component.
  normalizeImage(yuv , &normYUV , 1 );
    
  
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
  

  
  // Calculate the histogram of the normalized YUV image from the 
  // u & v channels.
  cv::calcHist( &normYUV , 1 , histChannels , cv::Mat() , *hist , 
    histDims , histSize , ranges ) ;
  
  
  
  }


// Normalizes an image by its #ch channel.

void HistogramMask::normalizeImage(cv::Mat &image , cv::Mat *normImage , 
      int ch)
{
  // Check if the image is valid.
  if ( !image.data )
  {
      std::cerr << " The image array is empty! " << std::endl;
      normImage->data =  NULL;
      return ;
  }
  
  // Check if the image has enough channels.
  if ( image.channels() <= 1 )
  {
    std::cerr << "The input image has only one channel! " << std::endl;
    normImage->data =  NULL;
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
  
  // Initialize the image to be returned.
  *normImage  = cv::Mat( floatImg.size() , floatImg.type() );
  
  
  std::vector<cv::Mat> channels ;
  std::vector<cv::Mat> normChannels( normImage->channels() ) ;
  
  // Split the image to its channels.
  cv::split( floatImg , channels );
  
  // Perform the normalization.
  for (int i = 0 ; i < floatImg.channels() ; i++ )
  {
    cv::divide( channels[i] , channels[ch] , normChannels[i] );
  }
  
  // Merge the normalized image channels into a single image.
  cv::merge(normChannels , *normImage );
  
  // Return an image that is of the same type as the input image.
  normImage->convertTo( *normImage , image.type() , maxVal );
  
  }
