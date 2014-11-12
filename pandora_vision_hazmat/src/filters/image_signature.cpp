#include "pandora_vision_hazmat/image_signature.h"


ImageSignature::ImageSignature(HazmatDetector *baseDetector) : 
  detector_(baseDetector)
{
  
  }

/**
 @brief Calculates the signs of an arbitrary 1-channel matrix.
 @param image [const cv::Mat &] : The input image
 @param imgSign [cv::Mat *] : The output matrix with the signs of the 
                              image.
**/

void ImageSignature::signFunction(const cv::Mat &array , cv::Mat *signs)
{
  if ( array.channels() > 1 )
  {
    std::cerr << "Invalid channel number " << std::endl;
    signs->data = NULL;
    return;
  }
  
  // If the image is an array of unsigned chars then the output
  // should be a signed array with the same size and number of channels.
  cv::Mat floatArray ;

  switch (array.type())
  {
    // If the input is unsigned then return an array of ones,
    // since we can only have positive elements.
    case CV_8UC1:
    case CV_16UC1:
      *signs = cv::Mat( array.size() , CV_8SC1 );
      signs->setTo(1);
      return ;
      break;
    default:
      array.convertTo(floatArray, CV_32FC1);
      *signs = cv::Mat( array.size() , CV_32FC1 );
  }
    
  cv::divide( array , abs(array) , *signs );
  
  
  }
  
void ImageSignature::calculateSignature(const cv::Mat &image , 
      cv::Mat *imgSign)
{
  
  
  cv::Mat imageDCT ;
  
  // Compute the discrete cosine transform of the input image.
  cv::dct(image , imageDCT   );
  
  // Calculate the signature of the image.
  ImageSignature::signFunction( imageDCT , imgSign );
  
  return ;
  
  }
  
void  ImageSignature::createMask( const cv::Mat &frame , cv::Mat *mask , 
     const cv::Mat &data  )
{
  if ( !frame.data )
  {
    std::cerr << "Invalid frame " << std::endl;
    mask->data = NULL ;
    return ;
  }
  
  static cv::Mat saliency;
  
  // Convert the frame to a 3-channel float image and scale
  // it's values accordingly.
  frame.convertTo( saliency , CV_32FC3 , 1/255.f );
    
  // Resize the frame so as to process it correctly.
  // TO DO : read the size from file.
  cv::resize( saliency , saliency , cv::Size(64,48) );
  
  static cv::Mat signature;
  static cv::Mat invDCT ; 
  
  if ( frame.channels() ==  1 )
  { 
    ImageSignature::calculateSignature( saliency , &signature );
    
    // Perform the inverse DCT on the signature.
    cv::dct( signature , invDCT , cv::DCT_INVERSE );
    
    *mask = invDCT.mul(invDCT);
    
    // Filter the result using a gaussian filter with a 5X5 kernel.
    
    cv::GaussianBlur(*mask , *mask , cv::Size(5,5) , 0.05 );
    //~ cv::medianBlur( *mask , *mask , 5  );
    
    return ;
  }
  
  static std::vector<cv::Mat> channels;

  // Split the input frame into it's separate channels.
  cv::split( saliency , channels );
  
  // The resulting mask is a 1-channel floating point image , so as to
  // correctly perform forward and inverse Discrete Cosine 
  // Transformatios.
  cv::Mat sum = cv::Mat::zeros( saliency.size() , CV_32FC1 );
  
  // Temporary container for the inverse Discrete Cosine Transform
  // for the #i channel of the image.
  static cv::Mat tempMap = cv::Mat::zeros( saliency.size() , CV_32FC1 );
  // For every channel of the image : 
  for (int i = 0 ; i < saliency.channels() ; i++  )
  {
    ImageSignature::calculateSignature( channels[i] , &signature );
    
    // Perform the inverse DCT on the signature.
    cv::dct( signature , invDCT , cv::DCT_INVERSE );
    
    tempMap = invDCT.mul(invDCT);
    cv::GaussianBlur( tempMap , tempMap , cv::Size(5,5) , 0 );
    sum  = sum + tempMap ;
  }
  // Calculate the mean of the saliency values of the 3 input channels.
  sum = (1/3.f) * sum ;
  
  
  // Resize the mask so that it can be applied to the frame.
  cv::resize( sum , sum  , frame.size() );
  
  
  // Threshold the mask to decrease noise and keep only the regions
  // of interest.
  cv::threshold( sum  , sum , 0.99 , 1 , cv::THRESH_BINARY); 
  
  // Convert the mask to 1-channel 8 bit format.
  sum.convertTo( sum , CV_8UC1 , 255 );
    cv::imshow("Sum",sum);

  
  cv::medianBlur( sum , sum  , 3 );
  
  
  
  // Use the logical or operatio between the calculated mask if we have
  // received a valid mask.
  if ( mask->data)
    cv::bitwise_or(sum , *mask , *mask );
  else
    *mask = sum ;
    
  // Call the rest of the decorators.
  detector_->createMask(frame , mask , data );
  
  
  
  return ;
  }
