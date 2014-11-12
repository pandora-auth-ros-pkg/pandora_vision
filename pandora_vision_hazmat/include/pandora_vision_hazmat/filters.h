#ifndef FILTERS_H
#define FILTERS_H

#include "pandora_vision_hazmat/detector_interface.h"

/** 
 @class HistogramMask
 @brief Class the implements the histogram back projection algorithm 
 **/
 

class HistogramMask : public Detector
{
  public : 
    
    // Function that calculates the normalized histogram
    static void calcNormYUVHist(const cv::Mat &image , cv::Mat *hist);
    
    // Function the calculates the backprojection 
    void virtual createMask( const cv::Mat &frame , cv::Mat *mask , 
      cv::Mat &data  );
    
    // Normalize an image by dividing it with the channel ch.
    static void normalizeImage(cv::Mat &image , cv::Mat *normImage , 
      int ch );
    
    // The class who is being decorated is called to detect the 
    // pattern we wish to find.
    
    bool virtual detect(const cv::Mat &frame , 
      float *x , float *y )
      
    {
      return detector_->detect(frame,x,y);
    }
    
    // Function used for reading the training data from an xml file.
    // The necessary data has to be read by the object point by the
    // detector_ pointer.
    void virtual readData( void ) 
    {
      return ;
      }
    
    // Function used to get the best feature matches between a frame
    // and a number of patterns.
    // In this decorator class it has no need to be implemented
    // since it's purpose is to create the correct filter.
    int virtual getBestMatches( const cv::Mat &frame ,
     const cv::Mat &features , double *minDist , double *maxDist  ) 
     {
       return 0;
     }
     
    // Constructor
    HistogramMask(Detector *baseDetector);
    
    virtual ~HistogramMask() 
    {
    } 
    
  private :
    // Pointer the base class that will be decorated.
    Detector *detector_ ;
  };
  
/**
 @class ImageSignature 
 @brief Class that implements the image signature saliency map.
**/

class ImageSignature : public Detector
{
  public : 
    
    // Function that calculates the image signature.
    static void calculateSignature(const cv::Mat &image , 
      cv::Mat *imgSign);
    
    // Function the creates the mask that will be applied to the 
    // incoming frame based on the saliency map produced by the 
    // signature of the image.
    void virtual createMask( const cv::Mat &frame , cv::Mat *mask , 
      cv::Mat &data  );
    
    // Return the array that containts the signs of an arbitrary
    // matrix.
    static void signFunction(cv::Mat &array , cv::Mat *signs , 
      int ch );
    
    // The class who is being decorated is called to detect the 
    // pattern we wish to find.
    
    bool virtual detect(const cv::Mat &frame , 
      float *x , float *y )
      
    {
      return detector_->detect(frame,x,y);
    }
    
    // Function used for reading the training data from an xml file.
    // The necessary data has to be read by the object point by the
    // detector_ pointer.
    void virtual readData( void ) 
    {
      return ;
      }
    
    // Function used to get the best feature matches between a frame
    // and a number of patterns.
    // In this decorator class it has no need to be implemented
    // since it's purpose is to create the correct filter.
    int virtual getBestMatches( const cv::Mat &frame ,
     const cv::Mat &features , double *minDist , double *maxDist  ) 
     {
       return 0;
     }
     
    // Constructor
    ImageSignature(Detector *baseDetector);
    
    virtual ~ImageSignature() 
    {
    } 
    
  private :
    // Pointer the base class that will be decorated.
    Detector *detector_ ;
  };

#endif
