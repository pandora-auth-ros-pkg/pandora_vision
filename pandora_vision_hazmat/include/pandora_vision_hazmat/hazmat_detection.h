#include "pandora_vision_hazmat/hazmat_detector.h"
#include "pandora_vision_hazmat/sift_hazmat_detector.h"
#include "pandora_vision_hazmat/surf_hazmat_detector.h"
#include "pandora_vision_hazmat/orb_hazmat_detector.h"

class HazmatDetectionNode
{
  public:
    HazmatDetectionNode();

    ~HazmatDetectionNode()
    { };
  
    void imageCallback(const sensor_msgs::Image& inputImage);

    void setImageTopic(const std::string& imageTopic)
    {
      imageTopic_ = imageTopic;
    }
  private:
    ros::NodeHandle nodeHandle_;

    ros::Subscriber imageSubscriber_ ;
    
    ros::Publisher hazmatPublisher_; 

    std::string imageTopic_;

    std::string hazmatTopic_ ;
    SiftHazmatDetector detector_ ;
    // OrbHazmatDetector detector_;
};
