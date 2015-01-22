#include "pandora_vision_hazmat/hazmat_detection.h"

/**
  @brief Main function of the face node
  @param argc [int] Number of input arguments
  @param argv [char**] The input arguments
  @return int : 0 for success
 **/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "hazmat_node");
  HazmatDetectionNode hazmat_node;
  ros::spin();
  return 0;
}
