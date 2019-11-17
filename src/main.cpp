#ifdef __x86_64__

#include <stdio.h>

int main(int argc, char** argv) {
  (void)fprintf(stderr, "The raspicam_node for the x86/64 architecture is a fake!\n");
  return 1;
}

#endif  // __x86_64__

#ifdef __arm__
#include "interfaces/raspicam_mono_ros.hpp"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "raspicam_node");
    ros::NodeHandle nh_params("~");
    bool private_topics;
    nh_params.param<bool>("private_topics", private_topics, true);

  // The node handle used for topics will be private or public depending on the value of the ~private_topics parameter
  ros::NodeHandle nh_topics(private_topics ? std::string("~") : std::string());
  RASPICAM_INPUT_DATA input_data;
  RaspicamMonoRos mono_camera(input_data);
  mono_camera.init(nh_topics);
  mono_camera.start_capture();
  ros::spin();
  mono_camera.close_cam();
  ros::shutdown();
}
#endif  // __arm__