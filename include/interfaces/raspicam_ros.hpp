#pragma once
#include "ros/ros.h"
#include "camera_info_manager/camera_info_manager.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/SetCameraInfo.h"
#include "std_srvs/Empty.h"
#include "raspicam_node/MotionVectors.h"
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <dynamic_reconfigure/server.h>
#include <raspicam_node/CameraConfig.h>

#include "interfaces/raspicam_interface.hpp"
using diagnostic_updater::DiagnosedPublisher;
template<typename T>
struct DiagnosedMsgPublisher {
  std::unique_ptr<DiagnosedPublisher<T>> pub;
  T msg;
};

class RaspicamRos: public RaspicamInterface
{
private:
    /* data */
public:
    ~RaspicamRos();
    bool init(ros::NodeHandle& nh);
    virtual void setup_publishers(ros::NodeHandle& nh, double min_freq, double max_freq) {};
protected:
    RaspicamRos(RASPICAM_INPUT_DATA& input_parameters);
    void reconfigure_callback(raspicam_node::CameraConfig& config, uint32_t level);
    void configure_parameters(ros::NodeHandle& nh);
    virtual void get_additional_parameters(ros::NodeHandle& nh) = 0;
    // A method for specific managing with compressed image data
    virtual void image_compressed_out(PORT_USERDATA *pData, MMAL_BUFFER_HEADER_T* buffer) override {}
    // A method for specific managing with image raw data
    virtual void image_raw_out(PORT_USERDATA *pData, MMAL_BUFFER_HEADER_T* buffer) override {}
    // A method for specific managing with video data
    virtual void video_encoded_out(PORT_USERDATA *pData, MMAL_BUFFER_HEADER_T* buffer) override {}
    diagnostic_updater::Updater updater;
};
