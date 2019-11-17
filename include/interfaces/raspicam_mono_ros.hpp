#pragma once
#include "interfaces/raspicam_ros.hpp"

using diagnostic_updater::DiagnosedPublisher;
using diagnostic_updater::FrequencyStatusParam;
using diagnostic_updater::TimeStampStatusParam;

class RaspicamMonoRos: public RaspicamRos
{
    private:
        DiagnosedMsgPublisher<sensor_msgs::Image> image;
        DiagnosedMsgPublisher<sensor_msgs::CompressedImage> compressed_image;
        DiagnosedMsgPublisher<raspicam_node::MotionVectors> motion_vectors;
        ros::Publisher camera_info_pub;
        sensor_msgs::CameraInfo c_info;
        std::string camera_frame_id;
        std::string camera_info_url;
        std::string camera_name;
        void setup_publishers(ros::NodeHandle& nh, double min_freq, double max_freq) override;
        void get_additional_parameters(ros::NodeHandle& nh) override;
        // A method for specific managing with compressed image data
        void image_compressed_out(PORT_USERDATA *pData, MMAL_BUFFER_HEADER_T* buffer) override;
        // A method for specific managing with image raw data
        void image_raw_out(PORT_USERDATA *pData, MMAL_BUFFER_HEADER_T* buffer) override;
        // A method for specific managing with video data
        void video_encoded_out(PORT_USERDATA *pData, MMAL_BUFFER_HEADER_T* buffer) override;
    public:
        RaspicamMonoRos(RASPICAM_INPUT_DATA& input_parameters);
        ~RaspicamMonoRos();
};


