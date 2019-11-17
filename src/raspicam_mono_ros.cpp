#include "interfaces/raspicam_mono_ros.hpp"

RaspicamMonoRos::RaspicamMonoRos(RASPICAM_INPUT_DATA& input_parameters):RaspicamRos(input_parameters)
{

}

RaspicamMonoRos::~RaspicamMonoRos()
{

}

void RaspicamMonoRos::get_additional_parameters(ros::NodeHandle& nh)
{

  nh.param("camera_info_url", camera_info_url, std::string("package://raspicam_node/camera_info/camera.yaml"));
  nh.param("camera_name", camera_name, std::string("camera"));

  camera_info_manager::CameraInfoManager c_info_man(nh, camera_name, camera_info_url);


  if (!c_info_man.loadCameraInfo(camera_info_url)) {
    ROS_INFO("Calibration file missing. Camera not calibrated");
  } else {
    c_info = c_info_man.getCameraInfo();
    ROS_INFO("Camera successfully calibrated from device specifc file");
  }

  // if (!c_info_man.loadCameraInfo("")) {
  //   ROS_INFO("No device specifc calibration found");
  // } else {
  //   c_info = c_info_man.getCameraInfo();
  //   ROS_INFO("Camera successfully calibrated from device specifc file");
  // }
}

void RaspicamMonoRos::image_compressed_out(PORT_USERDATA *pData, MMAL_BUFFER_HEADER_T* buffer) 
{
  compressed_image.msg.header.seq = pData->frame;
  compressed_image.msg.header.frame_id = camera_frame_id;
  compressed_image.msg.header.stamp = ros::Time::now();
  compressed_image.msg.format = "jpeg";
  auto start = pData->buffer[pData->frame & 1].get();
  auto end = &(pData->buffer[pData->frame & 1].get()[pData->id]);
  compressed_image.msg.data.resize(pData->id);
  std::copy(start, end, compressed_image.msg.data.begin());
  compressed_image.pub->publish(compressed_image.msg);

  c_info.header.seq = pData->frame;
  c_info.header.stamp = compressed_image.msg.header.stamp;
  c_info.header.frame_id = compressed_image.msg.header.frame_id;
  camera_info_pub.publish(c_info);
}

void RaspicamMonoRos::image_raw_out(PORT_USERDATA *pData, MMAL_BUFFER_HEADER_T* buffer) 
{
  image.msg.header.seq = pData->frame;
  image.msg.header.frame_id = camera_frame_id;
  image.msg.header.stamp = c_info.header.stamp;
  image.msg.encoding = "bgr8";
  image.msg.is_bigendian = false;
  image.msg.height = pData->pstate.input_parameters.height;
  image.msg.width = pData->pstate.input_parameters.width;
  image.msg.step = (pData->pstate.input_parameters.width * 3);
  auto start = pData->buffer[pData->frame & 1].get();
  auto end = &(pData->buffer[pData->frame & 1].get()[pData->id]);
  image.msg.data.resize(pData->id);
  std::copy(start, end, image.msg.data.begin());
  image.pub->publish(image.msg);
}

void RaspicamMonoRos::video_encoded_out(PORT_USERDATA *pData, MMAL_BUFFER_HEADER_T* buffer)
{
   // Frame information
    motion_vectors.msg.header.seq = pData->frame;
    motion_vectors.msg.header.frame_id = camera_frame_id;
    motion_vectors.msg.header.stamp = ros::Time::now();

    // Number of 16*16px macroblocks
    motion_vectors.msg.mbx = pData->pstate.input_parameters.width / 16;
    if (pData->pstate.input_parameters.width % 16)
      motion_vectors.msg.mbx++;

    motion_vectors.msg.mby = pData->pstate.input_parameters.height / 16;
    if (pData->pstate.input_parameters.height % 16)
      motion_vectors.msg.mby++;
    // Motion vector data
    struct __attribute__((__packed__)) imv {
      int8_t x;
      int8_t y;
      uint16_t sad;
    }* imv = reinterpret_cast<struct imv*>(buffer->data);

    size_t num_elements = buffer->length / sizeof(struct imv);
    motion_vectors.msg.x.resize(num_elements);
    motion_vectors.msg.y.resize(num_elements);
    motion_vectors.msg.sad.resize(num_elements);

    for (size_t i = 0; i < num_elements; i++) {
      motion_vectors.msg.x[i] = imv->x;
      motion_vectors.msg.y[i] = imv->y;
      motion_vectors.msg.sad[i] = imv->sad;
      imv++;
    }
    motion_vectors.pub->publish(motion_vectors.msg);
}
void RaspicamMonoRos::setup_publishers(ros::NodeHandle& nh, double min_freq, double max_freq)
{
   if (state.input_parameters.enable_raw_pub){
    auto image_pub = nh.advertise<sensor_msgs::Image>("image", 1);
    image.pub.reset(new DiagnosedPublisher<sensor_msgs::Image>(
        image_pub, updater, FrequencyStatusParam(&min_freq, &max_freq, 0.1, 10), TimeStampStatusParam(0, 0.2)));
  }
  if (state.input_parameters.enable_imv_pub) {
    auto imv_pub = nh.advertise<raspicam_node::MotionVectors>("motion_vectors", 1);
    motion_vectors.pub.reset(new DiagnosedPublisher<raspicam_node::MotionVectors>(
        imv_pub, updater, FrequencyStatusParam(&min_freq, &max_freq, 0.1, 10), TimeStampStatusParam(0, 0.2)));
  }
  auto cimage_pub = nh.advertise<sensor_msgs::CompressedImage>("image/compressed", 1);
  compressed_image.pub.reset(new DiagnosedPublisher<sensor_msgs::CompressedImage>(
      cimage_pub, updater, FrequencyStatusParam(&min_freq, &max_freq, 0.1, 10), TimeStampStatusParam(0, 0.2)));
  
  camera_info_pub = nh.advertise<sensor_msgs::CameraInfo>("camera_info", 1);
}

