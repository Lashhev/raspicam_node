#include "interfaces/raspicam_ros.hpp"

RaspicamRos::RaspicamRos(RASPICAM_INPUT_DATA& input_parameters):RaspicamInterface(input_parameters)
{

}

RaspicamRos::~RaspicamRos()
{

}

void RaspicamRos::reconfigure_callback(raspicam_node::CameraConfig& config, uint32_t level)
{
  ROS_DEBUG("figure Request: contrast %d, sharpness %d, brightness %d, "
            "saturation %d, ISO %d, exposureCompensation %d,"
            " videoStabilisation %d, vFlip %d, hFlip %d,"
            " zoom %.2f, exposure_mode %s, awb_mode %s, shutter_speed %d",
            config.contrast, config.sharpness, config.brightness, config.saturation, config.ISO,
            config.exposure_compensation, config.video_stabilisation, config.vFlip, config.hFlip, config.zoom,
            config.exposure_mode.c_str(), config.awb_mode.c_str(), config.shutter_speed);

  if (!state.camera_component.get()) {
    ROS_WARN("reconfiguring, but camera_component not initialized");
    return;
  }

  if (config.zoom < 1.0) {
    ROS_ERROR("Zoom value %f too small (must be at least 1.0)", config.zoom);
  } else {
    const double size = 1.0 / config.zoom;
    const double offset = (1.0 - size) / 2.0;
    PARAM_FLOAT_RECT_T roi;
    roi.x = roi.y = offset;
    roi.w = roi.h = size;
    raspicamcontrol_set_ROI(state.camera_component.get(), roi);
  }

  raspicamcontrol_set_exposure_mode(state.camera_component.get(), exposure_mode_from_string(config.exposure_mode.c_str()));

  raspicamcontrol_set_awb_mode(state.camera_component.get(), awb_mode_from_string(config.awb_mode.c_str()));

  raspicamcontrol_set_contrast(state.camera_component.get(), config.contrast);
  raspicamcontrol_set_sharpness(state.camera_component.get(), config.sharpness);
  raspicamcontrol_set_brightness(state.camera_component.get(), config.brightness);
  raspicamcontrol_set_saturation(state.camera_component.get(), config.saturation);
  raspicamcontrol_set_ISO(state.camera_component.get(), config.ISO);
  raspicamcontrol_set_exposure_compensation(state.camera_component.get(), config.exposure_compensation);
  raspicamcontrol_set_video_stabilisation(state.camera_component.get(), config.video_stabilisation);
  raspicamcontrol_set_flips(state.camera_component.get(), config.hFlip, config.vFlip);
  raspicamcontrol_set_shutter_speed(state.camera_component.get(), config.shutter_speed);

  ROS_DEBUG("Reconfigure done");
}

void RaspicamRos::configure_parameters(ros::NodeHandle& nh) {

  nh.param<int>("skip_frames", skip_frames, 0);
  nh.param<int>("width", state.input_parameters.width, 1280);
  nh.param<int>("height", state.input_parameters.height, 720);
  nh.param<int>("quality", state.input_parameters.quality, 80);
  if (state.input_parameters.quality < 0 && state.input_parameters.quality > 100) {
    ROS_WARN("quality: %d is outside valid range 0-100, defaulting to 80", state.input_parameters.quality);
    state.input_parameters.quality = 80;
  }
  nh.param<int>("framerate", state.input_parameters.framerate, 30);
  if (state.input_parameters.framerate < 0 && state.input_parameters.framerate > 90) {
    ROS_WARN("framerate: %d is outside valid range 0-90, defaulting to 30", state.input_parameters.framerate);
    state.input_parameters.framerate = 30;
  }
  get_additional_parameters(nh);
  // nh.param<std::string>("camera_frame_id", camera_frame_id, "");

  nh.param<bool>("enable_raw", state.input_parameters.enable_raw_pub, false);
  nh.param<bool>("enable_imv", state.input_parameters.enable_imv_pub, false);
  nh.param<int>("camera_id", state.input_parameters.camera_id, 0);

  // Set up the camera_parameters to default
  raspicamcontrol_set_defaults(&state.camera_parameters);

  bool temp;
  nh.param<bool>("hFlip", temp, false);
  state.camera_parameters.hflip = temp;  // Hack for bool param => int variable
  nh.param<bool>("vFlip", temp, false);
  state.camera_parameters.vflip = temp;  // Hack for bool param => int variable
  nh.param<int>("shutter_speed", state.camera_parameters.shutter_speed, 0);

  state.input_parameters.isInit = false;
}
bool RaspicamRos::init(ros::NodeHandle& nh)
{
  get_additional_parameters(nh);
  configure_parameters(nh);
  init_cam();
  // diagnostics parameters
  updater.setHardwareID("raspicam");
  double desired_freq = state.input_parameters.framerate;
  double min_freq = desired_freq * 0.95;
  double max_freq = desired_freq * 1.05;
  setup_publishers(nh, min_freq, max_freq);
  dynamic_reconfigure::Server<raspicam_node::CameraConfig> server;
  dynamic_reconfigure::Server<raspicam_node::CameraConfig>::CallbackType f;
  f = boost::bind(&RaspicamRos::reconfigure_callback,this, _1, _2);
  server.setCallback(f);
  return true;
}

