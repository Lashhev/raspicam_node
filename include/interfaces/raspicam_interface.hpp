#pragma once
// We use some GNU extensions (basename)
#include <memory.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <memory>

#define VCOS_ALWAYS_WANT_LOGGING
#define VERSION_STRING "v1.2"

#include "bcm_host.h"

#include "mmal_cxx_helper.h"


// extern "C" {
// #include "RaspiCamControl.h"
// }

static constexpr int IMG_BUFFER_SIZE = 10 * 1024 * 1024;  // 10 MB

// Video format information
static constexpr int VIDEO_FRAME_RATE_DEN = 1;

// Video render needs at least 2 buffers.
static constexpr int VIDEO_OUTPUT_BUFFERS_NUM = 3;

/** Structure containing all state information for the current run
 */
typedef struct RASPICAM_INPUT_DATA_T {
  bool isInit;
  int width;      /// Requested width of image
  int height;     /// requested height of image
  int framerate;  /// Requested frame rate (fps)
  int quality;
  bool enable_raw_pub;  // Enable Raw publishing
  bool enable_imv_pub;  // Enable publishing of inline motion vectors

  int camera_id = 0;

} RASPICAM_INPUT_DATA;

struct RASPIVID_STATE 
{
  RASPIVID_STATE():
    camera_component(nullptr), 
    splitter_component(nullptr), 
    image_encoder_component(nullptr),
    video_encoder_component(nullptr),
    splitter_connection(nullptr),
    image_encoder_connection(nullptr),
    video_encoder_connection(nullptr),
    splitter_pool(nullptr, mmal::default_delete_pool),
    image_encoder_pool(nullptr, mmal::default_delete_pool),
    video_encoder_pool(nullptr, mmal::default_delete_pool)
  {}
  RASPICAM_INPUT_DATA input_parameters;
  RASPICAM_CAMERA_PARAMETERS camera_parameters;  /// Camera setup parameters

  mmal::component_ptr camera_component;
  mmal::component_ptr splitter_component;
  mmal::component_ptr image_encoder_component;
  mmal::component_ptr video_encoder_component;

  mmal::connection_ptr splitter_connection;       /// Pointer to camera => splitter
  mmal::connection_ptr image_encoder_connection;  /// Pointer to splitter => encoder
  mmal::connection_ptr video_encoder_connection;  /// Pointer to camera => encoder

  mmal::pool_ptr splitter_pool;       // Pointer buffer pool used by splitter (raw) output
  mmal::pool_ptr image_encoder_pool;  // Pointer buffer pool used by encoder (jpg) output
  mmal::pool_ptr video_encoder_pool;  // Pointer buffer pool used by encoder (h264) output
//   diagnostic_updater::Updater updater; // TODO: put it in ros interface
};

typedef struct MMAL_PORT_USERDATA_T {
  MMAL_PORT_USERDATA_T(RASPIVID_STATE& state) : pstate(state){};
  std::unique_ptr<uint8_t[]> buffer[2];  // Memory to write buffer data to.
  RASPIVID_STATE& pstate;                // pointer to our state for use by callback
  bool abort;                            // Set to 1 in callback if an error occurs to attempt to abort
                                         // the capture
  size_t frame;
  size_t id;

  int frames_skipped = 0;
  ~MMAL_PORT_USERDATA_T()
  {
    // if(pstate!=nullptr)
    //   delete pstate;
  }
} PORT_USERDATA;

class RaspicamInterface
{
  MMAL_PORT_T* camera_video_port;
  MMAL_PORT_T* camera_preview_port;
  MMAL_PORT_T* splitter_input_port;
  MMAL_PORT_T* splitter_output_enc;
  MMAL_PORT_T* splitter_output_raw;
  MMAL_PORT_T* image_encoder_input_port;
  MMAL_PORT_T* image_encoder_output_port;
  MMAL_PORT_T* video_encoder_input_port;
  MMAL_PORT_T* video_encoder_output_port;
  PORT_USERDATA* callback_data_enc;
  PORT_USERDATA* h264_callback_data_enc;
  PORT_USERDATA* callback_data_raw;
public:
    int init_cam();
    int start_capture();
    int close_cam();
protected:
    RaspicamInterface(RASPICAM_INPUT_DATA& input_parameters);
    void image_encoder_buffer_callback(MMAL_PORT_T* port, MMAL_BUFFER_HEADER_T* buffer);
    void video_encoder_buffer_callback(MMAL_PORT_T* port, MMAL_BUFFER_HEADER_T* buffer);
    void splitter_buffer_callback(MMAL_PORT_T* port, MMAL_BUFFER_HEADER_T* buffer);
    // A method for specific managing with compressed image data
    virtual void image_compressed_out(PORT_USERDATA *pData, MMAL_BUFFER_HEADER_T* buffer) {};
    // A method for specific managing with image raw data
    virtual void image_raw_out(PORT_USERDATA *pData, MMAL_BUFFER_HEADER_T* buffer) {};
    // A method for specific managing with video data
    virtual void video_encoded_out(PORT_USERDATA *pData, MMAL_BUFFER_HEADER_T* buffer) {};
    MMAL_COMPONENT_T* create_camera_component();
    MMAL_STATUS_T create_image_encoder_component();
    MMAL_STATUS_T create_video_encoder_component();
    MMAL_STATUS_T create_splitter_component();
    MMAL_STATUS_T connect_ports(MMAL_PORT_T* output_port, MMAL_PORT_T* input_port,
                                   mmal::connection_ptr& connection);

    RASPIVID_STATE state;
    int skip_frames;
public:
    ~RaspicamInterface();
};

