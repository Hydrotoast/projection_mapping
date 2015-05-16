#ifndef FREENECT_WORKER_H
#define FREENECT_WORKER_H

#include "buffers.h"

#include "libfreenect.h"

class FreenectWorker {
public:
  FreenectWorker(size_t width, size_t height, 
      buffers::SharedBuffer<uint16_t> &pcl_back, 
      buffers::SharedBuffer<uint8_t> &rgb_back);
  ~FreenectWorker();

  // Too expensive to copy or move
  FreenectWorker(const FreenectWorker&) = delete;
  FreenectWorker& operator=(const FreenectWorker&) = delete;
  FreenectWorker(FreenectWorker&&);
  FreenectWorker& operator=(FreenectWorker&&) = delete;

  // Callbacks
  void depth_callback(freenect_device *dev, void *v_depth, uint32_t timestamp);
  void video_callback(freenect_device *dev, void *v_video, uint32_t timestamp);

  // Call operator
  void operator()();
private:
  int width_;
  int height_;

  uint16_t *depth_buffer_;
  uint8_t *video_buffer_;

  freenect_context *f_ctx_;
  freenect_device *f_dev_;

  freenect_video_format video_format_;

  buffers::SharedBuffer<uint16_t> &pcl_back_;
  buffers::SharedBuffer<uint8_t> &rgb_back_;

  static void freenect_depth_callback(freenect_device *dev, void *v_depth, uint32_t timestamp);
  static void freenect_video_callback(freenect_device *dev, void *v_video, uint32_t timestamp);
};

#endif
