#include "libfreenect.h"

#include "pcl/common/projection_matrix.h"
#include "pcl/ModelCoefficients.h"
#include "pcl/point_types.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"

#include <GL/glut.h>

#include <atomic>
#include <chrono>
#include <mutex>
#include <stdexcept>
#include <thread>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cassert>
#include <cmath>

std::atomic<bool> die(false);

std::thread *freenect_thread;
std::mutex back_buf_mutex;

freenect_context *f_ctx;
freenect_device *f_dev;

freenect_video_format video_format = FREENECT_VIDEO_RGB;

// triple buffering
uint8_t *rgb_back, *rgb_mid, *rgb_front;

int window;
int gl_argc;
char **gl_argv;

void gl_draw() {
  std::lock_guard<std::mutex> guard(back_buf_mutex);
}

void gl_resize(int width, int height) {
  glViewport(0, 0, width, height);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(0, 640, 0, 480, -5.0f, 5.0f);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}

void gl_init(int width, int height) {
  glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

  gl_resize(width, height);
}

void gl_runner() {
  glutInit(&gl_argc, gl_argv);
  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
  glutInitWindowSize(640, 480);
  glutInitWindowPosition(0, 0);

  window = glutCreateWindow("Projection Pipeline");

  glutDisplayFunc(&gl_draw);
  glutReshapeFunc(&gl_resize);

  gl_init(640, 480);

  glutMainLoop();
}

void depth_cb(freenect_device *dev, void *v_depth, uint32_t timestamp) {
  std::lock_guard<std::mutex> guard(back_buf_mutex);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->width = 640;
  cloud->height = 480;
  cloud->points.resize(cloud->width * cloud->height);

  uint16_t *depth = (uint16_t*) v_depth;
  for (int col = 0; col < 640; col++) {
    for (int row = 0; row < 480; row++) {
      int cell = row * 480 + col;
      cloud->points[cell].x = row;
      cloud->points[cell].y = col;
      cloud->points[cell].z = depth[cell];
    }
  }

  auto start = std::chrono::high_resolution_clock::now();

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(25);
  seg.setDistanceThreshold(0.01);

  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);

  auto end = std::chrono::high_resolution_clock::now();

  if (inliers->indices.size() == 0) {
    printf("No planes found!\n");
  } else {
    printf("Model coefficients: %.2f, %.2f, %.2f, %.2f (%.2f ms)\n", 
        coefficients->values[0],
        coefficients->values[1],
        coefficients->values[2],
        coefficients->values[3],
        std::chrono::duration<double, std::milli>(end - start).count());
  }
}

void video_cb(freenect_device *dev, void *rgb, uint32_t timestamp) {
  std::lock_guard<std::mutex> guard(back_buf_mutex);
  rgb_back = rgb_mid;
  freenect_set_video_buffer(f_dev, rgb_back);
  rgb_mid = (uint8_t*) rgb;
}

void freenect_runner() {
  freenect_set_depth_callback(f_dev, depth_cb);
  freenect_set_video_callback(f_dev, video_cb);

  freenect_set_depth_mode(f_dev, freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_11BIT));
  freenect_set_video_mode(f_dev, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, video_format));
  freenect_set_video_buffer(f_dev, rgb_back);

  freenect_start_depth(f_dev);
  freenect_start_video(f_dev);

  while (!die) {
    std::this_thread::sleep_for(std::chrono::seconds(2));
  }

  // Stop writing to callbacks
  freenect_stop_video(f_dev);
  freenect_stop_depth(f_dev);

  freenect_close_device(f_dev);
  freenect_shutdown(f_ctx);
}

/**
 * Initializes the freenect_context and the freenect_device.
 */
void init_freenect() {
  if (freenect_init(&f_ctx, NULL) < 0) {
    throw std::runtime_error("freenect_init() failed");
  }

  printf("Setting log level: FREENECT_LOG_DEBUG\n");
  freenect_set_log_level(f_ctx, FREENECT_LOG_DEBUG);

  printf("Selecting freenect subdevices: FREENECT_DEVICE_CAMERA\n");
  freenect_select_subdevices(f_ctx, FREENECT_DEVICE_CAMERA);

  int num_devices = freenect_num_devices(f_ctx);
  int user_device_number = 0;

  printf("Number of devices found: %d\n", num_devices);
  if (num_devices < 1) {
    freenect_shutdown(f_ctx);
    throw std::runtime_error("No device found");
  }

  if (freenect_open_device(f_ctx, &f_dev, user_device_number) < 0) {
    freenect_shutdown(f_ctx);
    throw std::runtime_error("Could not open device");
  }

  freenect_thread = new std::thread(freenect_runner);

  printf("Shutting down freenect\n");
}

int main(int argc, char *argv[]) {
  gl_argc = argc;
  gl_argv = argv;

  rgb_back = new uint8_t[640*400*3];
  rgb_mid = new uint8_t[640*400*3];
  rgb_front = new uint8_t[640*400*3];

  try {
    init_freenect();
  } catch (const std::runtime_error& e) {
    printf(e.what());
  }

  std::thread gl_thread(gl_runner);

  std::this_thread::sleep_for(std::chrono::seconds(10));
  die = true;

  freenect_thread->join();
  gl_thread.join();

  delete freenect_thread;

  delete[] rgb_back;
  delete[] rgb_mid;
  delete[] rgb_front;
}
