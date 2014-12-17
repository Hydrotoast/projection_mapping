#include "buffers.h"

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
#include <condition_variable>
#include <mutex>
#include <stdexcept>
#include <thread>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cassert>
#include <cmath>

#define PKTS_PER_XFER 32
#define NUM_XFERS 6

/**
 * Concurrency
 **/
std::atomic<bool> die(false);

std::mutex rgb_back_buf_mutex;

/**
 * Freenect state
 **/
freenect_context *f_ctx;
freenect_device *f_dev;

/**
 * Default video format
 **/
freenect_video_format video_format = FREENECT_VIDEO_IR_8BIT;

/**
 * Triple buffering where
 * rgb_back is for freenect
 * rgb_mid is for PCL
 * rgb_front is for OpenGL
 **/
buffers::SharedBuffer<uint16_t> *pcl_back;
uint16_t *freenect_depth_buffer, *pcl_front;

buffers::SharedBuffer<uint8_t> *rgb_back;
uint8_t *freenect_rgb_buffer, *rgb_front;

/**
 * OpenGL state
 **/
int window;
int WINDOW_WIDTH = 640;
int WINDOW_HEIGHT = 480;

/**
 * PCL State
 **/
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

void opengl_draw() { }

void opengl_resize(int width, int height) {
  glViewport(0, 0, width, height);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(0, WINDOW_WIDTH, 0, WINDOW_HEIGHT, -5.0f, 5.0f);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}

void opengl_init(int width, int height) {
  glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

  opengl_resize(width, height);
}

/**
 * Executes the OpenGL thread
 **/
void opengl_runner(int argc, char* argv[]) {
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
  glutInitWindowSize(WINDOW_WIDTH, WINDOW_HEIGHT);
  glutInitWindowPosition(0, 0);

  window = glutCreateWindow("Projection Pipeline");

  glutDisplayFunc(&opengl_draw);
  glutReshapeFunc(&opengl_resize);

  opengl_init(WINDOW_WIDTH, WINDOW_HEIGHT);

  glutMainLoop();
}

void pcl_runner() {
  while (true) {
    // Copy depth buffer into point cloud
    *pcl_back >> pcl_front;

    for (int col = 0; col < WINDOW_WIDTH; col++) {
      for (int row = 0; row < WINDOW_HEIGHT; row++) {
        int cell = row * WINDOW_HEIGHT + col;
        cloud->points[cell].x = row;
        cloud->points[cell].y = col;
        cloud->points[cell].z = pcl_front[cell];
      }
    }

    // Perform plane segmentation and time it
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
}

void depth_cb(freenect_device *dev, void *v_depth, uint32_t timestamp) {
  assert(freenect_depth_buffer == v_depth);
  *pcl_back << freenect_depth_buffer;
  freenect_set_depth_buffer(f_dev, freenect_depth_buffer);
}

void video_cb(freenect_device *dev, void *rgb, uint32_t timestamp) {
  assert(freenect_rgb_buffer == rgb);
  *rgb_back << freenect_rgb_buffer;
  freenect_set_video_buffer(f_dev, freenect_rgb_buffer);
}

/**
 * Executes the freenect pipeline
 **/
void freenect_runner() {
  // Setup callbacks
  freenect_set_depth_callback(f_dev, depth_cb);
  freenect_set_video_callback(f_dev, video_cb);

  // Setup modes (determines buffer parameters) and attach buffers
  freenect_set_depth_mode(f_dev, freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_11BIT));
  freenect_set_depth_buffer(f_dev, freenect_depth_buffer);
  freenect_set_video_mode(f_dev, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, video_format));
  freenect_set_video_buffer(f_dev, freenect_rgb_buffer);

  // Start streaming
  freenect_start_depth(f_dev);
  freenect_start_video(f_dev);

  // Process events synchronously?
  while (!die && freenect_process_events(f_ctx) >= 0) {
    fflush(stdout);
  }

  // Stop writing to callbacks
  printf("Shutting down freenect\n");
  freenect_stop_video(f_dev);
  freenect_stop_depth(f_dev);

  // Cleanup resources
  freenect_close_device(f_dev);
  freenect_shutdown(f_ctx);
}

/**
 * Initializes the freenect_context and the freenect_device.
 **/
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
}

int main(int argc, char *argv[]) {
  // Allocate cloud
  cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->width = WINDOW_WIDTH;
  cloud->height = WINDOW_HEIGHT;
  cloud->points.resize(cloud->width * cloud->height);

  // Allocate depth buffers
  freenect_depth_buffer = new uint16_t[WINDOW_WIDTH * WINDOW_HEIGHT];
  pcl_back = new buffers::SharedBuffer<uint16_t>(WINDOW_WIDTH, WINDOW_HEIGHT);
  pcl_front = new uint16_t[WINDOW_WIDTH * WINDOW_HEIGHT];

  // Allocate RGB buffers
  freenect_rgb_buffer = new uint8_t[WINDOW_WIDTH * WINDOW_HEIGHT];
  rgb_back = new buffers::SharedBuffer<uint8_t>(WINDOW_WIDTH, WINDOW_HEIGHT);
  rgb_front = new uint8_t[WINDOW_WIDTH * WINDOW_HEIGHT];

  // Several errors can arise from this
  try {
    init_freenect();
  } catch (const std::runtime_error& e) {
    printf(e.what());
  }

  std::thread freenect_thread(freenect_runner);
  std::thread pcl_thread(pcl_runner);

  opengl_runner(argc, argv);

  freenect_thread.join();
  pcl_thread.join();

  // std::this_thread::sleep_for(std::chrono::seconds(120));
  // die = true;
  
  delete [] freenect_depth_buffer;
  delete [] freenect_rgb_buffer;

  delete [] pcl_front;
  delete [] rgb_front;
}
