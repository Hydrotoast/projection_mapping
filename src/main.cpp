#include "glfw_renderer/camera_builder.hpp"

#include "glfw_renderer/gl_engine.hpp"
#include "glfw_renderer/renderer.hpp"

#include "glfw_renderer/loader.hpp"

#include "glfw_renderer/shape_factory.hpp"
#include "glfw_renderer/scenegraph.hpp"

#include "ransac_cube.hpp"
#include "utility.hpp"
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

using namespace giocc;

/**
 * Concurrency
 **/
std::atomic<bool> die(false);

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
unsigned int WINDOW_WIDTH = 640;
unsigned int WINDOW_HEIGHT = 480;
Renderer<GLEngine> renderer{WINDOW_WIDTH, WINDOW_HEIGHT};
Scenegraph<GLEngine, Cube::N> scene;

/**
 * PCL State
 **/
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr;

void draw_with_renderer(const CubeParams& params)
{
  Eigen::Matrix4f scale;
  scale <<
      10.16, 0,    0,    0,
      0,   10.16,  0,    0,
      0,   0,   10.16,   0,
      0,   0,   0,    1;
  std::clog << "Offset: " << params.translation << std::endl;
  Eigen::Matrix4f transformation = params.rotation * scale;
  Eigen::Vector3f true_translation{params.translation(0), 
      params.translation(1), -params.translation(2)};
  transformation.block<3,1>(0,3) = true_translation;
  scene.geode().transformation(transformation);
  std::clog << "Rendering cube" << std::endl;
}

void pcl_runner() {
  while (true) {
    // Copy depth buffer into point cloud_ptr
    *pcl_back >> pcl_front;

    auto start = std::chrono::high_resolution_clock::now();
    for (int col = 0; col < WINDOW_WIDTH; col++) {
      for (int row = 0; row < WINDOW_HEIGHT; row++) {
        int cell = row * WINDOW_WIDTH + col;

        float x = col;
        float y = row;
        float z = pcl_front[cell];

        float za = zcm(z);
        float xa = xcm(x, za), ya = ycm(y, za);

        cloud_ptr->points[cell].x = xa;
        cloud_ptr->points[cell].y = ya;
        cloud_ptr->points[cell].z = za;
      }
    }
    auto end = std::chrono::high_resolution_clock::now();
    printf("Cloud copy time: (%.4f ms)\n", 
        std::chrono::duration<double, std::milli>(end - start).count());

    // Generate the subcloud_ptrs
    /* start = std::chrono::high_resolution_clock::now(); */
    /* static int denom = 8; */
    /* std::vector<Indices> subcloud_indices = PartitionSubcloudsByNormals(cloud_ptr, M_PI / denom); */
    /* end = std::chrono::high_resolution_clock::now(); */
    /* printf("Subcloud partitioning time: (%.4f ms)\n", */ 
    /*     std::chrono::duration<double, std::milli>(end - start).count()); */

    // Try multiplane segmentation
    start = std::chrono::high_resolution_clock::now();
    Regions regions = SegmentRegions(*cloud_ptr);
    end = std::chrono::high_resolution_clock::now();
    cout << "Time to segment planar regions " << regions.size() << " planar regions: " 
        << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() 
        << " ms" << endl;

    // Find cube given the subcloud_indices
    /* start = std::chrono::high_resolution_clock::now(); */
    /* std::vector<PlaneSummary> plane_summs = FindPlanesInSubclouds(cloud_ptr, subcloud_indices); */
    /* end = std::chrono::high_resolution_clock::now(); */
    /* printf("Finding planes in subclouds time: (%.4f ms)\n", */ 
    /*     std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()); */

    start = std::chrono::high_resolution_clock::now();
    try
    {
      Tuple3 triplet = FindOrthoPlaneTriplet(regions);

      std::clog << "Rendering cube" << std::endl;
      CubeParams params = EstimateCubeParams(triplet, regions);

      draw_with_renderer(params);

      /* float x = params.translation(0), y = params.translation(1), z = params.translation(2); */
      /* float za = zcm(z); */
      /* float xa = xcm(x, za), ya = ycm(y, za); */
      /* std::clog << x << " " << y << " " << z << std::endl; */
      /* std::clog << xa << " " << ya << " " << za << std::endl; */
      /* renderer.translation(Eigen::Vector3f{0, 0, -za}); */
      end = std::chrono::high_resolution_clock::now();
      printf("Cube parameter estimation time: (%.4f ms)\n", 
          std::chrono::duration<double, std::milli>(end - start).count());
    }
    catch (const std::length_error& e)
    {
      std::clog << "[EXCE] Less than three planes found!" << std::endl;
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
  // Allocate cloud_ptr
  cloud_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  cloud_ptr->width = WINDOW_WIDTH;
  cloud_ptr->height = WINDOW_HEIGHT;
  cloud_ptr->points.resize(cloud_ptr->width * cloud_ptr->height);

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

  // Load the shaders
  ShaderProgramFileLoader<GLEngine, FileLoader> loader;
  ShaderProgram<GLEngine> program = loader.load("shader.vert", "shader.frag");
  program();

  // Load the resolvers
  AttributeResolver<GLEngine> attribute_resolver{program};
  UniformResolver<GLEngine> uniform_resolver{program};
  renderer.uniform_resolver(uniform_resolver);

  // Load the geode
  auto shape_ptr = std::move(ShapeFactory<GLEngine, Cube>{program});
  auto geode = std::unique_ptr<Geode<GLEngine, Cube::N>>{
      new Geode<GLEngine, Cube::N>{std::move(shape_ptr)}};
  scene.geode(std::move(geode));

  // Load the default camera
  CameraBuilder<GLEngine> camera_builder{0.61, -400};
  camera_builder.add_persp_matrix();
  camera_builder.add_ndc_matrix(10, 10);
  scene.camera(std::move(camera_builder));

  Eigen::Matrix4f view_translation;
  view_translation <<
      1, 0, 0, 0,
      0, -1, 0, 0,
      0, 0, 1, 0,
      0, 0, 0, 1;
  scene.camera().view(view_translation);

  renderer(scene);

  freenect_thread.join();
  pcl_thread.join();

  // std::this_thread::sleep_for(std::chrono::seconds(120));
  // die = true;
  
  delete [] freenect_depth_buffer;
  delete [] freenect_rgb_buffer;

  delete [] pcl_front;
  delete [] rgb_front;
}
