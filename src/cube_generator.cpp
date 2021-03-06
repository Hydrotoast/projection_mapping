#include <Eigen/Core>

#include "pcl/point_types.h"
#include "pcl/common/projection_matrix.h"
#include "pcl/common/transforms.h"
#include "pcl/io/pcd_io.h"

#define _USE_MATH_DEFINES
#include <math.h>

//#define HALF_CCW (M_PI)
#define QUARTER_CCW (M_PI / 4)
#define QUARTER_CW (-M_PI / 4)

#define WIDTH 640
#define HEIGHT 480

// Fills the front, top, and right faces of a cube.
void FillCloud(pcl::PointCloud<pcl::PointXYZ> &cube, int side) {
  int p = 0;

  std::cout << "Generating sides from " 
      << 0 << " to " 
      << side - 1 << std::endl;
  
  // Front face
  for (size_t x = 0; x < side; x++) 
    for (size_t y = 0; y < side; y++) 
      cube.points[p++].getVector3fMap() = Eigen::Vector3f(x, y, side - 1); 

  // Top face
  for (size_t x = 0; x < side; x++) 
    for (size_t z = 0; z < side; z++) 
      cube.points[p++].getVector3fMap() = Eigen::Vector3f(x, side - 1, z); 

  // Right face
  for (size_t y = 0; y < side; y++) 
    for (size_t z = 0; z < side; z++) 
      cube.points[p++].getVector3fMap() = Eigen::Vector3f(side - 1, y, z); 

  Eigen::Affine3f transform(
    Eigen::Translation<float,3>(-(side/2), -(side/2), -(side/2)));

  // Transform cloud to center
  pcl::transformPointCloud(cube, cube, transform);
}

void RotateCloud(pcl::PointCloud<pcl::PointXYZ> &rotated_cube, 
                 pcl::PointCloud<pcl::PointXYZ> &cube,
                 int side) {
  // Build rotation matrix
  Eigen::Affine3f transform(
    Eigen::Translation<float,3>(WIDTH / 4,  HEIGHT / 4, side / 2)
    * Eigen::AngleAxisf(QUARTER_CW, Eigen::Vector3f::UnitY()) 
    * Eigen::AngleAxisf(QUARTER_CCW, Eigen::Vector3f::UnitX()));

  // Transform source cloud with transformation matrix
  pcl::transformPointCloud(cube, rotated_cube, transform);
}

void MakeDense(pcl::PointCloud<pcl::PointXYZ> &dense_cloud, 
               pcl::PointCloud<pcl::PointXYZ> &cloud, 
               int width, int height) {
  dense_cloud.width = width;
  dense_cloud.height = height;
  dense_cloud.resize(width * height);

  // Initialize dense image as zeros
  for (size_t y = 0; y < height; y++) 
    for (size_t x = 0; x < width; x++) 
      dense_cloud.points[y * width + x].getVector3fMap() = Eigen::Vector3f(x, y, 0); 

  for (auto point : cloud)
  {
    assert(lround(point.y) * width + lround(point.x) >= 0);
    float x_round = lround(point.x), y_round = lround(point.y);

    auto current_point = dense_cloud.points[y_round * width + x_round].getVector3fMap();
    auto new_point = Eigen::Vector3f(x_round, y_round, point.z);

    if (new_point(2) <= current_point(2))
      continue;
    
    current_point = Eigen::Vector3f(x_round, y_round, point.z);
  }
}

int main(int argc, char *argv[]) {
  int side = 10 + 1; 

  if (argc > 1)
    side = atoi(argv[1]) + 1;

  // Number of points in the cloud
  int size = side * side * 3;

  pcl::PointCloud<pcl::PointXYZ> cube; 
  cube.width = size; 
  cube.height = 1; 
  cube.points.resize(size);
  FillCloud(cube, side);

  std::cout << "Rotating cube" << std::endl;
  pcl::PointCloud<pcl::PointXYZ> rotated_cube; 
  RotateCloud(rotated_cube, cube, side);

  int dense = 1;
  if (argc > 2)
    int dense = atoi(argv[2]);

  pcl::PointCloud<pcl::PointXYZ> final_cube = rotated_cube; 
  if (dense) {
    std::cout << "Making cube dense" << std::endl;
    MakeDense(final_cube, rotated_cube, WIDTH, HEIGHT);
  }

  pcl::io::savePCDFileASCII("cube.pcd", final_cube);
  return 0;
}
