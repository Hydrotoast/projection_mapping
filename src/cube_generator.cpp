#include <Eigen/Core>

#include "pcl/point_types.h"
#include "pcl/common/projection_matrix.h"
#include "pcl/common/transforms.h"
#include "pcl/io/pcd_io.h"

#define _USE_MATH_DEFINES
#include <math.h>

#define QUARTER_CCW (M_PI / 4)
#define QUARTER_CW (-M_PI / 4)

void FillCloud(pcl::PointCloud<pcl::PointXYZ> &cube, int side) {
  int p = 0; 
  
  // Front face
  for (size_t x = 0; x < side; x++) 
    for (size_t y = 0; y < side; y++) 
      cube.points[p++].getVector3fMap() = Eigen::Vector3f(x, y, 0); 

  // Top face
  for (size_t x = 0; x < side; x++) 
    for (size_t z = 0; z < side; z++) 
      cube.points[p++].getVector3fMap() = Eigen::Vector3f(x, side - 1, z); 

  // Right face
  for (size_t y = 0; y < side; y++) 
    for (size_t z = 0; z < side; z++) 
      cube.points[p++].getVector3fMap() = Eigen::Vector3f(side - 1, y, z); 
}

void RotateCloud(pcl::PointCloud<pcl::PointXYZ> &rotated_cube, 
                 pcl::PointCloud<pcl::PointXYZ> &cube) {
  // Build rotation matrix
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.rotate(Eigen::AngleAxisf(QUARTER_CW, Eigen::Vector3f::UnitZ()));
  transform.rotate(Eigen::AngleAxisf(QUARTER_CCW, Eigen::Vector3f::UnitX()));

  // Transform source cloud with transformation matrix
  pcl::transformPointCloud(cube, rotated_cube, transform);
}

void ProjectCloud(pcl::PointCloud<pcl::PointXYZ> &rotated_cube) {
  for 
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

  pcl::PointCloud<pcl::PointXYZ> rotated_cube; 
  cube.width = size; 
  cube.height = 1; 
  cube.points.resize(size);
  RotateCloud(rotated_cube, cube);

  pcl::io::savePCDFileASCII("cube.pcd", rotated_cube);
  return 0;
}
