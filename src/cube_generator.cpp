#include <Eigen/Core>

#include "pcl/point_types.h"
#include "pcl/common/projection_matrix.h"
#include "pcl/io/pcd_io.h"

int main(int argc, char *argv[]) {
  pcl::PointCloud<pcl::PointXYZ> cube; 

  int side = 10 + 1; 
  int size = side * side * 3;
  cube.width = size; 
  cube.height = 1; 
  cube.is_dense = false;
  cube.points.resize(size);

  int p = 0; 
  for (size_t x = 0; x < side; x++) 
    for (size_t y = 0; y < side; y++) 
      cube.points[p++].getVector3fMap() = Eigen::Vector3f(x, y, 0); 

  for (size_t x = 0; x < side; x++) 
    for (size_t z = 0; z < side; z++) 
      cube.points[p++].getVector3fMap() = Eigen::Vector3f(x, side - 1, z); 

  for (size_t y = 0; y < side; y++) 
    for (size_t z = 0; z < side; z++) 
      cube.points[p++].getVector3fMap() = Eigen::Vector3f(side - 1, y, z); 

  pcl::io::savePCDFileASCII("cube.pcd", cube);
  return 0;
}
