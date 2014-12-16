#include <Eigen/Core>

#include "pcl/point_types.h"
#include "pcl/common/projection_matrix.h"
#include "pcl/io/pcd_io.h"

int main(int argc, char *argv[]) {
  pcl::PointCloud<pcl::PointXYZ> cube; 

  int side = 10 + 1; 
  int size = pow(side, 3);
  cube.width = size; 
  cube.height = 1; 
  cube.is_dense = false;
  cube.points.resize(size);

  int p = 0; 
  for (size_t i = 0; i < side; i++) 
    for (size_t j = 0; j < side; j++) 
      for (size_t k = 0; k < side; k++, p++) 
        cube.points[p].getVector3fMap() = Eigen::Vector3f(i, j, k); 

  pcl::io::savePCDFileASCII("cube.pcd", cube);
  return 0;
}
