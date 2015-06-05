#ifndef RANSAC_CUBE_HPP
#define RANSAC_CUBE_HPP

#include "utility.hpp"

#include <pcl/point_types.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_normal_plane.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/planar_region.h>

#include <vector>

using Point = pcl::PointXYZ;
using Cloud = pcl::PointCloud<Point>;
using Indices = std::vector<int>;
using NormalCloud = pcl::PointCloud<pcl::Normal>;
using PlaneCoeffs = Eigen::Vector4f;

using Region = pcl::PlanarRegion<Point>;
using Regions = std::vector<Region, Eigen::aligned_allocator<Region>>;

// Describes the cube extrinsic parameters in cm
typedef struct CubeParams
{
  Eigen::Matrix4f rotation;
  Eigen::Vector3f translation;
} CubeParams;


// Window parameters
extern unsigned int WINDOW_WIDTH;
extern unsigned int WINDOW_HEIGHT;

// Depth calibration
extern float k1;
extern float k2;
extern float k3;

// Coordinate transformation
float zcm(float z);
float xcm(float x, float zcm);
float ycm(float y, float zcm);

/* // Finds a single plane in each subcloud and stores the result in the plane */
/* // summaries vector. */
/* std::vector<Region> */
/* FindPlanesInSubclouds(Cloud::Ptr cloud_ptr, std::vector<Indices>& subclouds); */

// Cost function for three normal vectors estimating a cube which is primarily
// determined by their orthogonality with each other.
double
CubeCost(Eigen::Vector3f& n1, Eigen::Vector3f& n2, Eigen::Vector3f& n3);

// Finds a triplet of orthogonal planes in the vector of plane summaries that
// maximizes the CubeCost function.
Tuple3
FindOrthoPlaneTriplet(Regions& plane_summs);

// Estimates the extrinsic parameters of the cube in centimeters.
CubeParams
EstimateCubeParams(const Tuple3& triplet, Regions& plane_summs);

// Estimates cloud normals from the specified cloud pointer and stores the
// results in the normals cloud.
NormalCloud
EstimateCloudNormals(Cloud::Ptr cloud_ptr);

/* // Partitions the cloud pointed to by the cloud pointer into subclouds based on */
/* // the similiarty of their normal vectors. */
/* std::vector<Indices> */
/* PartitionSubcloudsByNormals(Cloud::Ptr cloud_ptr, double threshold); */

// Segment input cloud into planar regions with inliers determined by an
// angular threshold and a distance threshold
Regions SegmentRegions(Cloud input_cloud);

#endif
