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
using PlaneCoeffs = Eigen::VectorXf;

using Regions = std::vector<pcl::PlanarRegion<Point>,
  Eigen::aligned_allocator<pcl::PlanarRegion<Point>>>;

typedef struct PlaneSummary
{
  // Points to the subcloud where the points were found
  Cloud::Ptr subcloud_ptr;

  PlaneCoeffs coeffs;
  std::vector<int> inliers;
  size_t points_size;
} PlaneSummary;

// Finds a single plane in each subcloud and stores the result in the plane
// summaries vector.
std::vector<PlaneSummary>
FindPlanesInSubclouds(Cloud::Ptr cloud_ptr, std::vector<Indices>& subclouds);

// Cost function for three normal vectors estimating a cube which is primarily
// determined by their orthogonality with each other.
double
CubeCost(Eigen::Vector3f& n1, Eigen::Vector3f& n2, Eigen::Vector3f& n3);

// Finds a triplet of orthogonal planes in the vector of plane summaries that
// maximizes the CubeCost function.
Tuple3
FindOrthoPlaneTriplet(std::vector<PlaneSummary>& plane_summs);

// Extracts model coefficients from the plane summaries into a vector of
// ModelCoefficients for displaying on the viewer.
std::vector<pcl::ModelCoefficients>
ExtractModelCoefficients(std::vector<PlaneSummary>& plane_summs);

// Estimates cloud normals from the specified cloud pointer and stores the
// results in the normals cloud.
NormalCloud
EstimateCloudNormals(Cloud::Ptr cloud_ptr);

// Partitions the cloud pointed to by the cloud pointer into subclouds based on
// the similiarty of their normal vectors.
std::vector<Indices>
PartitionSubcloudsByNormals(Cloud::Ptr cloud_ptr, double threshold);

// Segment input cloud into planar regions with inliers determined by an
// angular threshold and a distance threshold
Regions SegmentRegions(Cloud::Ptr input_cloud);

#endif
