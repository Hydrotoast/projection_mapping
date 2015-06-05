#include "ransac_cube.hpp"

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>

#define _USE_MATH_DEFINES
#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <iterator>

using namespace pcl;
using namespace Eigen;
using namespace std;

float k1 = 0.1236;
float k2 = 2842.5;
float k3 = 1.1863;

float zcm(float z) { return k1 * tanf(z / k2 + k3) * 100; }

float xcm(float x, float zcm)
{
  static float asp = 640 / 480.0;
  return (x - 640 / 2) * (zcm - 10) * .0021 * 1;
}

float ycm(float y, float zcm)
{
  return (y - 480 / 2) * (zcm - 10) * .0021;
}

// Finds a single plane in each subcloud and stores the result in the plane
// summaries vector.
/* vector<Region> */
/* FindPlanesInSubclouds(Cloud::Ptr cloud_ptr, vector<Indices>& subclouds) */
/* { */
/*   vector<Region> regions; */
/*   for (size_t i = 1; i < subclouds.size(); i++) { */
/*     Indices& indices = subclouds.at(i); */
    
/*     // Skip clouds with less than three points */
/*     /1* clog << "Number of points in subcloud: " << cloud_ptr->size() << endl; *1/ */
/*     if (indices.size() < 3) */
/*       continue; */

/*     SampleConsensusModelPlane<PointXYZ>::Ptr */ 
/*       model_ptr(new SampleConsensusModelPlane<PointXYZ>(cloud_ptr, indices)); */
/*     RandomSampleConsensus<PointXYZ> ransac(model_ptr); */
/*     Eigen::VectorXf coeffs; */
/*     ransac.setDistanceThreshold(100.0); */
/*     ransac.computeModel(); */
/*     ransac.getModelCoefficients(coeffs); */

/*     assert(coeffs.size() >= 4); */
/*     Cloud::VectorType v{}; */
/*     Eigen::Vector4f coeffs4{coeffs(0), coeffs(1), coeffs(2), coeffs(3)}; */
/*     regions.push_back(Region{ */
/*         pcl::Region3D<Point>{}, */ 
/*         pcl::PlanarPolygon<Point>{v, coeffs4}}); */
/*   } */
/*   return regions; */
/* } */

// Cost function for three normal vectors estimating a cube which is primarily
// determined by their orthogonality with each other.
double
CubeCost(Region& r1, Region& r2, Region& r3)
{
  Vector3f n1{r1.getCoefficients().head<3>()};
  Vector3f n2{r2.getCoefficients().head<3>()};
  Vector3f n3{r2.getCoefficients().head<3>()};

  Vector3f n12_perp = n1.cross(n2);
  Vector3f n23_perp = n2.cross(n3);
  Vector3f n13_perp = n1.cross(n3);

  double c3 = fabs(n12_perp.dot(n3));
  double c1 = fabs(n23_perp.dot(n1));
  double c2 = fabs(n13_perp.dot(n2));

  const Vector3f& cent1 = r1.getCentroid();
  const Vector3f& cent2 = r2.getCentroid();
  const Vector3f& cent3 = r3.getCentroid();

  return c3 + c1 + c2;
}

// Finds a triplet of orthogonal planes in the vector of plane summaries that
// maximizes the CubeCost function.
Tuple3 FindOrthoPlaneTriplet(Regions& regions)
{
  if (regions.size() < 3)
    throw std::length_error{"Needs at least three regions."};
  vector<Tuple3> triplets = GenerateTriplets(regions.size() - 1);
  /* clog << "Number of planes: " << plane_summs.size() << endl; */
  /* clog << "Number of triplets: " << triplets.size() << endl; */
  Tuple3 *best_triplet;
  double best_cost = numeric_limits<double>::min();
  double best_unnormalized_cost = numeric_limits<double>::min();
  for (Tuple3 &triplet : triplets) {
    Region &r1 = regions.at(triplet.at(0));
    Region &r2 = regions.at(triplet.at(1));
    Region &r3 = regions.at(triplet.at(2));
    double unnormalized_cost = CubeCost(r1, r2, r3);
    /* double weights = region1. */
    double cost = unnormalized_cost;
    if (cost > best_cost) {
      best_cost = cost;
      best_unnormalized_cost = unnormalized_cost;
      best_triplet = &triplet;
    }
    /* clog << "Triplet: "; */
    /* for (size_t i : triplet) */
    /*   clog << i << " "; */
    /* clog << " Cost: " << cost << endl; */
  }
  clog << "Best triplet: ";
  for (size_t i : *best_triplet)
    clog << i << " ";
  clog << endl;
  clog << "Unnormalized cost: " << best_unnormalized_cost << endl;
  return *best_triplet;
}

// Estimates the extrinsic parameters of the cube in centimeters.
CubeParams
EstimateCubeParams(const Tuple3& triplet, Regions& regions)
{
  CubeParams params;

  // Construct system of equations
  Eigen::Matrix3f A;
  Eigen::Vector3f b;
  int row = 0;
  for (size_t i : triplet) {
    Eigen::Vector4f& v = regions.at(i).getCoefficients();
    A.row(row) = v.head<3>();
    b(row) = v.tail<4>()(1);
    row++;
  }

  // Solve for corner in system
  /* Eigen::FullPivHouseholderQR<Eigen::Matrix3f> QR{A}; */
  params.translation = A.transpose() * -b;

  // Construct rotation from normals
  params.rotation.setIdentity(4, 4);
  params.rotation.block<3,3>(0, 0) = A;

  return params;
}

// Estimates cloud normals from the specified cloud pointer and stores the
// results in the normals cloud.
NormalCloud
EstimateCloudNormals(Cloud::Ptr cloud_ptr)
{
  NormalCloud normals;
  IntegralImageNormalEstimation<Point, Normal> ne;
  ne.setNormalEstimationMethod(ne.AVERAGE_DEPTH_CHANGE);
  ne.setMaxDepthChangeFactor(0.3f);
  ne.setNormalSmoothingSize(4.0f);
  ne.setInputCloud(cloud_ptr);
  ne.compute(normals); 
  return normals;
}

/* // Partitions the cloud pointed to by the cloud pointer into subclouds based on */
/* // the similiarty of their normal vectors. */
/* std::vector<Indices> */
/* PartitionSubcloudsByNormals(Cloud::Ptr cloud_ptr, double threshold) */
/* { */
/*   std::vector<vector<int>> output; */

/*   clog << endl << "Finding cloud normals" << endl; */
/*   auto start = chrono::steady_clock::now(); */
/*   NormalCloud normals = EstimateCloudNormals(cloud_ptr); */
/*   auto end = chrono::steady_clock::now(); */
/*   assert(normals.size() == cloud_ptr->size()); */

/*   clog << "Time for finding cloud normals: " */ 
/*       << chrono::duration_cast<chrono::milliseconds>(end - start).count() << " ms" */
/*       << endl; */

/*   // Allocate and reserve space for index data structures */
/*   vector<size_t> indices; */
/*   vector<size_t> new_indices; */
/*   indices.reserve(cloud_ptr->size()); */
/*   new_indices.reserve(cloud_ptr->size()); */

/*   // Initialize indices to search */
/*   for (size_t i = 0; i < cloud_ptr->size(); i++) */
/*     indices.push_back(i); */

/*   // Building subclouds */
/*   /1* clog << "Building subclouds" << endl; *1/ */
/*   while (!indices.empty()) { */
/*     /1* clog << indices.size() << " points remaining" << endl; *1/ */
/*     // Obtain representative point and its normal */
/*     while (!isfinite(normals.at(indices.back()).normal_x)) { */
/*       indices.pop_back(); */
/*     } */
/*     int rep_index = indices.back(); */
/*     indices.pop_back(); */
/*     PointXYZ &rep = cloud_ptr->at(rep_index); */
/*     Normal &rep_normal = normals[rep_index]; */

/*     // Add representative point to its own subcloud */
/*     output.push_back(vector<int>{}); */
/*     output.reserve(cloud_ptr->size() / 2); */
/*     output.back().push_back(rep_index); */
/*     assert(output.back().size() == 1); */

/*     // Add points to the subcloud if they have similar normals to the */
/*     // representative point */
/*     for (size_t i : indices) { */
/*       if (!isfinite(normals.at(i).normal_x)) */
/*         continue; */

/*       PointXYZ &point = cloud_ptr->at(i); */
/*       Normal &point_normal = normals[i]; */
/*       float similarity = abs(rep_normal.getNormalVector4fMap().dot(point_normal.getNormalVector4fMap())); */
/*       if (similarity > 0.9) { */
/*         output.back().push_back(i); */
/*       } else { */
/*         new_indices.push_back(i); */
/*       } */
/*     } */

/*     /1* clog << "Number of points in subcloud " << output.size() << ": " *1/ */ 
/*         /1* << output.back().size() << endl; *1/ */
/*     assert(new_indices.size() <= indices.size()); */

/*     indices.swap(new_indices); */
/*     new_indices.clear(); */
/*   } */

/*   clog << "Number of subclouds found: " << output.size() << endl; */
/*   assert(output.size() > 1); */
/*   output.erase(output.begin()); */
/*   return output; */
/* } */

// Segment input cloud into planar regions with inliers determined by an
// angular threshold and a distance threshold
Regions SegmentRegions(Cloud input_cloud)
{
  Cloud::Ptr cloud_ptr{input_cloud.makeShared()};
  NormalCloud normal_cloud = EstimateCloudNormals(cloud_ptr);
  NormalCloud::Ptr normal_cloud_ptr{normal_cloud.makeShared()};
  Regions regions;
  pcl::OrganizedMultiPlaneSegmentation<Point, Normal, pcl::Label> mps;
  mps.setMinInliers(400);
  mps.setAngularThreshold(0.017453 * 4.0); // 4 degrees
  mps.setDistanceThreshold(0.05); // 5 cm
  mps.setInputNormals(normal_cloud_ptr);
  mps.setInputCloud(cloud_ptr);
  mps.segmentAndRefine(regions);
  return regions;
}

