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

// Finds a single plane in each subcloud and stores the result in the plane
// summaries vector.
vector<PlaneSummary>
FindPlanesInSubclouds(Cloud::Ptr cloud_ptr, vector<Indices>& subclouds)
{
  vector<PlaneSummary> plane_summs;
  for (size_t i = 1; i < subclouds.size(); i++) {
    Indices& indices = subclouds.at(i);
    
    // Skip clouds with less than three points
    /* clog << "Number of points in subcloud: " << cloud_ptr->size() << endl; */
    if (indices.size() < 3)
      continue;

    plane_summs.push_back(PlaneSummary());
    plane_summs.back().subcloud_ptr = cloud_ptr; // makes a copy of the cloud ptr
    plane_summs.back().coeffs = Vector4f{};
    plane_summs.back().points_size = indices.size();

    SampleConsensusModelPlane<PointXYZ>::Ptr 
      model_ptr(new SampleConsensusModelPlane<PointXYZ>(cloud_ptr, indices));

    RandomSampleConsensus<PointXYZ> ransac(model_ptr);
    ransac.setDistanceThreshold(100.0);
    ransac.computeModel();
    ransac.getInliers(plane_summs.back().inliers);
    ransac.getModelCoefficients(plane_summs.back().coeffs);
  }
  return plane_summs;
}

// Cost function for three normal vectors estimating a cube which is primarily
// determined by their orthogonality with each other.
double
CubeCost(Vector3f &n1, Vector3f &n2, Vector3f &n3)
{
  Vector3f n12_perp = n1.cross(n2);
  Vector3f n23_perp = n2.cross(n3);
  Vector3f n13_perp = n1.cross(n3);
  double c3 = fabs(n12_perp.dot(n3));
  double c1 = fabs(n23_perp.dot(n1));
  double c2 = fabs(n13_perp.dot(n2));
  return c3 + c1 + c2;
}

// Finds a triplet of orthogonal planes in the vector of plane summaries that
// maximizes the CubeCost function.
Tuple3 FindOrthoPlaneTriplet(vector<PlaneSummary>& plane_summs)
{
  vector<Tuple3> triplets = GenerateTriplets(plane_summs.size() - 1);
  /* clog << "Number of planes: " << plane_summs.size() << endl; */
  /* clog << "Number of triplets: " << triplets.size() << endl; */
  Tuple3 *best_triplet;
  double best_cost = numeric_limits<double>::min();
  double best_unnormalized_cost = numeric_limits<double>::min();
  for (Tuple3 &triplet : triplets) {
    PlaneSummary &summ1 = plane_summs.at(triplet.at(0));
    PlaneSummary &summ2 = plane_summs.at(triplet.at(1));
    PlaneSummary &summ3 = plane_summs.at(triplet.at(2));
    VectorXf p1{summ1.coeffs};
    p1.conservativeResize(3, 1);
    VectorXf p2{summ2.coeffs};
    p2.conservativeResize(3, 1);
    VectorXf p3{summ3.coeffs};
    p3.conservativeResize(3, 1);
    Vector3f n1{p1}, n2{p2}, n3{p3};
    double unnormalized_cost = CubeCost(n1, n2, n3);
    double cost = unnormalized_cost * (summ1.points_size + summ2.points_size + summ3.points_size);
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

// Extracts model coefficients from the plane summaries into a vector of
// ModelCoefficients for displaying on the viewer.
vector<ModelCoefficients> 
ExtractModelCoefficients(vector<PlaneSummary>& plane_summs)
{
  vector<ModelCoefficients> coeffs;
  for (PlaneSummary &plane_summ : plane_summs) {
    VectorXf &vector_coeffs = plane_summ.coeffs;
    assert(vector_coeffs.size() == 4);

    ModelCoefficients coeffs_obj;
    for (int i = 0; i < vector_coeffs.size(); i++) {
      coeffs_obj.values.push_back(vector_coeffs(i));
      clog << vector_coeffs(i) << " ";
    }
    clog << endl;

    coeffs.push_back(coeffs_obj);
  }
  return coeffs;
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

// Partitions the cloud pointed to by the cloud pointer into subclouds based on
// the similiarty of their normal vectors.
std::vector<Indices>
PartitionSubcloudsByNormals(Cloud::Ptr cloud_ptr, double threshold)
{
  std::vector<vector<int>> output;

  clog << endl << "Finding cloud normals" << endl;
  auto start = chrono::steady_clock::now();
  NormalCloud normals = EstimateCloudNormals(cloud_ptr);
  auto end = chrono::steady_clock::now();
  assert(normals.size() == cloud_ptr->size());

  clog << "Time for finding cloud normals: " 
      << chrono::duration_cast<chrono::milliseconds>(end - start).count() << " ms"
      << endl;

  // Allocate and reserve space for index data structures
  vector<size_t> indices;
  vector<size_t> new_indices;
  indices.reserve(cloud_ptr->size());
  new_indices.reserve(cloud_ptr->size());

  // Initialize indices to search
  for (size_t i = 0; i < cloud_ptr->size(); i++)
    indices.push_back(i);

  // Building subclouds
  /* clog << "Building subclouds" << endl; */
  while (!indices.empty()) {
    /* clog << indices.size() << " points remaining" << endl; */
    // Obtain representative point and its normal
    while (!isfinite(normals.at(indices.back()).normal_x)) {
      indices.pop_back();
    }
    int rep_index = indices.back();
    indices.pop_back();
    PointXYZ &rep = cloud_ptr->at(rep_index);
    Normal &rep_normal = normals[rep_index];

    // Add representative point to its own subcloud
    output.push_back(vector<int>{});
    output.reserve(cloud_ptr->size() / 2);
    output.back().push_back(rep_index);
    assert(output.back().size() == 1);

    // Add points to the subcloud if they have similar normals to the
    // representative point
    for (size_t i : indices) {
      if (!isfinite(normals.at(i).normal_x))
        continue;

      PointXYZ &point = cloud_ptr->at(i);
      Normal &point_normal = normals[i];
      float similarity = abs(rep_normal.getNormalVector4fMap().dot(point_normal.getNormalVector4fMap()));
      if (similarity > 0.9) {
        output.back().push_back(i);
      } else {
        new_indices.push_back(i);
      }
    }

    /* clog << "Number of points in subcloud " << output.size() << ": " */ 
        /* << output.back().size() << endl; */
    assert(new_indices.size() <= indices.size());

    indices.swap(new_indices);
    new_indices.clear();
  }

  clog << "Number of subclouds found: " << output.size() << endl;
  assert(output.size() > 1);
  output.erase(output.begin());
  return output;
}

// Segment input cloud into planar regions with inliers determined by an
// angular threshold and a distance threshold
Regions SegmentRegions(Cloud::Ptr input_cloud)
{
  NormalCloud normal_cloud = EstimateCloudNormals(input_cloud);
  NormalCloud::Ptr normal_cloud_ptr{normal_cloud.makeShared()};
  Regions regions;
  pcl::OrganizedMultiPlaneSegmentation<Point, Normal, pcl::Label> mps;
  mps.setMinInliers(400);
  mps.setAngularThreshold(0.017453 * 4.0); // 4 degrees
  mps.setDistanceThreshold(0.05); // 5 cm
  mps.setInputNormals(normal_cloud_ptr);
  mps.setInputCloud(input_cloud);
  mps.segmentAndRefine(regions);
  return regions;
}

