#include "utility.hpp"

#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_normal_plane.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <set>
#include <iterator>
#include <algorithm>

#define _USE_MATH_DEFINES
#include <cassert>
#include <cmath>

#include <thread>
#include <mutex>
#include <memory>
#include <chrono>

#define AXIS_SCALE 160.0
#define POINT_SIZE 3

using namespace pcl;
using namespace Eigen;
using namespace std;

typedef visualization::PCLVisualizer Viewer;
typedef shared_ptr<Viewer> ViewerPtr;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> CloudT;
typedef pcl::PointCloud<Normal> NormalCloudT;
typedef Eigen::VectorXf PlaneCoeffsT;

typedef struct PlaneSummaryT {
  // Points to the subcloud where the points were found
  CloudT::Ptr subcloud_ptr;

  PlaneCoeffsT coeffs;
  vector<int> inliers;
  size_t points_size;
} PlaneSummaryT;

bool running;

// Returns a shared pointer to a viewer object initialized with the clouds
// parameter. The viewer is initialized with a black background and standard
// (X, Y, Z) coordinate axes.
ViewerPtr InitViewer() {
  ViewerPtr viewer_ptr{new Viewer{"3D Viewer"}};
  viewer_ptr->setBackgroundColor(0, 0, 0);  // black

  viewer_ptr->addCoordinateSystem(AXIS_SCALE, "global");
  viewer_ptr->initCameraParameters();
  return viewer_ptr;
}


// Adds each cloud in the `clouds` vector to the shared viewer specified by
// `viewer_ptr`. Each cloud in the vector is colored with shade of red
// and green.
void AddClouds(ViewerPtr viewer_ptr, vector<CloudT::Ptr>& clouds) {
  clog << "Adding " << clouds.size() << " clouds to the viewer" << endl;
  int stride = 255 / clouds.size();
  for (size_t i = 0; i < clouds.size(); i++) {
    string cloud_name{"cloud"};
    cloud_name += i;
    visualization::PointCloudColorHandlerCustom<PointT> color(
        clouds.at(i), stride * i, 255 - (stride * i), 0);
    viewer_ptr->addPointCloud<PointT>(clouds.at(i), color, cloud_name);
    viewer_ptr->setPointCloudRenderingProperties(
        visualization::PCL_VISUALIZER_POINT_SIZE, POINT_SIZE, cloud_name);

  }
}

// Adds a plane for each of the model coefficients given by the `coeffs` vector
// to the shared viewer specified by `viewer_ptr`.
void AddPlanes(ViewerPtr viewer_ptr, vector<ModelCoefficients> &coeffs) {
  clog << "Adding " << coeffs.size() << " planes to the viewer" << endl;
  for (size_t i = 0; i < coeffs.size(); i++) {
    string name{"plane"};
    name += i;
    viewer_ptr->addPlane(coeffs.at(i), name);
  }
}

/* void UpdateCloudNormals( */
/*     PointCloud<PointXYZ>::ConstPtr cloud, */
/*     PointCloud<Normal>::ConstPtr normals) { */
/*   clog << "Updating point cloud and normals" << endl; */
/*   visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green_color(cloud, 0, 255, 0); // green */
/*   main_viewer_ptr->updatePointCloud<PointXYZ>(cloud, green_color, "cloud"); */

/*   main_viewer_ptr->removePointCloud("normals"); */
/*   main_viewer_ptr->addPointCloudNormals<PointXYZ, Normal>(cloud, normals, 40, 40, "normals"); */
/* } */

// Runs the shared viewer specified by the `viewer_ptr`.
void ViewerTask(ViewerPtr viewer_ptr) {
  while (!viewer_ptr->wasStopped()) {
    viewer_ptr->spinOnce(100);
    this_thread::sleep_for(chrono::milliseconds(500));
  }
}

// Loads the cube point cloud into the cloud reference parameter.
void LoadCubePC(CloudT &cloud) {
  if (io::loadPCDFile<PointXYZ>("cube.pcd", cloud) == -1) {
    cerr << "Could not open cube.pcd" << endl;
    exit(-1);
  }
}

void FindPlanarInliers(vector<int> &inliers, 
                       CloudT::Ptr output_cloud_ptr, 
                       CloudT::Ptr input_cloud_ptr) {
  // created RandomSampleConsensus object and compute the appropriated model
  /* SampleConsensusModelNormalPlane<PointXYZ, pcl::Normal>::Ptr */ 
  /*     model_p(new SampleConsensusModelNormalPlane<PointXYZ, pcl::Normal>(cloud)); */
  /* model_p->setInputNormals(normals); */
  /* model_p->setNormalDistanceWeight(0.0); */
  SampleConsensusModelPlane<PointXYZ>::Ptr 
      model_p(new SampleConsensusModelPlane<PointXYZ>(input_cloud_ptr));

  RandomSampleConsensus<PointXYZ> ransac(model_p);
  ransac.setDistanceThreshold(100.0);

  if (!(running = ransac.computeModel())) {
    clog << "Running: " << running << endl;
    clog << "Could not find any planes!" << endl;
  }

  ransac.getInliers(inliers);
  clog << "Number of inliers found: " << inliers.size() << endl;

  // copies all inliers of the model computed to another PointCloud
  /* auto start = chrono::steady_clock::now(); */
  /* copyPointCloud<PointXYZ>(input_cloud_ptr, inliers, output_cloud_ptr); */
  /* auto end = chrono::steady_clock::now(); */
  /* cout << "Time to copy inliers: " */ 
  /*     << chrono::duration_cast<chrono::milliseconds>(end - start).count() */ 
  /*     << endl; */
}

// Finds a single plane in each subcloud and stores the result in the plane
// summaries vector.
void FindPlanesInSubclouds(vector<PlaneSummaryT> &plane_summs,
                           vector<CloudT::Ptr> &subclouds) {
  for (size_t i = 1; i < subclouds.size(); i++) {
    CloudT::Ptr cloud_ptr = subclouds.at(i);
    
    // Skip clouds with less than three points
    clog << "Number of points in subcloud: " << cloud_ptr->size() << endl;
    if (cloud_ptr->size() < 3)
      continue;

    plane_summs.push_back(PlaneSummaryT());
    plane_summs.back().subcloud_ptr = cloud_ptr; // makes a copy of the cloud ptr
    plane_summs.back().coeffs = Vector4f{};
    plane_summs.back().points_size = cloud_ptr->size();

    SampleConsensusModelPlane<PointXYZ>::Ptr 
      model_ptr(new SampleConsensusModelPlane<PointXYZ>(cloud_ptr));

    RandomSampleConsensus<PointXYZ> ransac(model_ptr);
    ransac.setDistanceThreshold(100.0);
    ransac.computeModel();
    ransac.getInliers(plane_summs.back().inliers);
    ransac.getModelCoefficients(plane_summs.back().coeffs);
  }
}

// Cost function for three normal vectors estimating a cube which is primarily
// determined by their orthogonality with each other.
double CubeCost(Vector3f &n1, Vector3f &n2, Vector3f &n3) {
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
Tuple3 FindOrthoPlaneTriplet(vector<PlaneSummaryT> &plane_summs) {
  vector<Tuple3> triplets = GenerateTriplets(plane_summs.size() - 1);
  clog << "Number of planes: " << plane_summs.size() << endl;
  clog << "Number of triplets: " << triplets.size() << endl;
  Tuple3 *best_triplet;
  double best_cost = numeric_limits<double>::min();
  double best_unnormalized_cost = numeric_limits<double>::min();
  for (Tuple3 &triplet : triplets) {
    PlaneSummaryT &summ1 = plane_summs.at(triplet.at(0));
    PlaneSummaryT &summ2 = plane_summs.at(triplet.at(1));
    PlaneSummaryT &summ3 = plane_summs.at(triplet.at(2));
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
vector<ModelCoefficients> &ExtractModelCoefficients(
    vector<ModelCoefficients> &coeffs, vector<PlaneSummaryT> &plane_summs) {
  for (PlaneSummaryT &plane_summ : plane_summs) {
    VectorXf &vector_coeffs = plane_summ.coeffs;
    assert(vector_coeffs.size() == 4);

    ModelCoefficients coeffs_obj;
    for (int i = 0; i < vector_coeffs.size(); i++)
      coeffs_obj.values.push_back(vector_coeffs(i));

    coeffs.push_back(coeffs_obj);
  }
  return coeffs;
}

// Displays the cube specified by the planes indexed the by the triplet.
// This will launch a viewer containing the inliers of the plane.
void DisplayCube(Tuple3 triplet, vector<PlaneSummaryT> &plane_summs) {
  vector<CloudT::Ptr> cube_subclouds;
  vector<PlaneSummaryT> cube_plane_summs;

  // Copy subclouds and plane summaries used in the cube
  for (size_t i : triplet) {
    cube_subclouds.push_back(plane_summs.at(i).subcloud_ptr);
    cube_plane_summs.push_back(plane_summs.at(i));
  }

  vector<ModelCoefficients> coeffs;

  ViewerPtr viewer_ptr = InitViewer();
  /* AddClouds(viewer_ptr, cube_subclouds); */
  AddPlanes(viewer_ptr, ExtractModelCoefficients(coeffs, cube_plane_summs));
  ViewerTask(viewer_ptr);
}

// Estimates cloud normals from the specified cloud pointer and stores the
// results in the normals cloud.
void EstimateCloudNormals(NormalCloudT &normals,
                      CloudT::Ptr cloud_ptr) {
  IntegralImageNormalEstimation<PointT, Normal> ne;
  ne.setNormalEstimationMethod(ne.AVERAGE_DEPTH_CHANGE);
  ne.setMaxDepthChangeFactor(0.3f);
  ne.setNormalSmoothingSize(4.0f);
  ne.setInputCloud(cloud_ptr);
  ne.compute(normals); 
}

// Partitions the cloud pointed to by the cloud pointer into subclouds based on
// the similiarty of their normal vectors.
void PartitionSubcloudsByNormals(vector<CloudT::Ptr> &output,
                                 CloudT::Ptr cloud_ptr,
                                 double threshold) {
  NormalCloudT normals;

  clog << endl << "Finding cloud normals" << endl;
  auto start = chrono::steady_clock::now();
  EstimateCloudNormals(normals, cloud_ptr);
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
  clog << "Building subclouds" << endl;
  while (!indices.empty()) {
    clog << indices.size() << " points remaining" << endl;
    // Obtain representative point and its normal
    while (!isfinite(normals.at(indices.back()).normal_x)) {
      indices.pop_back();
    }
    int rep_index = indices.back();
    indices.pop_back();
    PointXYZ &rep = cloud_ptr->at(rep_index);
    Normal &rep_normal = normals[rep_index];

    // Add representative point to its own subcloud
    output.push_back(CloudT::Ptr{new CloudT});
    output.back()->push_back(rep);
    assert(output.back()->size() == 1);

    // Add points to the subcloud if they have similar normals to the
    // representative point
    for (size_t i : indices) {
      if (!isfinite(normals.at(i).normal_x))
        continue;

      PointXYZ &point = cloud_ptr->at(i);
      Normal &point_normal = normals[i];
      double angle = NormalSimilarity(rep_normal, point_normal);
      //clog << angle << endl;
      if (angle < threshold) {
        output.back()->push_back(point);
      } else {
        new_indices.push_back(i);
      }
    }

    clog << "Number of points in subcloud " << output.size() << ": " 
        << output.back()->size() << endl;
    assert(new_indices.size() <= indices.size());

    indices.swap(new_indices);
    new_indices.clear();
  }

  clog << "Number of subclouds found: " << output.size() << endl;
  assert(output.size() > 1);
}

int main(int argc, char** argv){
  srand(time(NULL));

  // Initialize point clouds
  clog << "Initializing point cloud with 640 * 480 points" << endl;
  CloudT::Ptr input_cloud{new CloudT}, final_output{new CloudT};
  vector<CloudT::Ptr> subclouds;
  vector<PlaneSummaryT> plane_summs;
  LoadCubePC(*input_cloud);

  // Generate the subclouds
  int denom = 8;
  PartitionSubcloudsByNormals(subclouds, input_cloud, M_PI / denom);
  subclouds.erase(subclouds.begin());

  // View the initial subclouds partitioned by their normal
  clog << "Visualizing" << endl;
  ViewerPtr main_viewer_ptr = InitViewer();
  AddClouds(main_viewer_ptr, subclouds);
  ViewerTask(main_viewer_ptr);
  clog << endl << "Waiting for main_viewer_ptr to close" << endl;

  // Find cube given the subclouds
  if (console::find_argument(argc, argv, "-f") >= 0) {
    FindPlanesInSubclouds(plane_summs, subclouds);
    DisplayCube(FindOrthoPlaneTriplet(plane_summs), plane_summs);
  }

  return 0;
}
