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

typedef shared_ptr<visualization::PCLVisualizer> ViewerPtr;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> CloudT;
typedef pcl::PointCloud<Normal> NormalCloudT;
typedef Eigen::VectorXf PlaneCoeffsT;

typedef struct PlaneSummaryT {
  PlaneCoeffsT coeffs;
  size_t points_size;
} PlaneSummaryT;

ViewerPtr viewer;
bool running;

ViewerPtr InitViewer(vector<CloudT::Ptr>& clouds) {
  ViewerPtr viewer{new visualization::PCLVisualizer("3D Viewer")};

  viewer->setBackgroundColor(0, 0, 0);  // black

  for (size_t i = 0; i < clouds.size(); i++) {
    string cloud_name{"cloud"};
    cloud_name += i;
    int stride = 255 / clouds.size();
    visualization::PointCloudColorHandlerCustom<PointT> color(clouds.at(i), stride * i, 255 - (stride * i), 0);
    viewer->addPointCloud<PointT>(clouds.at(i), color, cloud_name);
    viewer->setPointCloudRenderingProperties(
        visualization::PCL_VISUALIZER_POINT_SIZE, POINT_SIZE, cloud_name);
  }

  viewer->addCoordinateSystem(AXIS_SCALE, "global");
  viewer->initCameraParameters();
  return viewer;
}

void UpdateCloudNormals(
    PointCloud<PointXYZ>::ConstPtr cloud,
    PointCloud<Normal>::ConstPtr normals) {
  clog << "Updating point cloud and normals" << endl;
  visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green_color(cloud, 0, 255, 0); // green
  viewer->updatePointCloud<PointXYZ>(cloud, green_color, "cloud");

  viewer->removePointCloud("normals");
  viewer->addPointCloudNormals<PointXYZ, Normal>(cloud, normals, 40, 40, "normals");
}

void ViewerTask() {
  while (!viewer->wasStopped()) {
    viewer->spinOnce(100);
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

void FindPlanarInliersBenchmarked(vector<int> &inliers, 
                                  CloudT::Ptr output_cloud_ptr, 
                                  CloudT::Ptr input_cloud_ptr) {
    auto start = chrono::steady_clock::now();
    FindPlanarInliers(inliers, output_cloud_ptr, input_cloud_ptr);
    auto end = chrono::steady_clock::now();

    cout << "Time for 1000 iterations: " 
        << chrono::duration_cast<chrono::milliseconds>(end - start).count() 
        << endl;
}

void FindPlanesInSubclouds(vector<CloudT::Ptr> &subclouds,
                           vector<PlaneSummaryT> &plane_summs) {
  for (size_t i = 1; i < subclouds.size(); i++) {
    CloudT::Ptr cloud_ptr = subclouds.at(i);
    
    // Skip clouds with less than three points
    clog << "Number of points in subcloud: " << cloud_ptr->size() << endl;
    if (cloud_ptr->size() < 3)
      continue;

    plane_summs.push_back(PlaneSummaryT());
    plane_summs.back().coeffs = Vector4f{};
    plane_summs.back().points_size = cloud_ptr->size();

    SampleConsensusModelPlane<PointXYZ>::Ptr 
      model_ptr(new SampleConsensusModelPlane<PointXYZ>(cloud_ptr));

    RandomSampleConsensus<PointXYZ> ransac(model_ptr);
    ransac.setDistanceThreshold(100.0);
    ransac.computeModel();
    ransac.getModelCoefficients(plane_summs.back().coeffs);
  }
}

typedef vector<size_t> Tuple3;

vector<Tuple3> GenerateUptoTriplets(int n) {
  if (n < 0) {
    vector<Tuple3> triplets;
    triplets.push_back({});
    return triplets;
  }
  vector<Tuple3> subtriplets = GenerateUptoTriplets(n - 1);
  vector<Tuple3> triplets(subtriplets);
  for (Tuple3 &t : subtriplets) {
    t.push_back(n);
    if (t.size() > 3)
      continue;
    triplets.push_back(t);
  }
  return triplets;
}

vector<Tuple3> GenerateTriplets(int n) {
  vector<Tuple3> upto_triplets = GenerateUptoTriplets(n);
  vector<Tuple3> triplets;
  for (Tuple3 triplet : upto_triplets) {
    if (triplet.size() < 3)
      continue;
    triplets.push_back(triplet);
  }
  return triplets;
}

double CubeCost(Vector3f &n1, Vector3f &n2, Vector3f &n3) {
  Vector3f n12_perp = n1.cross(n2);
  Vector3f n23_perp = n2.cross(n3);
  Vector3f n13_perp = n1.cross(n3);
  double c3 = fabs(n12_perp.dot(n3));
  double c1 = fabs(n23_perp.dot(n1));
  double c2 = fabs(n13_perp.dot(n2));
  return c3 + c1 + c2;
}

void FindCubeFromPlanes(vector<PlaneSummaryT> &plane_summs) {
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
    VectorXf &p1 = summ1.coeffs;
    p1.conservativeResize(3, 1);
    VectorXf &p2 = summ2.coeffs;
    p2.conservativeResize(3, 1);
    VectorXf &p3 = summ3.coeffs;
    p3.conservativeResize(3, 1);
    Vector3f n1{p1}, n2{p2}, n3{p3};
    double unnormalized_cost = CubeCost(n1, n2, n3);
    double cost = unnormalized_cost * (summ1.points_size + summ2.points_size + summ3.points_size);
    if (cost > best_cost) {
      best_cost = cost;
      best_unnormalized_cost = unnormalized_cost;
      best_triplet = &triplet;
    }
    clog << "Triplet: ";
    for (size_t i : triplet)
      clog << i << " ";
    clog << " Cost: " << cost << endl;
  }
  clog << "Best triplet: ";
  for (size_t i : *best_triplet)
    clog << i << " ";
  clog << endl;
  clog << "Unnormalized cost: " << best_unnormalized_cost << endl;
}

void RemovePlanarInliers(CloudT::Ptr input_cloud_ptr,
                         CloudT::Ptr output_cloud_ptr) {
  running = true;
  while (running) {
      // Add updated normals
    /* UpdateCloudNormals(cloud, normals); */

    clog << endl << "Finding planar inliers" << endl;
    vector<int> inliers;
    FindPlanarInliersBenchmarked(inliers, output_cloud_ptr, input_cloud_ptr);

    // Delete from cloud
    /* cout << "Erasing inliers" << endl; */
    /* auto start = chrono::steady_clock::now(); */
    /* for (int i : inliers) */
    /*   cloud->erase(cloud->begin() + i); */
    /* auto end = chrono::steady_clock::now(); */
    /* cout << "Time for erasing inliers: " */ 
    /*     << chrono::duration_cast<chrono::milliseconds>(end - start).count() */ 
    /*     << endl; */

    // Remove points from the viewer
    /* UpdateCloudNormals(cloud, normals); */
  }
}

void FindCloudNormals(CloudT::Ptr cloud_ptr,
                      NormalCloudT &normals) {
  IntegralImageNormalEstimation<PointXYZ, Normal> ne;
  ne.setNormalEstimationMethod (ne.AVERAGE_DEPTH_CHANGE);
  ne.setMaxDepthChangeFactor(0.3f);
  ne.setNormalSmoothingSize(4.0f);
  ne.setInputCloud(cloud_ptr);
  ne.compute(normals); 
}

// Returns an angle between [0, PI / 2] which describes the similarity
// between the two normals i.e. how close they are to each other.
double NormalSimilarity(Normal n1, Normal n2) {
  /* static double prev = 0; */
  Vector4f v1 = n1.getNormalVector4fMap(), v2 = n2.getNormalVector4fMap();
  double angle = getAngle3D(v1, v2);
  double axis_angle = min(angle, abs(angle - M_PI));
  /* if (axis_angle != prev) { */
  /*   clog << axis_angle / M_PI * 180 << endl; */
  /*   prev = axis_angle; */
  /* } */
  assert(axis_angle >= 0 && axis_angle <= M_PI);
  return axis_angle;
}

void GenerateSubcloudsByNormals(CloudT::Ptr cloud_ptr,
                                vector<CloudT::Ptr> &output,
                                double threshold) {
  NormalCloudT normals;

  clog << endl << "Finding cloud normals" << endl;
  auto start = chrono::steady_clock::now();
  FindCloudNormals(cloud_ptr, normals);
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

    clog << "Number of points in subcloud: " << output.back()->size() << endl;
    assert(new_indices.size() <= indices.size());

    indices.swap(new_indices);
    new_indices.clear();
  }

  clog << "Number of subclouds found: " << output.size() << endl;
  assert(output.size() > 1);
}

int main(int argc, char** argv) {
  srand(time(NULL));

  // initialize PointClouds
  clog << "Initializing point cloud with 640 * 480 points" << endl;
  CloudT::Ptr input_cloud{new CloudT}, final_output{new CloudT};
  vector<CloudT::Ptr> subclouds;
  vector<PlaneSummaryT> plane_summs;
  LoadCubePC(*input_cloud);

  int denom = 8;
  GenerateSubcloudsByNormals(input_cloud, subclouds, M_PI / denom);
  subclouds.erase(subclouds.begin());

  clog << "Visualizing" << endl;
  // creates the visualization object and adds either our orignial cloud or all of the inliers
  // depending on the command line arguments specified.
  viewer = InitViewer(subclouds);
  thread viewer_thread(ViewerTask);

  if (console::find_argument (argc, argv, "-f") >= 0) {
    FindPlanesInSubclouds(subclouds, plane_summs);
    FindCubeFromPlanes(plane_summs);
  }

  clog << endl << "Waiting for viewer to close" << endl;
  viewer_thread.join();
  return 0;
}
