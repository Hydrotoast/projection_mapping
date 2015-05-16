#include "drawable.hpp"
#include "renderer.hpp"
#include "ransac_cube.hpp"
#include "utility.hpp"

#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/visualization/pcl_visualizer.h>

#define _USE_MATH_DEFINES
#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <iterator>
#include <thread>
#include <mutex>
#include <memory>

#define AXIS_SCALE 160.0
#define POINT_SIZE 3

using namespace giocc;

using namespace pcl;
using namespace Eigen;
using namespace std;

using Viewer = pcl::visualization::PCLVisualizer;
using ViewerPtr = std::shared_ptr<Viewer>;

Renderer<Cube> renderer{640, 480};

static bool running;

// Returns a shared pointer to a viewer object initialized with the clouds
// parameter. The viewer is initialized with a black background and standard
// (X, Y, Z) coordinate axes.
ViewerPtr
InitViewer()
{
  ViewerPtr viewer_ptr{new Viewer{"3D Viewer"}};
  viewer_ptr->setBackgroundColor(0, 0, 0);  // black

  viewer_ptr->addCoordinateSystem(AXIS_SCALE, "global");
  viewer_ptr->initCameraParameters();
  return viewer_ptr;
}

// Add an individual cloud to the shared viewer `viewer_ptr`. 
void
AddCloud(ViewerPtr viewer_ptr, Cloud::Ptr cloud_ptr)
{
  clog << "Adding cloud to the viewer" << endl;
  string cloud_name{"cloud_main"};
  visualization::PointCloudColorHandlerCustom<Point> color(
      cloud_ptr, 0, 0, 255);
  viewer_ptr->addPointCloud<Point>(cloud_ptr, color, cloud_name);
  viewer_ptr->setPointCloudRenderingProperties(
      visualization::PCL_VISUALIZER_POINT_SIZE, POINT_SIZE, cloud_name);
}

// Adds each cloud in the `clouds` vector to the shared viewer specified by
// `viewer_ptr`. Each cloud in the vector is colored with shade of red
// and green.
void
AddClouds(ViewerPtr viewer_ptr, Cloud::Ptr cloud, vector<Indices>& clouds)
{
  clog << "Adding " << clouds.size() << " clouds to the viewer" << endl;
  int stride = 255 / clouds.size();
  for (size_t i = 0; i < clouds.size(); i++) {
    string cloud_name{"cloud"};
    cloud_name += i;
    Cloud::Ptr cloud_ptr{new Cloud{*cloud, clouds.at(i)}};
    visualization::PointCloudColorHandlerCustom<Point> color(
        cloud_ptr, stride * i, 255 - (stride * i), 0);
    viewer_ptr->addPointCloud<Point>(cloud_ptr, color, cloud_name);
    viewer_ptr->setPointCloudRenderingProperties(
        visualization::PCL_VISUALIZER_POINT_SIZE, POINT_SIZE, cloud_name);

  }
}

// Adds a plane for each of the model coefficients given by the `coeffs` vector
// to the shared viewer specified by `viewer_ptr`.
void
AddPlanes(ViewerPtr viewer_ptr, vector<ModelCoefficients>&& coeffs)
{
  clog << "Adding " << coeffs.size() << " planes to the viewer" << endl;
  for (size_t i = 0; i < coeffs.size(); i++) {
    string name{"plane"};
    name += i;
    viewer_ptr->addPlane(coeffs.at(i), name);
  }
}

void
AddCube(ViewerPtr viewer_ptr)
{
  Matrix3f normal_matrix;
  normal_matrix <<
      0.6993,   0.5034,   0.4954,
      -0.0112,  -0.7003,  0.7182,
      0.7147,   -0.5062,  -0.4887;
  Vector3f translation{300.17, 238.04, 136.22};
  Quaternionf rotation{normal_matrix};
  double width = 100.0, height = 100.0, depth = 100.0;
  viewer_ptr->addCube(translation, rotation, width, height, depth);

  // Display the corner point
  Cloud::Ptr corner_ptr{new Cloud{}};
  corner_ptr->width = 1;
  corner_ptr->height = 1;
  corner_ptr->points.resize(1);
  corner_ptr->points[0].x = 300.17;
  corner_ptr->points[0].y = 238.04;
  corner_ptr->points[0].z = 136.22;
  visualization::PointCloudColorHandlerCustom<Point> color(corner_ptr, 0, 0, 255);
  viewer_ptr->addPointCloud<Point>(corner_ptr, color, "corner");
  viewer_ptr->setPointCloudRenderingProperties(
      visualization::PCL_VISUALIZER_POINT_SIZE, POINT_SIZE, "corner");
}

// Runs the shared viewer specified by the `viewer_ptr`.
void
ViewerTask(ViewerPtr viewer_ptr)
{
  while (!viewer_ptr->wasStopped()) {
    viewer_ptr->spinOnce(100);
    this_thread::sleep_for(std::chrono::milliseconds(500));
  }
}

// Loads the cube point cloud into the cloud reference parameter.
void
LoadCubePC(Cloud& cloud)
{
  if (io::loadPCDFile<PointXYZ>("cube.pcd", cloud) == -1) {
    cerr << "Could not open cube.pcd" << endl;
    exit(-1);
  }
}

void
FindPlanarInliers(vector<int>& inliers, 
                  Cloud::Ptr output_cloud_ptr, 
                  Cloud::Ptr input_cloud_ptr)
{
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

// Displays the cube specified by the planes indexed the by the triplet.
// This will launch a viewer containing the inliers of the plane.
void
DisplayCube(Tuple3 triplet, vector<PlaneSummary>& plane_summs)
{
  vector<Cloud::Ptr> cube_subclouds;
  vector<PlaneSummary> cube_plane_summs;

  // Copy subclouds and plane summaries used in the cube
  for (size_t i : triplet) {
    cube_subclouds.push_back(plane_summs.at(i).subcloud_ptr);
    cube_plane_summs.push_back(plane_summs.at(i));
  }

  CubeParams params = EstimateCubeParams(triplet, plane_summs);

  std::clog << "Rendering cube" << std::endl;
  renderer.camera_position({0, 0, 100});
  renderer.scale({5.397, 5.397, 5.397});
  std::array<GLfloat, 16> rotation;
  for (int col = 0; col < 4; col++)
    for (int row = 0; row < 4; row++)
      rotation.at(col * 4 + row) = params.rotation(row, col);
  renderer.rotation(rotation);
  float x = params.translation(0), y = params.translation(1), z = params.translation(2);
  float za = zcm(z);
  float xa = xcm(x, za), ya = ycm(y, za);
  renderer.translation({xa, ya, za});
  renderer();
}

int main(int argc, char** argv)
{
  srand(time(NULL));

  // Initialize point clouds
  clog << "Initializing point cloud with 640 * 480 points" << endl;
  Cloud::Ptr input_cloud{new Cloud}, final_output{new Cloud};
  LoadCubePC(*input_cloud);

  // Generate the subclouds
  int denom = 8;
  auto start = std::chrono::steady_clock::now();
  vector<Indices> subcloud_indices = PartitionSubcloudsByNormals(input_cloud, M_PI / denom);
  auto end = std::chrono::steady_clock::now();
  cout << "Time to partition subcloud_indices by normals: " 
      << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() 
      << " ms" << endl;

  start = std::chrono::steady_clock::now();
  Regions regions = SegmentRegions(input_cloud);
  end = std::chrono::steady_clock::now();
  cout << "Time to segment planar regions " << regions.size() << " planar regions: " 
      << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() 
      << " ms" << endl;


  // Find cube given the subcloud_indices
  if (console::find_argument(argc, argv, "-f") >= 0) {
    vector<PlaneSummary> plane_summs = FindPlanesInSubclouds(input_cloud, subcloud_indices);
    DisplayCube(FindOrthoPlaneTriplet(plane_summs), plane_summs);
  }
  else
  {
    // View the initial subcloud_indices partitioned by their normal
    clog << "Visualizing" << endl;
    ViewerPtr main_viewer_ptr = InitViewer();
    /* AddCloud(main_viewer_ptr, input_cloud); */
    AddClouds(main_viewer_ptr, input_cloud, subcloud_indices);
    ViewerTask(main_viewer_ptr);
    clog << endl << "Waiting for main_viewer_ptr to close" << endl;
  }

  return 0;
}
