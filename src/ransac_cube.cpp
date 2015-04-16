#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_normal_plane.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <set>
#include <iterator>
#include <algorithm>
#include <cassert>

#include <thread>
#include <memory>
#include <chrono>

#define AXIS_SCALE 160.0
#define POINT_SIZE 3

using namespace pcl;
using namespace std;

typedef shared_ptr<visualization::PCLVisualizer> ViewerPtr;

ViewerPtr viewer;

ViewerPtr InitViewer(PointCloud<PointXYZ>::ConstPtr cloud) {
  ViewerPtr viewer(new visualization::PCLVisualizer("3D Viewer"));

  viewer->setBackgroundColor(0, 0, 0);  // black

  visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green_color(cloud, 0, 255, 0); // green
  viewer->addPointCloud<PointXYZ>(cloud, green_color, "cloud");
  viewer->setPointCloudRenderingProperties(
      visualization::PCL_VISUALIZER_POINT_SIZE, POINT_SIZE, "cloud");
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

void LoadCubePC(PointCloud<PointXYZ>::Ptr &cloud) {
  if (pcl::io::loadPCDFile<PointXYZ>("cube.pcd", *cloud) == -1) {
    cerr << "Could not open cube.pcd" << endl;
    exit(-1);
  }
}

bool FindPlanarInliers(vector<int> &inliers, 
                       PointCloud<PointXYZ>::Ptr &final, 
                       PointCloud<PointXYZ>::Ptr &cloud, 
                       PointCloud<pcl::Normal>::Ptr &normals) {
  // created RandomSampleConsensus object and compute the appropriated model
  /* SampleConsensusModelNormalPlane<PointXYZ, pcl::Normal>::Ptr */ 
  /*     model_p(new SampleConsensusModelNormalPlane<PointXYZ, pcl::Normal>(cloud)); */
  /* model_p->setInputNormals(normals); */
  /* model_p->setNormalDistanceWeight(0.0); */
  SampleConsensusModelPlane<PointXYZ>::Ptr 
      model_p(new SampleConsensusModelPlane<PointXYZ>(cloud));

  RandomSampleConsensus<PointXYZ> ransac(model_p);
  ransac.setDistanceThreshold(100.0);
  bool run = false;
  if (!(run = ransac.computeModel()))
    clog << "Could not find any planes!" << endl;
  ransac.getInliers(inliers);
  clog << "Number of inliers found: " << inliers.size() << endl;

  // copies all inliers of the model computed to another PointCloud
  auto start = chrono::steady_clock::now();
  copyPointCloud<PointXYZ>(*cloud, inliers, *final);
  auto end = chrono::steady_clock::now();
  cout << "Time to copy inliers: " 
      << chrono::duration_cast<chrono::milliseconds>(end - start).count() 
      << endl;

  return run;
}

bool FindPlanarInliersBenchmarked(vector<int> &inliers, 
                                  PointCloud<PointXYZ>::Ptr &final, 
                                  PointCloud<PointXYZ>::Ptr &cloud, 
                                  PointCloud<pcl::Normal>::Ptr &normals) {
    auto start = chrono::steady_clock::now();
    bool run = FindPlanarInliers(inliers, final, cloud, normals);
    auto end = chrono::steady_clock::now();

    cout << "Time for 1000 iterations: " 
        << chrono::duration_cast<chrono::milliseconds>(end - start).count() 
        << endl;

    return run;
}

void FindCloudNormals(pcl::PointCloud<PointXYZ>::ConstPtr cloud,
                      pcl::PointCloud<pcl::Normal>::Ptr normals) {
    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setNormalEstimationMethod (ne.AVERAGE_DEPTH_CHANGE);
    ne.setMaxDepthChangeFactor(0.1f);
    ne.setNormalSmoothingSize(10.0f);
    ne.setInputCloud(cloud);
    ne.compute(*normals); 
}

void RemovePlanarInliers(pcl::PointCloud<PointXYZ>::Ptr cloud,
                         pcl::PointCloud<PointXYZ>::Ptr final)
{
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

  bool run = true;
  while (run) {
    clog << endl << "Finding cloud normals" << endl;
    auto start = chrono::steady_clock::now();
    FindCloudNormals(cloud, normals);
    auto end = chrono::steady_clock::now();
    assert(normals->size() == cloud->size());

    cout << "Time for normal estimation: " 
        << chrono::duration_cast<chrono::milliseconds>(end - start).count() 
        << endl;

    // Add updated normals
    UpdateCloudNormals(cloud, normals);

    clog << endl << "Finding planar inliers" << endl;
    vector<int> inliers;
      run = FindPlanarInliersBenchmarked(inliers, final, cloud, normals);

    // Delete from cloud
    cout << "Erasing inliers" << endl;
    start = chrono::steady_clock::now();
    for (int i : inliers)
      cloud->erase(cloud->begin() + i);
    end = chrono::steady_clock::now();
    cout << "Time for erasing inliers: " 
        << chrono::duration_cast<chrono::milliseconds>(end - start).count() 
        << endl;

    // Remove points from the viewer
    UpdateCloudNormals(cloud, normals);
  }
}

int main(int argc, char** argv) {
  srand(time(NULL));

  // initialize PointClouds
  clog << "Initializing point cloud with 640 * 480 points" << endl;
  PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
  PointCloud<PointXYZ>::Ptr final(new PointCloud<PointXYZ>);
  LoadCubePC(cloud);

  clog << "Visualizing" << endl;
  // creates the visualization object and adds either our orignial cloud or all of the inliers
  // depending on the command line arguments specified.
  viewer = InitViewer(cloud);
  std::thread viewer_thread(ViewerTask);

  if(console::find_argument (argc, argv, "-f") >= 0)
    RemovePlanarInliers(cloud, final);

  viewer_thread.join();
  return 0;
}
