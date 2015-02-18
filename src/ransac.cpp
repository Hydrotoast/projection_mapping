#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <thread>
#include <memory>
#include <chrono>

using namespace pcl;

using namespace std;

shared_ptr<visualization::PCLVisualizer> simpleVis(PointCloud<PointXYZ>::ConstPtr cloud) {
  shared_ptr<visualization::PCLVisualizer> viewer(new visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem(1.0, "global");
  viewer->initCameraParameters();
  return viewer;
}

void InitClouds(PointCloud<PointXYZ>::Ptr &cloud, PointCloud<PointXYZ>::Ptr &final) {
  cloud->width = 640 * 480;
  cloud->height = 1;
  cloud->is_dense = false;
  cloud->points.resize(cloud->width * cloud->height);

  // Populate with random points
  for (size_t i = 0; i < cloud->points.size(); ++i)
  {
    cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0);
    cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0);
    if( i % 2 == 0)
      cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0);
    else
      cloud->points[i].z = -1 * (cloud->points[i].x + cloud->points[i].y);
  }
}

void FindPlanarInliers(vector<int> &inliers, 
                       PointCloud<PointXYZ>::Ptr &final, 
                       PointCloud<PointXYZ>::Ptr &cloud) {
  // created RandomSampleConsensus object and compute the appropriated model
  SampleConsensusModelPlane<PointXYZ>::Ptr 
      model_p(new SampleConsensusModelPlane<PointXYZ>(cloud));

  RandomSampleConsensus<PointXYZ> ransac(model_p);
  ransac.setDistanceThreshold(.01);
  ransac.computeModel();
  ransac.getInliers(inliers);

  // copies all inliers of the model computed to another PointCloud
  copyPointCloud<PointXYZ>(*cloud, inliers, *final);
}

void FindPlanarInliersBenchmarked(vector<int> &inliers, 
                                  PointCloud<PointXYZ>::Ptr &final, 
                                  PointCloud<PointXYZ>::Ptr &cloud) {
    auto start = chrono::steady_clock::now();
    FindPlanarInliers(inliers, final, cloud);
    auto end = chrono::steady_clock::now();

    cout << "Time for 1000 iterations: " 
        << chrono::duration_cast<chrono::milliseconds>(end - start).count() 
        << endl;
}

int main(int argc, char** argv) {
  srand(time(NULL));

  // initialize PointClouds
  clog << "Initializing point cloud with 640 * 480 points" << endl;
  PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
  PointCloud<PointXYZ>::Ptr final(new PointCloud<PointXYZ>);
  InitClouds(cloud, final);

  clog << "Finding planar inliers" << endl;
  vector<int> inliers;
  if(console::find_argument (argc, argv, "-f") >= 0)
    FindPlanarInliersBenchmarked(inliers, final, cloud);

  clog << "Visualizing" << endl;
  // creates the visualization object and adds either our orignial cloud or all of the inliers
  // depending on the command line arguments specified.
  shared_ptr<visualization::PCLVisualizer> viewer;
  viewer = console::find_argument (argc, argv, "-f") >= 0 ? simpleVis(final) : simpleVis(cloud);

  while (!viewer->wasStopped()) {
    viewer->spinOnce (100);
    this_thread::sleep_for(chrono::milliseconds(500));
  }
  return 0;
}
