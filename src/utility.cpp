#include "utility.hpp"

#include <pcl/common/common.h>

#include <cassert>

using namespace Eigen;
using namespace pcl;
using namespace std;

vector<Tuple3> GenerateUptoTriplets(int n) {
  if (n < 0) {
    vector<Tuple3> triplets;
    triplets.push_back({});
    return triplets;
  }
  vector<Tuple3> subtriplets = GenerateUptoTriplets(n - 1);
  vector<Tuple3> triplets{subtriplets};
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

double NormalSimilarity(Normal n1, Normal n2) {
  Vector4f v1 = n1.getNormalVector4fMap(), v2 = n2.getNormalVector4fMap();
  double angle = getAngle3D(v1, v2);
  double axis_angle = min(angle, abs(angle - M_PI));
  assert(axis_angle >= 0 && axis_angle <= M_PI);
  return axis_angle;
}
