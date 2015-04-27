#ifndef COMBINATORICS_HPP
#define COMBINATORICS_HPP

#include <vector>

#include <cstdlib>

#include <pcl/point_types.h>

typedef std::vector<size_t> Tuple3;

// Generates upto triplets of values between [1, n].
std::vector<Tuple3> GenerateUptoTriplets(int n);

// Generates triplets of values between [1, n].
std::vector<Tuple3> GenerateTriplets(int n);

// Returns an angle between [0, PI / 2] which describes the similarity
// between the two normals i.e. how close they are to each other.
double NormalSimilarity(pcl::Normal n1, pcl::Normal n2);

#endif
