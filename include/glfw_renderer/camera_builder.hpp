#ifndef GIOCC_CAMERA_BUILDER_HPP
#define GIOCC_CAMERA_BUILDER_HPP

#include "glfw_renderer/scenegraph.hpp"

#include <Eigen/Dense>

namespace giocc
{

// Builds a camera.
template <class Engine>
class CameraBuilder
{
public:
  CameraBuilder(float near, float far);
  
  void add_persp_matrix(Eigen::Matrix4f);
  void add_persp_matrix();

  void add_ortho_matrix(float width, float height);

  void add_ndc_matrix(Eigen::Matrix4f);
  void add_ndc_matrix(float width, float height);

  void add_view_matrix(Eigen::Matrix4f);

  operator std::unique_ptr<Camera>();
private:
  float near_;
  float far_;

  Eigen::Matrix4f proj_m_;
  Eigen::Matrix4f ndc_m_;
  Eigen::Matrix4f view_m_;
};

template <class Engine>
CameraBuilder<Engine>::CameraBuilder(float near, float far)
    : near_{near}
    , far_{far}
    , proj_m_{Eigen::Matrix4f::Identity()}
    , ndc_m_{Eigen::Matrix4f::Identity()}
    , view_m_{Eigen::Matrix4f::Identity()}
{}

template <class Engine>
void CameraBuilder<Engine>::add_persp_matrix(Eigen::Matrix4f m) { proj_m_ = m; }

template <class Engine>
void
CameraBuilder<Engine>::add_persp_matrix()
{
  float A = near_ + far_;
  float B = near_ * far_;
  proj_m_ <<
      near_,  0,      0,    0,
      0,      near_,  0,    0,
      0,      0,      A,    B,
      0,      0,      -1,   0;
}

template <class Engine>
void CameraBuilder<Engine>::add_ndc_matrix(Eigen::Matrix4f m) { ndc_m_ = m; }

template <class Engine>
void
CameraBuilder<Engine>::add_ortho_matrix(float width, float height)
{
  float tz = -(far_ + near_) / (far_ - near_);
  float d1 = 1.0 / width;
  float d2 = 1.0 / height;
  float d3 = -2.0 / (far_ - near_);
  ndc_m_ <<
      d1, 0,  0,  0,
      0,  d2, 0,  0,
      0,  0,  d3, tz,
      0,  0,  0,  1;
}

template <class Engine>
void
CameraBuilder<Engine>::add_ndc_matrix(float width, float height)
{
  /* float tz = -2.0 * (far_ * near_) / (far_ - near_); */
  /* float d1 = near_ / right; */
  /* float d2 = near_ / left; */
  /* float d3 = -(far_ + near_) / (far_ - near_); */
  /* ndc_m_ << */
  /*     d1, 0,  0,  0, */
  /*     0,  d2, 0,  0, */
  /*     0,  0,  d3, tz, */
  /*     0,  0,  -1, 0; */
  /* float tx = -(right + left) / (right - left); */
  /* float ty = -(top + bottom) / (top - bottom); */
  float tz = -(far_ + near_) / (far_ - near_);
  float d1 = 2.0 / width;
  float d2 = 2.0 / height;
  float d3 = -2.0 / (far_ - near_);
  ndc_m_ <<
      d1, 0,  0,  0,
      0,  d2, 0,  0,
      0,  0,  d3, tz,
      0,  0,  0,  1;
}

template <class Engine>
void CameraBuilder<Engine>::add_view_matrix(Eigen::Matrix4f m) { view_m_ = m; }

template <class Engine>
CameraBuilder<Engine>::operator std::unique_ptr<Camera>()
{
  return std::unique_ptr<Camera>{new Camera{ndc_m_ * proj_m_, view_m_}};
}

}

#endif
