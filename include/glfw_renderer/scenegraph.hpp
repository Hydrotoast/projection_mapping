#ifndef GIOCC_SCENEGRAPH_HPP
#define GIOCC_SCENEGRAPH_HPP

#include "geometry_buffer.hpp"

#include <Eigen/Dense>
#include <memory>

namespace giocc
{

template <class Derived>
class Visitable
{
public:
  template <class T>
  void accept(const T& visitor)
  {
    visitor.visit(static_cast<Derived&>(*this));
  }
};

// Encapsulates the state of a geometry buffer for rendering meshes in the
// graphics engine.
//
// Templates:
//    Engine  : the graphics engine.
//    N       : the number of vertices in the geometry.
template <class Engine, unsigned int N>
class Geode : public Visitable<Geode<Engine, N>>
{
public:
  Geode();
  Geode(std::unique_ptr<GeometryBuffer<Engine, N>>);

  const Eigen::Matrix4f& transformation() const;
  void transformation(Eigen::Matrix4f);

  const GeometryBuffer<Engine, N>& geometry() const;

private:
  Eigen::Matrix4f transformation_;

  std::unique_ptr<GeometryBuffer<Engine, N>> buffer_;
};

template <class Engine, unsigned int N>
Geode<Engine, N>::Geode() 
    : transformation_{Eigen::Matrix4f::Identity()}
{}

template <class Engine, unsigned int N>
Geode<Engine, N>::Geode(std::unique_ptr<GeometryBuffer<Engine, N>> buffer)
    : transformation_{Eigen::Matrix4f::Identity()}
    , buffer_{std::move(buffer)}
{}

template <class Engine, unsigned int N>
const Eigen::Matrix4f&
Geode<Engine, N>::transformation() const { return transformation_; }

template <class Engine, unsigned int N>
void
Geode<Engine, N>::transformation(Eigen::Matrix4f m)
{
  transformation_ = m;
}

template <class Engine, unsigned int N>
const GeometryBuffer<Engine, N>&
Geode<Engine, N>::geometry() const { return *buffer_; }

// Encapsulates the state of the camera in a scene. The camera includes
// projection and view matrices. The projection matrix contains intrinsic
// camera parameters and the view matrix contains extrinsic camera parameters.
class Camera : public Visitable<Camera>
{
public:
  Camera();
  Camera(Eigen::Matrix4f projection, Eigen::Matrix4f view);

  const Eigen::Matrix4f& projection() const;
  const Eigen::Matrix4f& view() const;

  void projection(Eigen::Matrix4f);
  void view(Eigen::Matrix4f);

private:
  Eigen::Matrix4f projection_;
  Eigen::Matrix4f view_;
};

Camera::Camera()
    : projection_{Eigen::Matrix4f::Identity()}
    , view_{Eigen::Matrix4f::Identity()}
{}

Camera::Camera(Eigen::Matrix4f projection, Eigen::Matrix4f view)
    : projection_{projection}
    , view_{view}
{}

const Eigen::Matrix4f&
Camera::projection() const { return projection_; }

const Eigen::Matrix4f&
Camera::view() const { return view_; }

void
Camera::projection(Eigen::Matrix4f m) { projection_ = m; }

void
Camera::view(Eigen::Matrix4f m) { view_ = m; }

// Encapsulates the scene to be rendered including camera state and geode
// state. The scenegraph owns the camera and geode and is thus responsible for
// destroying them.
template <class Engine, unsigned int N>
class Scenegraph : public Visitable<Scenegraph<Engine, N>>
{
public:
  Scenegraph();

  Camera& camera();
  const Camera& camera() const;

  Geode<Engine, N>& geode();
  const Geode<Engine, N>& geode() const;

  void camera(std::unique_ptr<Camera> camera);

  void geode(std::unique_ptr<Geode<Engine, N>> geode);

private:
  std::unique_ptr<Camera> camera_ptr_;
  std::unique_ptr<Geode<Engine, N>> geode_ptr_;
};

template <class Engine, unsigned int N>
Scenegraph<Engine, N>::Scenegraph()
    : camera_ptr_{nullptr}
    , geode_ptr_{nullptr}
{}

template <class Engine, unsigned int N>
Camera& Scenegraph<Engine, N>::camera() { return *camera_ptr_; }

template <class Engine, unsigned int N>
const Camera& Scenegraph<Engine, N>::camera() const { return *camera_ptr_; }

template <class Engine, unsigned int N>
Geode<Engine, N>& Scenegraph<Engine, N>::geode() { return *geode_ptr_; }

template <class Engine, unsigned int N>
const Geode<Engine, N>& Scenegraph<Engine, N>::geode() const { return *geode_ptr_; }

template <class Engine, unsigned int N>
void Scenegraph<Engine, N>::camera(std::unique_ptr<Camera> camera_ptr)
{
  camera_ptr_ = std::move(camera_ptr);
}

template <class Engine, unsigned int N>
void Scenegraph<Engine, N>::geode(std::unique_ptr<Geode<Engine, N>> geode_ptr)
{
  geode_ptr_ = std::move(geode_ptr);
}

}

#endif
