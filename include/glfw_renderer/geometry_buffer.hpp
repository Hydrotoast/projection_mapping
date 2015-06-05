#ifndef GIOCC_GEOMETRY_BUFFER_HPP
#define GIOCC_GEOMETRY_BUFFER_HPP

#include "float_buffer.hpp"

#include <GL/glew.h>
#include <GL/gl.h>

#include <iostream>
#include <algorithm>

namespace giocc
{

// Encapsulates the geometry of a drawable object. The geometry of a drawable
// object consists of a set of vertices with a specified dimension. Each vertex
// may have multiple attributes such as position and color. We bind
// `FloatBuffer` objects to the attribute locations of the specified
// attributes.
//
// Templates:
//    Engine    : the graphics engine.
//    N         : the number of vertices in each FloatBuffer.
template <class Engine, unsigned int N>
class GeometryBuffer 
{
public:
  using ObjectId  = typename Engine::ObjectId;
  using AttribLoc = typename Engine::AttribLoc;

public:
  GeometryBuffer();
  ~GeometryBuffer();

  // Cannoy be copied
  GeometryBuffer(const GeometryBuffer&) = delete;
  GeometryBuffer& operator=(const GeometryBuffer&) = delete;

  // Can be moved
  GeometryBuffer(GeometryBuffer&&) = default;
  GeometryBuffer& operator=(GeometryBuffer&&) = default;

  template <unsigned int Dimension>
  void attach_buffer(const FloatBuffer<Engine, N, Dimension>& buffer, 
      AttribLoc loc);

  operator ObjectId() const;
  unsigned int size() const;

private:
  ObjectId vao_;
};

template <class Engine, unsigned int N>
GeometryBuffer<Engine, N>::GeometryBuffer()
    : vao_{Engine::BufferArrayUtils::create()}
{}

template <class Engine, unsigned int N>
GeometryBuffer<Engine, N>::~GeometryBuffer()
{
  std::clog << "Buffer array " << vao_ << " destroyed" << std::endl;
  Engine::BufferArrayUtils::destroy(vao_);
}

template <class Engine, unsigned int N>
template <unsigned int Dimension>
void
GeometryBuffer<Engine, N>::attach_buffer(
    const FloatBuffer<Engine, N, Dimension>& buffer, AttribLoc loc)
{
  static_assert(N % Dimension == 0, "Number of elements must be divisable by the dimension");
  Engine::BufferArrayUtils::attach_buffer(vao_, loc, buffer);
}

template <class Engine, unsigned int N>
GeometryBuffer<Engine, N>::operator ObjectId() const { return vao_; }

template <class Engine, unsigned int N>
unsigned int
GeometryBuffer<Engine, N>::size() const { return N; }

}

#endif
