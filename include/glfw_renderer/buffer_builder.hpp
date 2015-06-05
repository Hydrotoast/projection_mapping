#ifndef GIOCC_BUFFER_BUILDER_HPP
#define GIOCC_BUFFER_BUILDER_HPP

#include "float_buffer.hpp"
#include "geometry_buffer.hpp"
#include "resolver.hpp"

#include <memory>

namespace giocc
{

// Creates geometry buffer object by attaching `FloatBuffer` objects to named
// attributes at runtime. The named attributes are retrieved from a compiled
// `ShaderProgram`.
//
// Templates:
//    Engine  : the graphics engine.
//    N       : the number of vertices in the geometry buffer.
template <class Engine, unsigned int N>
class GeometryBufferBuilder
{
public:
  using AttribLoc  = typename Engine::AttribLoc;

public:
  GeometryBufferBuilder(const AttributeResolver<Engine>&);

  // Attaches the `FloatBuffer` to the `GeometryBuffer` being built. The
  // buffer is bound the attribute specified its name.
  template <unsigned int Dimension>
  void attach_buffer(const FloatBuffer<Engine, N, Dimension>& buffer, 
      const std::string& name);

  // Returns an immutable `GeometryBuffer` built after attaching various
  // `FloatBuffer` objects.
  operator std::unique_ptr<GeometryBuffer<Engine, N>>();

private:
  AttributeResolver<Engine> resolver_;

  std::unique_ptr<GeometryBuffer<Engine, N>> buffer_ptr_;
};

template <class Engine, unsigned int N>
GeometryBufferBuilder<Engine, N>::GeometryBufferBuilder(
    const AttributeResolver<Engine>& resolver)
    : resolver_{resolver}
    , buffer_ptr_{new GeometryBuffer<Engine, N>{}}
{}

template <class Engine, unsigned int N>
template <unsigned int Dimension>
void
GeometryBufferBuilder<Engine, N>::attach_buffer(
    const FloatBuffer<Engine, N, Dimension>& buffer,
    const std::string& name)
{
  buffer_ptr_->attach_buffer(buffer, resolver_.resolve(name));
}

template <class Engine, unsigned int N>
GeometryBufferBuilder<Engine, N>::operator std::unique_ptr<GeometryBuffer<Engine, N>>()
{
  return std::move(buffer_ptr_);
}

}

#endif
