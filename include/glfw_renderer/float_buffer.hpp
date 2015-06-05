#ifndef GIOCC_FLOAT_BUFFER_HPP
#define GIOCC_FLOAT_BUFFER_HPP

#include <array>

namespace giocc
{

// Encapsulates the vertex buffer object with static definitions for the number
// of floats to buffer.
//
// Templates:
//    Engine    : the graphics engine.
//    N         : the number of vertices in the static buffer.
//    Dimension : the dimension of the vertices.
template <class Engine, unsigned int N, unsigned int Dimension = 3>
class FloatBuffer
{
public:
  using Float    = typename Engine::Float;
  using ObjectId = typename Engine::ObjectId;

public:
  FloatBuffer(std::array<Float, N * Dimension> raw_buffer);
  ~FloatBuffer();

  unsigned int size() const;

  operator ObjectId() const;
private:
  ObjectId vbo_;
};

template <class Engine, unsigned int N, unsigned int Dimension>
FloatBuffer<Engine, N, Dimension>::FloatBuffer(
    std::array<Float,  N * Dimension> raw_buffer)
{
  vbo_ = Engine::FloatBufferUtils::create();
  Engine::FloatBufferUtils::buffer(vbo_, raw_buffer.data(), N * Dimension);
}

template <class Engine, unsigned int N, unsigned int Dimension>
FloatBuffer<Engine, N, Dimension>::~FloatBuffer()
{
  Engine::FloatBufferUtils::destroy(vbo_);
}

template <class Engine, unsigned int N, unsigned int Dimension>
unsigned int 
FloatBuffer<Engine, N, Dimension>::size() const { return N * Dimension; }

template <class Engine, unsigned int N, unsigned int Dimension>
FloatBuffer<Engine, N, Dimension>::operator ObjectId() const { return vbo_; }

}

#endif
