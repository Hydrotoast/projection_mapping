#ifndef GIOCC_SHAPE_FACTORY_HPP
#define GIOCC_SHAPE_FACTORY_HPP

#include "shader_program.hpp"
#include "float_buffer.hpp"
#include "geometry_buffer.hpp"
#include "buffer_builder.hpp"

#include <memory>

namespace giocc
{

template <class Engine, class Shape>
class ShapeFactory
{
public:
  ShapeFactory(const ShaderProgram<Engine>&);
  operator std::unique_ptr<GeometryBuffer<Engine, Shape::N>>();

private:
  const ShaderProgram<Engine>& program_;
};

template <class Engine, class Shape>
ShapeFactory<Engine, Shape>::ShapeFactory(const ShaderProgram<Engine>& program)
    : program_{program}
{}

template <class Engine, class Shape>
ShapeFactory<Engine, Shape>::operator std::unique_ptr<GeometryBuffer<Engine, Shape::N>>()
{
  GeometryBufferBuilder<Engine, Shape::N> builder{program_};
  builder.attach_buffer(
      FloatBuffer<GLEngine, Shape::N, Shape::Dimension>{Shape::vertices},
      "v_position");
  builder.attach_buffer(
      FloatBuffer<GLEngine, Shape::N, Shape::Dimension>{Shape::colors}, 
      "v_color");
  return builder;
}

class Triangle
{
public:
  static constexpr unsigned int N = 3;
  static constexpr unsigned int Dimension = 3;

  static std::array<float, N * Dimension> vertices;
  static std::array<float, N * Dimension> colors;
};

std::array<float, Triangle::N * Triangle::Dimension> Triangle::vertices = {
    -0.3, -0.3, 0,
    0,    0.5,  0,
    0.3,  -0.3, 0
};

std::array<float, Triangle::N * Triangle::Dimension> Triangle::colors = {
    0, 0, 1,
    0, 1, 0,
    1, 0, 0
};

class Cube
{
public:
  static constexpr unsigned int N = 36;
  static constexpr unsigned int Dimension = 3;

  static std::array<float, N * Dimension> vertices;
  static std::array<float, N * Dimension> colors;
};

std::array<GLfloat, Cube::N * Cube::Dimension> Cube::vertices =
{
    -1.0f,-1.0f,-1.0f,
    -1.0f,-1.0f, 1.0f,
    -1.0f, 1.0f, 1.0f,
    1.0f, 1.0f,-1.0f,
    -1.0f,-1.0f,-1.0f,
    -1.0f, 1.0f,-1.0f,
    1.0f,-1.0f, 1.0f,
    -1.0f,-1.0f,-1.0f,
    1.0f,-1.0f,-1.0f,
    1.0f, 1.0f,-1.0f,
    1.0f,-1.0f,-1.0f,
    -1.0f,-1.0f,-1.0f,
    -1.0f,-1.0f,-1.0f,
    -1.0f, 1.0f, 1.0f,
    -1.0f, 1.0f,-1.0f,
    1.0f,-1.0f, 1.0f,
    -1.0f,-1.0f, 1.0f,
    -1.0f,-1.0f,-1.0f,
    -1.0f, 1.0f, 1.0f,
    -1.0f,-1.0f, 1.0f,
    1.0f,-1.0f, 1.0f,
    1.0f, 1.0f, 1.0f,
    1.0f,-1.0f,-1.0f,
    1.0f, 1.0f,-1.0f,
    1.0f,-1.0f,-1.0f,
    1.0f, 1.0f, 1.0f,
    1.0f,-1.0f, 1.0f,
    1.0f, 1.0f, 1.0f,
    1.0f, 1.0f,-1.0f,
    -1.0f, 1.0f,-1.0f,
    1.0f, 1.0f, 1.0f,
    -1.0f, 1.0f,-1.0f,
    -1.0f, 1.0f, 1.0f,
    1.0f, 1.0f, 1.0f,
    -1.0f, 1.0f, 1.0f,
    1.0f,-1.0f, 1.0f
};

std::array<GLfloat, Cube::N * Cube::Dimension> Cube::colors =
{
    0.583f,  0.771f,  0.014f,
    0.609f,  0.115f,  0.436f,
    0.327f,  0.483f,  0.844f,
    0.822f,  0.569f,  0.201f,
    0.435f,  0.602f,  0.223f,
    0.310f,  0.747f,  0.185f,
    0.597f,  0.770f,  0.761f,
    0.559f,  0.436f,  0.730f,
    0.359f,  0.583f,  0.152f,
    0.483f,  0.596f,  0.789f,
    0.559f,  0.861f,  0.639f,
    0.195f,  0.548f,  0.859f,
    0.014f,  0.184f,  0.576f,
    0.771f,  0.328f,  0.970f,
    0.406f,  0.615f,  0.116f,
    0.676f,  0.977f,  0.133f,
    0.971f,  0.572f,  0.833f,
    0.140f,  0.616f,  0.489f,
    0.997f,  0.513f,  0.064f,
    0.945f,  0.719f,  0.592f,
    0.543f,  0.021f,  0.978f,
    0.279f,  0.317f,  0.505f,
    0.167f,  0.620f,  0.077f,
    0.347f,  0.857f,  0.137f,
    0.055f,  0.953f,  0.042f,
    0.714f,  0.505f,  0.345f,
    0.783f,  0.290f,  0.734f,
    0.722f,  0.645f,  0.174f,
    0.302f,  0.455f,  0.848f,
    0.225f,  0.587f,  0.040f,
    0.517f,  0.713f,  0.338f,
    0.053f,  0.959f,  0.120f,
    0.393f,  0.621f,  0.362f,
    0.673f,  0.211f,  0.457f,
    0.820f,  0.883f,  0.371f,
    0.982f,  0.099f,  0.879f
};

}

#endif
