#ifndef GIOCC_SHADER_PROGRAM_HPP
#define GIOCC_SHADER_PROGRAM_HPP

#include <GL/glew.h>
#include <GL/gl.h>

#include <fstream>
#include <iostream>

namespace giocc
{

// Encapsulates shader resources for constructing shader programs. The shader
// types are defined by the graphics engine. The graphics engine is swappable
// through the Engine template.
//
// Templates:
//    Engine : the graphics engine
template <class Engine>
class Shader
{
public:
  using ShaderType = typename Engine::ShaderType;
  using ShaderId = typename Engine::ShaderId;

  Shader(ShaderType, std::string);
  ~Shader();

  operator ShaderId() const;

private:
  ShaderId shader_;
};

template <class Engine>
Shader<Engine>::Shader(ShaderType shader_type, std::string shader_src)
{
  shader_ = Engine::ShaderUtils::create(shader_type);
  Engine::ShaderUtils::compile(shader_, std::move(shader_src));
  Engine::ShaderUtils::check(shader_);
}

template <class Engine>
Shader<Engine>::~Shader() { Engine::ShaderUtils::destroy(shader_); }

template <class Engine>
Shader<Engine>::operator ShaderId() const { return shader_; }

// Encapsulates shader program resources for building rendering pipelines. This
// class can only instantiate simple vertex and fragment shader pipelines. The
// graphics engine is swappable through the Engine template.
//
// Templates:
//    Engine : the graphics engine
template <class Engine>
class ShaderProgram
{
public:
  using AttribLoc = typename Engine::AttribLoc;
  using UniformLoc = typename Engine::UniformLoc;
  using ProgramId = typename Engine::ProgramId;

  ShaderProgram(const Shader<Engine>&, const Shader<Engine>&);
  ~ShaderProgram();

  // Returns the attribute location of the specified attribute name bound to
  // the shader program during linkage with its shaders.
  AttribLoc attrib_location(const std::string&) const;
  UniformLoc uniform_location(const std::string&) const;

  // Initializes the shader program for usage.
  void operator()();
private:
  ProgramId program_;
};

template <class Engine>
ShaderProgram<Engine>::ShaderProgram(const Shader<Engine>& vert_shader,
    const Shader<Engine>& frag_shader)
{
  program_ = Engine::ProgramUtils::create();
  Engine::ProgramUtils::link(program_, vert_shader, frag_shader);
  Engine::ProgramUtils::check(program_);
}

template <class Engine>
ShaderProgram<Engine>::~ShaderProgram()
{
  Engine::ProgramUtils::destroy(program_);
}

template <class Engine>
typename ShaderProgram<Engine>::AttribLoc
ShaderProgram<Engine>::attrib_location(const std::string& attrib_name) const
{
  return Engine::ProgramUtils::attrib_location(program_, attrib_name.data());
}

template <class Engine>
typename ShaderProgram<Engine>::UniformLoc
ShaderProgram<Engine>::uniform_location(const std::string& uniform_name) const
{
  return Engine::ProgramUtils::uniform_location(program_, uniform_name.data());
}

template <class Engine>
void
ShaderProgram<Engine>::operator()()
{
  Engine::ProgramUtils::use(program_);
}

}

#endif
