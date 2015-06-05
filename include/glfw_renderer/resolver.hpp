#ifndef GIOCC_RESOLVER_HPP
#define GIOCC_RESOLVER_HPP

#include "shader_program.hpp"

namespace giocc
{

// Resolves attribute locations for a specified shader program.
//
// Templates:
//    Engine  : the graphics engine.
template <class Engine>
class AttributeResolver
{
public:
  using AttribLoc = typename Engine::AttribLoc;

public:
  AttributeResolver(const ShaderProgram<Engine>&);

  AttribLoc resolve(const std::string& attrib_name) const;
private:
  const ShaderProgram<Engine>& program_;
};

template <class Engine>
AttributeResolver<Engine>::AttributeResolver(
    const ShaderProgram<Engine>& program)
    : program_{program}
{}

template <class Engine>
typename AttributeResolver<Engine>::AttribLoc
AttributeResolver<Engine>::resolve(const std::string& attrib_name) const
{
  AttribLoc loc = program_.attrib_location(attrib_name);
  return loc;
}

// Resolves uniform locations for a specified shader program.
//
// Templates:
//    Engine  : the graphics engine.
template <class Engine>
class UniformResolver
{
public:
  using UniformLoc = typename Engine::UniformLoc;

public:
  UniformResolver(const ShaderProgram<Engine>&);

  UniformLoc resolve(const std::string& uniform_name) const;
private:
  const ShaderProgram<Engine>& program_;
};

template <class Engine>
UniformResolver<Engine>::UniformResolver(
    const ShaderProgram<Engine>& program)
    : program_{program}
{}

template <class Engine>
typename UniformResolver<Engine>::UniformLoc
UniformResolver<Engine>::resolve(const std::string& uniform_name) const
{
  UniformLoc loc = program_.uniform_location(uniform_name);
  return loc;
}


}

#endif
