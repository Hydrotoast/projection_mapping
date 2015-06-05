#ifndef GIOCC_LOADERS_HPP
#define GIOCC_LOADERS_HPP

#include "shader_program.hpp"

namespace giocc
{

// Functor for loading file contents from a specified file path.
struct FileLoader
{
  std::string load(const std::string& file_path);
};

std::string
FileLoader::load(const std::string& file_path)
{
  std::string content;
  std::ifstream file_stream{file_path, std::ios::in};

  if (!file_stream.is_open())
  {
    std::cerr << "Could not read file " << file_path 
        << ". File doe snot exist." << std::endl;
    return "";
  }

  std::string line = "";
  while (std::getline(file_stream, line))
    content.append(line + "\n");

  file_stream.close();
  return content;
}

// Loads shader objects for a specified graphics engine from shader files. The
// loading mechanism can be changed with the `Loader` template.
//
// Templates:
//    Engine  : the graphics engine.
//    Loader  : the loading mechanism for loading files.
template <class Engine, class Loader>
class ShaderFileLoader
{
private:
  using ShaderType = typename Engine::ShaderType;

public:
  Shader<Engine> load(ShaderType type, const std::string& file_path);

private:
  Loader loader_;
};

template <class Engine, class Loader>
Shader<Engine>
ShaderFileLoader<Engine, Loader>::load(ShaderType type,
    const std::string& file_path)
{
  return Shader<Engine>{type, loader_.load(file_path)};
}

// Loads shader program objects for a specified graphics engine from a set
// of shader files. The loading mechanism can be changed with the `Loader`
// template.
//
// Templates
//    Engine  : the graphics engine.
//    Loader  : the loading mechanism for loading files.
template <class Engine, class Loader>
class ShaderProgramFileLoader
{
public:
  ShaderProgram<Engine> load(const std::string& vert_shader_path, 
      const std::string& frag_shader_path);
private:
  ShaderFileLoader<Engine, Loader> shader_loader_;
};

template <class Engine, class Loader>
ShaderProgram<Engine>
ShaderProgramFileLoader<Engine, Loader>::load(
    const std::string& vert_shader_path, 
    const std::string& frag_shader_path)
{
  Shader<Engine> vert_shader = shader_loader_.load(GL_VERTEX_SHADER,
      vert_shader_path);
  Shader<Engine> frag_shader = shader_loader_.load(GL_FRAGMENT_SHADER,
      frag_shader_path);
  return ShaderProgram<Engine>{vert_shader, frag_shader};
}

}

#endif
