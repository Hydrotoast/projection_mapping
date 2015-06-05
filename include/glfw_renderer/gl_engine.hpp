#ifndef GIOCC_OPENGL_ENGINE_HPP
#define GIOCC_OPENGL_ENGINE_HPP

#include <GL/glew.h>
#include <GL/gl.h>

#include <iostream>

namespace giocc
{

// Wraps the OpenGL implementation details and procedures as a set of
// utility functions for an object oriented model. The components include
// shaders, shader programs, vertex buffers and vertex arrays.
//
// Special procedures include life cycle procedures. Components with life cycle
// procedures must be created and destroyed. Hence, these components must
// implement `create` and `destroy` functions.
struct GLEngine
{
public:
  // Shader types
  using ShaderId    = GLuint;
  using ShaderType  = GLenum;
  using ProgramId   = GLuint;

  // Attribute types
  using AttribLoc   = GLuint;
  using AttribName  = const char*;
  
  // Attribute types
  using UniformLoc   = GLuint;
  using UniformName  = const char*;

  // Object types
  using ObjectId  = GLuint;
  using Float     = GLfloat;

  // Wraps the shader procedures which includes life cycle procedures. In
  // addition to the life cycle procedures, shaders can be compiled and their
  // compilation status can be checked for errors.
  struct ShaderUtils
  {
    static ShaderId create(GLenum shader_type);
    static void destroy(ShaderId shader);
    static void compile(ShaderId shader, std::string shader_src);
    static void check(ShaderId shader);
  };

  // Wraps the program procedures which include life cycle procedures.
  // Additionally, programs can be linked to shaders, used for a given pipeline,
  // and attribute locations can be retrieved from them.
  //
  // Interacts with Shaders.
  struct ProgramUtils
  {
    static ProgramId create();
    static void destroy(ProgramId);
    static void link(ProgramId, ShaderId, ShaderId);
    static void check(ProgramId);
    static void use(ProgramId);
    static AttribLoc attrib_location(ProgramId, AttribName);
    static UniformLoc uniform_location(ProgramId, UniformName);
  };


  // Wraps the vertex buffer procedures which includes life cycle
  // procedures. Additionally, vertex buffers can buffer an array of vertex
  // attribute information.
  struct FloatBufferUtils
  {
    static ObjectId create();
    static void destroy(ObjectId);
    static void buffer(ObjectId, const Float*, unsigned int N);
  };

  // Wraps the vertex buffer array procedures which includes life cycle
  // procedures. Additionally, vertex arrays can attach vertex buffers to
  // attribute locations.
  // 
  // Interacts with vertex buffers.
  struct BufferArrayUtils
  {
    static ObjectId create();
    static void destroy(ObjectId);
    static void attach_buffer(ObjectId, ObjectId, AttribLoc);
  };

  // Wraps the drawing procedures.
  // 
  // Interacts with buffer arrays.
  struct RenderUtils
  {
    static void bind_matrix(AttribLoc, const Float*);
    static void render_vertices(ObjectId, unsigned int size);
  };
};

GLEngine::ShaderId
GLEngine::ShaderUtils::create(GLEngine::ShaderType shader_type)
{
  GLEngine::ShaderId shader = glCreateShader(shader_type);
  std::clog << "Shader created: " << shader << std::endl;
  return shader;
}

void
GLEngine::ShaderUtils::destroy(GLEngine::ShaderType shader)
{
  glDeleteShader(shader);
}

void
GLEngine::ShaderUtils::compile(GLEngine::ShaderId shader, std::string shader_src)
{
  const char* shader_str = shader_src.data();
  glShaderSource(shader, 1, &shader_str, NULL);
  glCompileShader(shader);
}

void
GLEngine::ShaderUtils::check(GLEngine::ShaderId shader)
{
  GLint success = GL_FALSE;
  int log_length;
  glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
  glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &log_length);
  if (!success)
  {
    std::string shader_error(log_length, ' ');
    glGetShaderInfoLog(shader, log_length, NULL, &shader_error[0]);
    std::cerr << &shader_error[0] << std::endl;
  }
}

GLEngine::ProgramId
GLEngine::ProgramUtils::create()
{
  GLEngine::ProgramId program = glCreateProgram();
  std::clog << "Program created: " << program << std::endl;
  return program;
}

void
GLEngine::ProgramUtils::destroy(ProgramId program)
{
  glDeleteProgram(program);
}

void
GLEngine::ProgramUtils::link(ProgramId program, GLEngine::ShaderId vert_shader,
    GLEngine::ShaderId frag_shader)
{
    glAttachShader(program, vert_shader);
    glAttachShader(program, frag_shader);
    glLinkProgram(program);
    glDetachShader(program, vert_shader);
    glDetachShader(program, frag_shader);
}

void
GLEngine::ProgramUtils::check(GLEngine::ProgramId program)
{
  GLint success = GL_FALSE;
  int log_length;
  glGetProgramiv(program, GL_LINK_STATUS, &success);
  glGetProgramiv(program, GL_INFO_LOG_LENGTH, &log_length);
  if (!success)
  {
    std::string program_error(log_length, ' ');
    glGetProgramInfoLog(program, log_length, NULL, &program_error[0]);
    std::cerr << &program_error[0] << std::endl;
    glDeleteProgram(program);
  }
}

void
GLEngine::ProgramUtils::use(GLEngine::ProgramId program)
{
  glUseProgram(program);
}

GLEngine::AttribLoc
GLEngine::ProgramUtils::attrib_location(GLEngine::ProgramId program,
    AttribName attrib_name)
{
  GLEngine::AttribLoc loc = glGetAttribLocation(program, attrib_name);
  std::clog << "Attribute " << attrib_name << " located at "
      << loc << std::endl;
  return loc;
}

GLEngine::UniformLoc
GLEngine::ProgramUtils::uniform_location(GLEngine::ProgramId program,
    UniformName location_name)
{
  GLEngine::UniformLoc loc = glGetUniformLocation(program, location_name);
  std::clog << "Uniform " << location_name << " located at "
      << loc << std::endl;
  return loc;
}

GLEngine::ObjectId
GLEngine::FloatBufferUtils::create()
{
  GLEngine::ObjectId vbo;
  glGenBuffers(1, &vbo);
  std::clog << "Buffer object created: " << vbo << std::endl;
  return vbo;
}

void
GLEngine::FloatBufferUtils::destroy(GLEngine::ObjectId vbo)
{
  glDeleteBuffers(1, &vbo);
}

void
GLEngine::FloatBufferUtils::buffer(GLEngine::ObjectId vbo, const Float* data,
    unsigned int N)
{
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glBufferData(GL_ARRAY_BUFFER, N * sizeof(Float), data, GL_STATIC_DRAW);
}

GLEngine::ObjectId
GLEngine::BufferArrayUtils::create()
{
  GLEngine::ObjectId vao;
  glGenVertexArrays(1, &vao);
  std::clog << "Buffer array created: " << vao << std::endl;
  return vao;
}

void
GLEngine::BufferArrayUtils::destroy(GLEngine::ObjectId vao)
{
  glDeleteVertexArrays(1, &vao);
}

void
GLEngine::BufferArrayUtils::attach_buffer(GLEngine::ObjectId vao,
    AttribLoc attrib_loc,
    GLEngine::ObjectId vbo)
{
  glBindVertexArray(vao);
  glEnableVertexAttribArray(attrib_loc);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glVertexAttribPointer(attrib_loc, 3, GL_FLOAT, GL_FALSE, 0, NULL);
  std::clog << "Attaching buffer " << vbo << " to " << vao << std::endl;
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindVertexArray(0);
}

void
GLEngine::RenderUtils::bind_matrix(AttribLoc loc, const Float* data)
{
  glUniformMatrix4fv(loc, 1, 0, data);
}

void
GLEngine::RenderUtils::render_vertices(ObjectId vao, unsigned int size)
{
  glBindVertexArray(vao);
  glDrawArrays(GL_TRIANGLES, 0, size);
  glBindVertexArray(0);
}

}

#endif
