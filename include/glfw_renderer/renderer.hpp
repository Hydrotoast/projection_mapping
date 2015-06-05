#ifndef GIOCC_RENDERER_HPP
#define GIOCC_RENDERER_HPP

#include "geometry_buffer.hpp"
#include "resolver.hpp"
#include "scenegraph.hpp"

#include <GLFW/glfw3.h>
#include <Eigen/Dense>
#include <GL/glew.h>
#include <GL/gl.h>

#include <array>
#include <cassert>
#include <cstdio>
#include <stdexcept>

namespace giocc
{

template <class Engine>
class Renderer
{
public:
  using AttribLoc = typename GLEngine::AttribLoc;

public:
  Renderer(unsigned int width, unsigned int height);
  ~Renderer();

  Renderer(const Renderer&) = delete;
  Renderer& operator=(const Renderer&) = delete;

  unsigned int width() const;
  unsigned int height() const;

  void uniform_resolver(const UniformResolver<Engine>& resolver);

  void visit(const Camera&);

  template <unsigned int N>
  void visit(const Geode<Engine, N>&);

  template <unsigned int N>
  void visit(const Scenegraph<Engine, N>& graph);

  template <unsigned int N>
  void display(const Scenegraph<Engine, N>& graph);

  template <unsigned int N>
  void operator()(const Scenegraph<Engine, N>& graph);

private:
  // Handles controls on the GLFW window
  static void controls(GLFWwindow* window, int key, int scancode, int action, int mods);

  // Initializes the window
  static GLFWwindow* init_window(int width, int height);

  // GLFW Window fields
  unsigned int width_;
  unsigned int height_;
  GLFWwindow* window_;

  AttribLoc projection_loc_;
  AttribLoc view_loc_;
  AttribLoc model_loc_;
};

template <class Engine>
Renderer<Engine>::Renderer(unsigned int width, unsigned int height)
    : width_{width}
    , height_{height}
    , window_{init_window(width, height)}
{
  if (window_ == nullptr)
    throw std::runtime_error{"Failed to initialize GLFW window"};
  glewInit();
}

template <class Engine>
Renderer<Engine>::~Renderer()
{
  glfwDestroyWindow(window_);
  glfwTerminate();
}

template <class Engine>
unsigned int Renderer<Engine>::width() const { return width_; };

template <class Engine>
unsigned int Renderer<Engine>::height() const { return height_; };

template <class Engine>
void
Renderer<Engine>::uniform_resolver(const UniformResolver<Engine>& resolver)
{
  projection_loc_ = resolver.resolve("projection");
  view_loc_ = resolver.resolve("view");
  model_loc_ = resolver.resolve("model");
};

template <class Engine>
void
Renderer<Engine>::visit(const Camera& node)
{
  Engine::RenderUtils::bind_matrix(projection_loc_, node.projection().data());
  Engine::RenderUtils::bind_matrix(view_loc_, node.view().data());
}

template <class Engine>
template <unsigned int N>
void
Renderer<Engine>::visit(const Geode<Engine, N>& node)
{
  Engine::RenderUtils::bind_matrix(model_loc_, node.transformation().data());
  Engine::RenderUtils::render_vertices(node.geometry(), node.geometry().size());
}

template <class Engine>
template <unsigned int N>
void
Renderer<Engine>::visit(const Scenegraph<Engine, N>& scene)
{
  visit(scene.camera());
  visit(scene.geode());
}

template <class Engine>
template <unsigned int N>
void
Renderer<Engine>::display(const Scenegraph<Engine, N>& scene)
{
  while (!glfwWindowShouldClose(window_))
  {
    // Scale to window size
    GLint windowWidth, windowHeight;
    glfwGetWindowSize(window_, &windowWidth, &windowHeight);
    glViewport(0, 0, windowWidth, windowHeight);

    // Draw stuff
    glClearColor(0.0, 0.8, 0.3, 1.0);
    glClearDepth(1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    visit(scene);

    // Update Screen
    glfwSwapBuffers(window_);

    // Check for any input, or window movement
    glfwPollEvents();
  }
}

template <class Engine>
template <unsigned int N>
void
Renderer<Engine>::operator()(const Scenegraph<Engine, N>& scene)
{
  assert(window_ != nullptr);
  display(scene);
}


template <class Engine>
void
Renderer<Engine>::controls(GLFWwindow* window, int key, int scancode, int action, int mods)
{
  if (action == GLFW_PRESS)
    if (key == GLFW_KEY_ESCAPE)
      glfwSetWindowShouldClose(window, GL_TRUE);
}

template <class Engine>
GLFWwindow*
Renderer<Engine>::init_window(int width, int height)
{
  if (!glfwInit())
  {
    fprintf(stderr, "Failed to initialize GLFW\n");
    return NULL;
  }
  glfwWindowHint(GLFW_SAMPLES, 4); // 4x antialiasing

  // Open a window and create its OpenGL context
  GLFWwindow* window = glfwCreateWindow(width, height, "TEST", NULL, NULL);

  if (window == NULL)
  {
    fprintf(stderr, "Failed to open GLFW window.\n");
    glfwTerminate();
    return NULL;
  }

  glfwMakeContextCurrent(window);
  glfwSetKeyCallback(window, Renderer::controls);

  // Get info of GPU and supported OpenGL version
  printf("Renderer: %s\n", glGetString(GL_RENDERER));
  printf("OpenGL version supported %s\n", glGetString(GL_VERSION));

  glEnable(GL_DEPTH_TEST); // Depth Testing
  glDepthFunc(GL_LEQUAL);
  glDisable(GL_CULL_FACE);
  glCullFace(GL_BACK);
  return window;
}

}

#endif
