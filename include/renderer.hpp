#ifndef CUBE_RENDERER_HPP
#define CUBE_RENDERER_HPP

#include <Eigen/Dense>
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <array>
#include <cassert>
#include <cstdio>
#include <stdexcept>
                          
namespace giocc
{

// Renders a shape in a scene using GLFW to create windows with OpenGL contexts
// and to handle control input. The shape is parameterized by a rotation and
// translation from the world coordinates.
// 
// Templates
//   Shape  the shape to render in the scene
template <typename Shape>
class Renderer
{
public:
  // Initialized a GLFW window with the specified width and height parameters.
  Renderer(int width, int height);
  ~Renderer();

  // Rendering loop to continously redraw the scene. The rendering loop will
  // draw the cube in the scene with the specified camera parameters and object
  // parameters.
  void display();

  // Accessors
  const Eigen::Vector3f& camera_position() const;
  const Eigen::Vector3f& view_direction() const;
  const Eigen::Vector3f& world_position() const;
  const Eigen::Matrix4f& rotation() const;
  const Eigen::Vector3f& translation() const;
  const Eigen::Vector3f& scale() const;

  int width() const;
  int height() const;

  // Modifiers
  void camera_position(Eigen::Vector3f v);
  void view_direction(Eigen::Vector3f v);
  void world_position(Eigen::Vector3f v);
  void rotation(Eigen::Matrix4f m);
  void translation(Eigen::Vector3f v);
  void scale(Eigen::Vector3f v);

  // Runs the system
  void operator()();
private:
  // Handles controls on the GLFW window
  static void controls(GLFWwindow* window, int key, int scancode, int action, int mods);

  // Initializes the window
  static GLFWwindow* init_window(int width, int height);

  // Scene extrinsic paramters
  Eigen::Vector3f camera_position_;
  Eigen::Vector3f view_direction_;
  Eigen::Vector3f world_position_;

  // Object extrinsic parameters
  Eigen::Matrix4f rotation_;
  Eigen::Vector3f translation_;
  Eigen::Vector3f scale_;

  // GLFW Window fields
  int width_;
  int height_;
  GLFWwindow* window_;
};

template <typename Shape>
Renderer<Shape>::Renderer(int width, int height)
    : width_{width}
    , height_{height}
    , window_{init_window(width, height)}
    // Scene extrinsic paramters
    , camera_position_{0, 0, -5}
    , view_direction_{0, 1, 0}
    , world_position_{0, 0, 0}
    // Object extrinsic parameters
    , rotation_{Eigen::Matrix4f::Identity(4,4)}
    , translation_{0, 0, 0}
{
  if (window_ == nullptr)
    throw std::runtime_error{"Failed to initialize GLFW window"};
}

template <typename Shape>
Renderer<Shape>::~Renderer()
{
  glfwDestroyWindow(window_);
  glfwTerminate();
}

// Accessors

template <typename Shape>
const Eigen::Vector3f&
Renderer<Shape>::camera_position() const { return camera_position_; };

template <typename Shape>
const Eigen::Vector3f&
Renderer<Shape>::view_direction() const { return view_direction_; };

template <typename Shape>
const Eigen::Vector3f&
Renderer<Shape>::world_position() const { return world_position_; };

template <typename Shape>
const Eigen::Matrix4f&
Renderer<Shape>::rotation() const { return rotation_; };

template <typename Shape>
const Eigen::Vector3f&
Renderer<Shape>::translation() const { return translation_; };

template <typename Shape>
const Eigen::Vector3f&
Renderer<Shape>::scale() const { return scale_; };

template <typename Shape>
int
Renderer<Shape>::width() const { return width_; };

template <typename Shape>
int
Renderer<Shape>::height() const { return height_; };

// Mutators
template <typename Shape>
void
Renderer<Shape>::camera_position(Eigen::Vector3f v) { camera_position_ = v; };

template <typename Shape>
void
Renderer<Shape>::view_direction(Eigen::Vector3f v) { view_direction_ = v; };

template <typename Shape>
void
Renderer<Shape>::world_position(Eigen::Vector3f v) { world_position_ = v; };

template <typename Shape>
void
Renderer<Shape>::rotation(Eigen::Matrix4f m) { rotation_ = m; };

template <typename Shape>
void
Renderer<Shape>::translation(Eigen::Vector3f v) { translation_ = v; };

template <typename Shape>
void
Renderer<Shape>::scale(Eigen::Vector3f v) { scale_ = v; };

template <typename Shape>
void
Renderer<Shape>::display()
{
  while (!glfwWindowShouldClose(window_))
  {
    // Scale to window size
    GLint windowWidth, windowHeight;
    glfwGetWindowSize(window_, &windowWidth, &windowHeight);
    glViewport(0, 0, windowWidth, windowHeight);

    // Draw stuff
    glClearColor(0.0, 0.8, 0.3, 1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION_MATRIX);
    glLoadIdentity();
    gluPerspective(45.6, (double)windowWidth / (double)windowHeight, 0.1, 500);

    glMatrixMode(GL_MODELVIEW_MATRIX);
    // View transformation
    gluLookAt(
      camera_position_(0), camera_position_(1), camera_position_(2), 
      world_position_(0),  world_position_(1),  world_position_(2),
      view_direction_(0),  view_direction_(1),  view_direction_(2)
    );

    // Model transformation
    glTranslatef(translation_(0), translation_(1), translation_(2));
    glScalef(scale_(0), scale_(1), scale_(2));
    glMultMatrixf(rotation_.data());
    /* glTranslatef(scale_(0) / 2.0, scale_(1) / 2.0, -scale_(2) / 2.0); */

    // Draw object
    Shape::draw();

    // Update Screen
    glfwSwapBuffers(window_);

    // Check for any input, or window movement
    glfwPollEvents();
  }
}

template <typename Shape>
void
Renderer<Shape>::operator()()
{
  assert(window_ != nullptr);
  display();
}

template <typename Shape>
void
Renderer<Shape>::controls(GLFWwindow* window, int key, int scancode, int action, int mods)
{
  if (action == GLFW_PRESS)
    if (key == GLFW_KEY_ESCAPE)
      glfwSetWindowShouldClose(window, GL_TRUE);
}

template <typename Shape>
GLFWwindow*
Renderer<Shape>::init_window(int width, int height)
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
