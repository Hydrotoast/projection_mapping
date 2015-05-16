#ifndef CUBE_RENDERER_HPP
#define CUBE_RENDERER_HPP

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
  using Mat4f = std::array<float, 16>;
  using Vec3f = std::array<float, 3>;

  // Initialized a GLFW window with the specified width and height parameters.
  Renderer(int width, int height);
  ~Renderer();

  // Rendering loop to continously redraw the scene. The rendering loop will
  // draw the cube in the scene with the specified camera parameters and object
  // parameters.
  void display();

  // Accessors
  const Vec3f& camera_position() const;
  const Vec3f& view_direction() const;
  const Vec3f& world_position() const;
  const Mat4f& rotation() const;
  const Vec3f& translation() const;
  const Vec3f& scale() const;

  int width() const;
  int height() const;

  // Modifiers
  void camera_position(Vec3f v);
  void view_direction(Vec3f v);
  void world_position(Vec3f v);
  void rotation(Mat4f m);
  void translation(Vec3f v);
  void scale(Vec3f);

  // Runs the system
  void operator()();
private:
  // Handles controls on the GLFW window
  static void controls(GLFWwindow* window, int key, int scancode, int action, int mods);

  // Initializes the window
  static GLFWwindow* init_window(int width, int height);

  // Scene extrinsic paramters
  Vec3f camera_position_;
  Vec3f view_direction_;
  Vec3f world_position_;

  // Object extrinsic parameters
  Mat4f rotation_;
  Vec3f translation_;
  Vec3f scale_;

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
    , camera_position_{{0, 0, -5}}
    , view_direction_{{0, 1, 0}}
    , world_position_{{0, 0, 0}}
    // Object extrinsic parameters
    , rotation_{{
      1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1, 0,
      0, 0, 0, 1}}
    , translation_{{0, 0, 0}}
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
const typename Renderer<Shape>::Vec3f&
Renderer<Shape>::camera_position() const { return camera_position_; };

template <typename Shape>
const typename Renderer<Shape>::Vec3f&
Renderer<Shape>::view_direction() const { return view_direction_; };

template <typename Shape>
const typename Renderer<Shape>::Vec3f&
Renderer<Shape>::world_position() const { return world_position_; };

template <typename Shape>
const typename Renderer<Shape>::Mat4f&
Renderer<Shape>::rotation() const { return rotation_; };

template <typename Shape>
const typename Renderer<Shape>::Vec3f&
Renderer<Shape>::translation() const { return translation_; };

template <typename Shape>
const typename Renderer<Shape>::Vec3f&
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
Renderer<Shape>::camera_position(Vec3f v) { camera_position_ = v; };

template <typename Shape>
void
Renderer<Shape>::view_direction(Vec3f v) { view_direction_ = v; };

template <typename Shape>
void
Renderer<Shape>::world_position(Vec3f v) { world_position_ = v; };

template <typename Shape>
void
Renderer<Shape>::rotation(Mat4f m) { rotation_ = m; };

template <typename Shape>
void
Renderer<Shape>::translation(Vec3f v) { translation_ = v; };

template <typename Shape>
void
Renderer<Shape>::scale(Vec3f v) { scale_ = v; };

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
      camera_position_.at(0), camera_position_.at(1), camera_position_.at(2), 
      world_position_.at(0),  world_position_.at(1),  world_position_.at(2),
      view_direction_.at(0),  view_direction_.at(1),  view_direction_.at(2)
    );

    // Model transformation
    glTranslatef(translation_.at(0), translation_.at(1), translation_.at(2));
    glScalef(scale_.at(0), scale_.at(1), scale_.at(2));
    glMultMatrixf(rotation_.data());
    
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
