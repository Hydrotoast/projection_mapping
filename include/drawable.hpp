#ifndef DRAWABLE_HPP
#define DRAWABLE_HPP

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <array>

namespace giocc
{

// CRTP mixin for injecting a draw function into its deriving subtypes.
//
// Templates:
//   Derived  the subtype to inject into
template <typename Derived>
class Drawable
{
public:
  // Draws the derived type
  static void draw();
};

template <typename Derived>
void
Drawable<Derived>::draw()
{
  /* We have a color array and a vertex array */
  glEnableClientState(GL_VERTEX_ARRAY);
  glEnableClientState(GL_COLOR_ARRAY);
  glVertexPointer(3, GL_FLOAT, 0, Derived::vertices.data());
  glColorPointer(3, GL_FLOAT, 0, Derived::colors.data());

  /* Send data : 24 vertices */
  glDrawArrays(GL_QUADS, 0, Derived::vertices.size() / 3);

  /* Cleanup states */
  glDisableClientState(GL_COLOR_ARRAY);
  glDisableClientState(GL_VERTEX_ARRAY);
}

// Static cube definiiton
class Cube : public Drawable<Cube>
{
public:
  static std::array<GLfloat, 72> vertices;
  static std::array<GLfloat, 72> colors;
};

}

#endif
