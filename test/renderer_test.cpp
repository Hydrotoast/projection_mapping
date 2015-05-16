#include "drawable.hpp"
#include "renderer.hpp"

using namespace giocc;

int
main(int argc, char** argv)
{
  Renderer<Cube> renderer{1024, 620};
  renderer.rotation(
      {
        0.6993, -0.0112,  0.7147,   0,
        0.5036, -0.7003,  -0.5062,  0,
        0.4954, 0.7182,   -0.4887,  0,
        0,      0,        0,        1
      });
  renderer();
  return 0;
}
