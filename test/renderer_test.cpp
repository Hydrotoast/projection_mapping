#include "glfw_renderer/gl_engine.hpp"
#include "glfw_renderer/camera_builder.hpp"
#include "glfw_renderer/renderer.hpp"

#include "glfw_renderer/loader.hpp"

#include "glfw_renderer/shape_factory.hpp"
#include "glfw_renderer/scenegraph.hpp"

using namespace giocc;

int
main(int argc, char** argv)
{
  Renderer<GLEngine> renderer{1024, 620};
  ShaderProgramFileLoader<GLEngine, FileLoader> loader;
  ShaderProgram<GLEngine> program = loader.load("shader.vert", "shader.frag");
  program();

  AttributeResolver<GLEngine> attribute_resolver{program};
  UniformResolver<GLEngine> uniform_resolver{program};
  renderer.uniform_resolver(uniform_resolver);

  CameraBuilder<GLEngine> camera_builder{80, -150};

  camera_builder.add_persp_matrix();
  camera_builder.add_ndc_matrix(640, 480);

  /* camera_builder.add_ortho_matrix(1, 1); */

  std::unique_ptr<Camera> camera = std::move(camera_builder);

  auto shape_ptr = std::move(ShapeFactory<GLEngine, Cube>{program});
  auto geode = std::unique_ptr<Geode<GLEngine, Cube::N>>{
      new Geode<GLEngine, Cube::N>{std::move(shape_ptr)}};
  Scenegraph<GLEngine, Cube::N> scene;
  scene.camera(std::move(camera));
  scene.geode(std::move(geode));

  Eigen::Matrix4f view_translation;
  view_translation <<
      1, 0, 0, 0,
      0, -1, 0, 0,
      0, 0, 1, -150,
      0, 0, 0, 1;
  scene.camera().view(view_translation);

  Eigen::Matrix4f rotation;
  rotation <<
      0.6993, -0.0112,  0.7147,   0,
      0.5036, -0.7003,  -0.5062,  0,
      0.4954, 0.7182,   -0.4887,  0,
      0,      0,        0,        1;
  Eigen::Matrix4f scale;
  scale <<
      40, 0,    0,    0,
      0,   40,  0,    0,
      0,   0,   40,   0,
      0,   0,   0,    1;
  scene.geode().transformation(rotation * scale);
  renderer(scene);
  return 0;
}
