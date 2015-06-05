#version 300

uniform mat4 projection;
uniform mat4 view;
uniform mat4 model;

in vec3 v_position;
in vec3 v_color;

out vec4 color;

void main(void) {
    gl_Position = projection * view * model * vec4(vertex, 1.0);
    color = vec4(v_color, 1.0);
}
