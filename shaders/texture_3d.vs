#version 330 core
layout (location = 0) in vec4 in_pos;
layout (location = 1) in vec4 in_color;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

out vec4 my_color;

void main()
{
    gl_Position = projection * view * model * in_pos;

    my_color = in_color;
}