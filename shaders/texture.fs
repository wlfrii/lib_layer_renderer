#version 330 core
out vec4 fragColor;

in vec2 texCoord;

// texture sampler
uniform sampler2D image;
uniform sampler2D mask;

void main()
{
    vec4 color = texture(image, texCoord);
    vec4 m = texture(mask, texCoord);
    color.x += m.x * 0.7;
    color.y += m.x * 0.7;
    fragColor = color;
}