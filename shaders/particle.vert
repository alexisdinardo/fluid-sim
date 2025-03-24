#version 330 core
layout (location = 0) in vec2 aPos;
layout (location = 1) in vec4 aColor;
layout (location = 2) in float aRadius;

uniform mat4 projection;
uniform mat4 view;

out vec4 Color;
out float Radius;

void main() {
    // Transform particle position to screen space
    gl_Position = projection * view * vec4(aPos, 0.0, 1.0);
    
    // Pass color and radius to fragment shader
    Color = aColor;
    Radius = aRadius;
    
    // Make particles much larger for better fluid blending
    // The large overlapping particles create the fluid effect
    gl_PointSize = Radius * 500.0;
}