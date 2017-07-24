#version 330

in vec4 line0;
uniform mat4 fullMat;
out vec2 p0;
out vec2 p1;
void main (void)
{
    p0 = line0.xy;
    p1 = line0.zw;
}
