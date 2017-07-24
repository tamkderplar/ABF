#version 330

layout(location = 0, index = 0) out vec4 fragColor;
in float coord_y;
in vec2 px0;
in vec2 objdir;
const vec3 color = vec3(0,1,0);
const vec3 cameradir = normalize(vec3(1,1,1));
const float pi = 3.14159265358979323846;
uniform mat4 fullMat;

void main (void)
{
    //float x = coords.x;
    //float theta = coords.y;
    float theta = coord_y;
    mat2 mat = mat2(cos(theta),sin(theta),-sin(theta),cos(theta));
    vec3 normal = normalize(cross(vec3(objdir,0),vec3(mat*mat2(0,1,-1,0)*px0,1/pi)));
    normal = vec3(fullMat*vec4(normalize(normal),0));
    fragColor = vec4(color*abs(dot(normal,cameradir)),1);
}
