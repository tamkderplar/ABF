#version 330
#ifdef NODEF
void EmitVertex();
void EndPrimitive();
#endif
layout(lines) in;
layout(line_strip, max_vertices = 256) out;

in vec2 p0[];
in vec2 p1[];
uniform mat4 fullMat;
const float pi = 3.14159265358979323846;
const int div = 63;
const float divf = 62.0;
/*out vec3 color;*/
void main (void)
{
    /*color = vec3(0,1,0);*/
    gl_Position = fullMat*vec4(p0[1]-p0[0],-1,1);
    gl_Position = gl_Position/gl_Position.w;
    gl_Position.z = gl_Position.z/16.0;
    EmitVertex();
    gl_Position = fullMat*vec4(p0[1]-p1[0],-1,1);
    gl_Position = gl_Position/gl_Position.w;
    gl_Position.z = gl_Position.z/16.0;
    EmitVertex();
    EndPrimitive();
    for(int i=0;i<div;++i){
        float theta = float(i)*2.0*pi/divf;
        mat2 mat = mat2(cos(theta),sin(theta),-sin(theta),cos(theta));
        gl_Position = fullMat*vec4(p0[1]-mat*p0[0],(theta-pi)/pi,1);
        gl_Position = gl_Position/gl_Position.w;
        gl_Position.z = gl_Position.z/16.0;
        EmitVertex();
    }
    EndPrimitive();
    for(int i=0;i<div;++i){
        float theta = float(i)*2.0*pi/divf;
        mat2 mat = mat2(cos(theta),sin(theta),-sin(theta),cos(theta));
        gl_Position = fullMat*vec4(p1[1]-mat*p0[0],(theta-pi)/pi,1);
        gl_Position = gl_Position/gl_Position.w;
        gl_Position.z = gl_Position.z/16.0;
        EmitVertex();
    }
    EndPrimitive();
    for(int i=0;i<div;++i){
        float theta = float(i)*2.0*pi/divf;
        mat2 mat = mat2(cos(theta),sin(theta),-sin(theta),cos(theta));
        gl_Position = fullMat*vec4(p0[1]-mat*p1[0],(theta-pi)/pi,1);
        gl_Position = gl_Position/gl_Position.w;
        gl_Position.z = gl_Position.z/16.0;
        EmitVertex();
    }
    EndPrimitive();
    for(int i=0;i<div;++i){
        float theta = float(i)*2.0*pi/divf;
        mat2 mat = mat2(cos(theta),sin(theta),-sin(theta),cos(theta));
        gl_Position = fullMat*vec4(p1[1]-mat*p1[0],(theta-pi)/pi,1);
        gl_Position = gl_Position/gl_Position.w;
        gl_Position.z = gl_Position.z/16.0;
        EmitVertex();
    }
    EndPrimitive();
}
