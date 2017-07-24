#version 330
#ifdef NODEF
void EmitVertex();
void EndPrimitive();
#endif
layout(lines) in;
layout(triangle_strip, max_vertices = 256) out;

in vec2 p0[];//start of: 0 - object; 1 - obstacle
in vec2 p1[];//end of:   0 - object; 1 - obstacle
uniform mat4 fullMat;
const float pi = 3.14159265358979323846;
const int div = 25;
const float divf = 24.0;
out float coord_y;
out vec2 px0;
out vec2 objdir;
void main (void)
{
    for(int i=0;i<div;++i){
        float theta = float(i)*2.0*pi/divf;
        mat2 mat = mat2(cos(theta),sin(theta),-sin(theta),cos(theta));
        gl_Position = fullMat*vec4(p0[1]-mat*p0[0],(theta-pi)/pi,1);
        gl_Position = gl_Position/gl_Position.w;
        gl_Position.z = gl_Position.z/16.0;
        //normal = normalize(cross(vec3(p1[0]-p0[0],0),vec3(mat*mat2(0,1,-1,0)*p0[1],1/pi)));
        //coords = vec2(0,(theta-pi)/pi);
        coord_y = (theta-pi)/pi;
        px0 = p0[0];
        objdir = p1[0]-p0[0];
        EmitVertex();
        gl_Position = fullMat*vec4(p0[1]-mat*p1[0],(theta-pi)/pi,1);
        gl_Position = gl_Position/gl_Position.w;
        gl_Position.z = gl_Position.z/16.0;
        //normal = normalize(cross(vec3(p1[0]-p0[0],0),vec3(mat*mat2(0,1,-1,0)*p1[1],1/pi)));
        //coords = vec2(1,(theta-pi)/pi);
        px0 = p1[0];
        EmitVertex();
    }
    EndPrimitive();
    for(int i=0;i<div;++i){
        float theta = float(i)*2.0*pi/divf;
        mat2 mat = mat2(cos(theta),sin(theta),-sin(theta),cos(theta));
        gl_Position = fullMat*vec4(p1[1]-mat*p1[0],(theta-pi)/pi,1);
        gl_Position = gl_Position/gl_Position.w;
        gl_Position.z = gl_Position.z/16.0;
        //normal = normalize(cross(vec3(p1[1]-p0[1],0),vec3(mat*mat2(0,1,-1,0)*p0[1],1/pi)));
        //coords = vec2(0,(theta-pi)/pi);
        coord_y = (theta-pi)/pi;
        px0 = p1[0];
        objdir = p0[0]-p1[0];
        EmitVertex();
        gl_Position = fullMat*vec4(p1[1]-mat*p0[0],(theta-pi)/pi,1);
        gl_Position = gl_Position/gl_Position.w;
        gl_Position.z = gl_Position.z/16.0;
        //normal = normalize(cross(vec3(p1[1]-p0[1],0),vec3(mat*mat2(0,1,-1,0)*p1[1],1/pi)));
        //coords = vec2(1,(theta-pi)/pi);
        px0 = p0[0];
        EmitVertex();
    }
    EndPrimitive();
    /*for(int i=0;i<div;++i){
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
    }*/
    EndPrimitive();
}
