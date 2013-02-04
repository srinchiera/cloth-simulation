uniform mat4 uProjMatrix;
uniform mat4 uModelViewMatrix;
uniform mat4 uNormalMatrix;

attribute vec3 aPosition;
attribute vec3 aNormal;

varying vec4 Ca;
varying vec4 Cd;
varying vec4 Cs;

varying vec4 V_eye;
varying vec4 L_eye;
varying vec4 N_eye;

void main(void)
{
V_eye = uModelViewMatrix * vec4(aPosition,1);
L_eye = gl_LightSource[0].position - V_eye;
N_eye = uNormalMatrix * vec4(aNormal, 1.0);

gl_Position = uProjMatrix * V_eye;
V_eye = -V_eye;

Ca = gl_FrontMaterial.ambient;
Cd = gl_FrontMaterial.diffuse;
Cs = gl_FrontMaterial.specular;
}
