#version 330 core

in vec3 vertexInEyeSpace;
in vec3 normalInEyeSpace;

uniform vec4 ka;
uniform vec4 kd;
uniform vec4 ks;
uniform float specular;
uniform bool textured;
uniform sampler2D diffuseTex;

uniform vec4 Ia;
uniform vec4 Id;
uniform vec4 Is;

uniform vec4 lightInEyeSpace;

out vec4 finalColor;

void main() {
    vec3 n = normalize(normalInEyeSpace);
    vec3 l = normalize(-lightInEyeSpace.xyz);
    vec3 e = normalize(-vertexInEyeSpace);
    vec3 h = normalize(l + e);

    vec3 outColor = vec3(0, 0, 0);
    outColor += Ia.xyz * ka.xyz;
    // vec3 diffuseMult = kd.xyz;
    // if (textured) {
    //     diffuseMult *= texture(
    outColor += Id.xyz * kd.xyz * max(0.0, dot(l, n));
    outColor += Is.xyz * ks.xyz * pow(max(dot(h, n), 0.0), specular);

    finalColor.rgb = outColor;
    finalColor.a   = 1.0;
}
