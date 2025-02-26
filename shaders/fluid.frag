uniform sampler2D texture;  // Put back the texture uniform
uniform float threshold;    // Intensity threshold for fluid appearance.
uniform float smoothness;   // Controls the softness of the fluid's edges.

void main() {
    vec2 uv = gl_TexCoord[0].xy;
    vec4 texColor = texture2D(texture, uv);  // Use the declared texture uniform

    // Use the average RGB for better intensity with blue particles
    float intensity = (texColor.r + texColor.g + texColor.b) / 3.0;
    
    // Create a smooth transition at the edge using smoothstep.
    float alpha = smoothstep(threshold, threshold + smoothness, intensity);
    
    // Make the fluid more blue-cyan for better visibility
    vec3 baseColor = vec3(0.1, 0.6, 1.0);
    gl_FragColor = vec4(baseColor * alpha, alpha);
}