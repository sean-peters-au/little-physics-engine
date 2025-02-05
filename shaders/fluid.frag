uniform sampler2D texture;
uniform float threshold;    // Intensity threshold for fluid appearance.
uniform float smoothness;   // Controls the softness of the fluid's edges.

void main() {
    vec2 uv = gl_TexCoord[0].xy;
    vec4 texColor = texture2D(texture, uv);

    // You can use the red channel (or the overall intensity) as the contribution.
    float intensity = texColor.r;
    
    // Create a smooth transition at the edge using smoothstep.
    float alpha = smoothstep(threshold, threshold + smoothness, intensity);
    
    // Set the fluid color (example here is a bluish tint).
    vec3 baseColor = vec3(0.0, 0.5, 1.0);
    gl_FragColor = vec4(baseColor * alpha, alpha);
}