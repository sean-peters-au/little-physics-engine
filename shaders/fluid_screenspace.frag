// High-Quality Screen-Space Fluid Shader
#version 120

uniform sampler2D densityTexture; // Blurred density (RGBA, density usually in Alpha)
uniform vec4 baseColor;          // Base fluid color tint (RGBA)
uniform float threshold;         // Density threshold for surface
uniform float smoothness;        // Softness of the density threshold edge
uniform vec2 resolution;         // Resolution of the densityTexture/simulation area
uniform float time;              // Optional: for animation

// --- Constants for Appearance ---
const float REFRACT_STRENGTH = 0.05; // How much background refracts
const float REFLECT_STRENGTH = 0.4;  // How much environment reflects (needs env map/SSR)
const float SPECULAR_STRENGTH = 0.8; // Intensity of highlights
const float SHININESS = 64.0;        // Size of highlights
const float NORMAL_STRENGTH = 1.5;   // How pronounced the surface normal is
const vec3 LIGHT_DIR = normalize(vec3(0.6, 0.7, 0.8)); // Directional light
const float GRADIENT_ALPHA_BOOST = 1.5; // How much high gradient boosts alpha
const float MIN_ALPHA_FOR_SHADING = 0.1; // Don't compute expensive lighting for very transparent pixels

// Function to calculate normal AND gradient magnitude from density field
vec3 calculateNormalAndGradient(vec2 uv, out float gradientMagnitude) {
    vec2 texel = 1.0 / resolution;
    float center = texture2D(densityTexture, uv).a;
    float N = texture2D(densityTexture, uv + vec2(0.0, texel.y)).a;
    float S = texture2D(densityTexture, uv - vec2(0.0, texel.y)).a;
    float E = texture2D(densityTexture, uv + vec2(texel.x, 0.0)).a;
    float W = texture2D(densityTexture, uv - vec2(texel.x, 0.0)).a;

    vec2 gradient = vec2(E - W, N - S); 
    gradientMagnitude = length(gradient);

    vec3 normal = vec3(-gradient * NORMAL_STRENGTH, 1.0);
    return normalize(normal);
}

void main() {
    vec2 uv = gl_TexCoord[0].st;
    float density = texture2D(densityTexture, uv).a;

    // --- Calculate Surface Alpha --- 
    // Base alpha from density thresholding
    float densityAlpha = smoothstep(threshold, threshold + smoothness, density);
    
    // Calculate gradient magnitude
    float gradientMag;
    vec3 normal = calculateNormalAndGradient(uv, gradientMag); // Keep normal calc
    
    // Alpha contribution purely from gradient (sensitive to small changes)
    // Make this smoothstep range narrow and start low
    float gradientAlpha = smoothstep(0.05, 0.25, gradientMag * GRADIENT_ALPHA_BOOST); 

    // --- New Combination Logic --- 
    // Ensure pixels with *any* non-negligible density get some base visibility,
    // strongly boosted by the presence of a gradient.
    float minVisibilityFactor = 0.1; // Minimum alpha if density > ~0
    float gradientBoostFactor = 0.8; // How much gradient contributes extra alpha
    
    // Base visibility for any non-zero density
    float baseVisibility = step(0.005, density) * minVisibilityFactor;
    
    // Add gradient contribution on top of base visibility
    float combinedAlpha = baseVisibility + gradientAlpha * gradientBoostFactor;
    
    // Make sure strong density overrides the gradient boost if it's higher
    float surfaceAlpha = max(densityAlpha, combinedAlpha);
    
    // Clamp final alpha
    surfaceAlpha = clamp(surfaceAlpha, 0.0, 1.0);
    // --- End New Combination --- 

    // --- Discard if alpha is too low --- 
    if (surfaceAlpha < 0.01) {
        discard;
    }

    // --- Lighting Calculations (only if pixel is somewhat opaque) --- 
    vec3 finalColor = baseColor.rgb * 0.4; // Start with ambient
    if (surfaceAlpha > MIN_ALPHA_FOR_SHADING) {
        vec3 viewDir = vec3(0.0, 0.0, 1.0); 

        // Diffuse
        float diffuse = max(0.0, dot(normal, LIGHT_DIR));

        // Specular
        vec3 reflectDir = reflect(-LIGHT_DIR, normal);
        float spec = pow(max(0.0, dot(viewDir, reflectDir)), SHININESS);
        vec3 specular = SPECULAR_STRENGTH * spec * vec3(1.0);

        // Base Color & Depth Tinting
        vec3 shadedColor = baseColor.rgb * mix(0.7, 1.0, density); // Tint based on original density

        // Combine Lighting & Color 
        finalColor = shadedColor * (0.4 + diffuse * 0.6) + specular;

        // Fresnel/Edge Highlighting
        float fresnel = pow(1.0 - max(0.0, dot(normal, viewDir)), 3.0); 
        finalColor = mix(finalColor, vec3(1.0), fresnel * 0.5);
    }

    // --- Final Output --- 
    finalColor = clamp(finalColor, 0.0, 1.0);
    gl_FragColor = vec4(finalColor, baseColor.a * surfaceAlpha);
}
