// Fluid surface fragment shader
// Adds lighting, reflections, and other water-like effects

#version 120

// Uniforms
uniform float time;             // Animation time for wave effects
uniform vec4 baseColor;         // Base water color (rgba)
uniform vec2 resolution;        // Screen resolution

// Vertex shader outputs
varying vec4 vertColor;
varying vec2 vertTexCoord;

// Constants
const vec3 lightDir = normalize(vec3(0.5, 0.5, 1.0));  // Light direction
const float specularStrength = 0.6;                   // Specular highlight intensity
const float shininess = 32.0;                         // Specular highlight size
const float ambientStrength = 0.4;                    // Ambient light level
const float waveSpeed = 2.0;                          // Wave animation speed
const float waveHeight = 0.03;                        // Wave height factor

// Wave function simulating water surface
vec3 waveNormal(vec2 position, float time) {
    // Generate wave patterns using sine functions
    float wave1 = sin(position.x * 0.1 + time * waveSpeed) * 
                 cos(position.y * 0.1 + time * waveSpeed * 0.8) * waveHeight;
    float wave2 = sin(position.x * 0.15 - time * waveSpeed * 1.2) * 
                 sin(position.y * 0.1 + time * waveSpeed * 0.6) * waveHeight;
    
    // Combine waves
    float waveZ = wave1 + wave2;
    
    // Calculate partial derivatives for normal
    float dx = cos(position.x * 0.1 + time * waveSpeed) * 
              cos(position.y * 0.1 + time * waveSpeed * 0.8) * 0.1 * waveHeight +
              cos(position.x * 0.15 - time * waveSpeed * 1.2) * 
              sin(position.y * 0.1 + time * waveSpeed * 0.6) * 0.15 * waveHeight;
    
    float dy = sin(position.x * 0.1 + time * waveSpeed) * 
              -sin(position.y * 0.1 + time * waveSpeed * 0.8) * 0.1 * waveHeight +
              sin(position.x * 0.15 - time * waveSpeed * 1.2) * 
              cos(position.y * 0.1 + time * waveSpeed * 0.6) * 0.1 * waveHeight;
    
    // Calculate normal from derivatives
    return normalize(vec3(-dx, -dy, 1.0));
}

void main() {
    // Use fragment coordinates for position-based effects
    vec2 position = gl_FragCoord.xy;
    
    // Calculate normal with wave animation
    vec3 normal = waveNormal(position, time);
    
    // Basic lighting calculation
    float diffuse = max(0.0, dot(normal, lightDir));
    
    // View direction (assuming view is directly above the water)
    vec3 viewDir = vec3(0.0, 0.0, 1.0);
    
    // Reflect light direction about the normal
    vec3 reflectDir = reflect(-lightDir, normal);
    
    // Calculate specular highlight
    float spec = pow(max(0.0, dot(viewDir, reflectDir)), shininess);
    float specular = specularStrength * spec;
    
    // Calculate light color (white light)
    vec3 ambient = ambientStrength * baseColor.rgb;
    vec3 diffuseLight = diffuse * baseColor.rgb;
    vec3 specularLight = specular * vec3(1.0, 1.0, 1.0);
    
    // Combine lighting components
    vec3 finalColor = ambient + diffuseLight + specularLight;
    
    // Add subtle color variation based on depth
    float depthVariation = 0.9 + 0.1 * sin(position.y * 0.01 + time * 0.5);
    finalColor *= depthVariation;
    
    // Add edge highlight
    float edgeHighlight = pow(1.0 - abs(dot(viewDir, normal)), 2.0) * 0.5;
    finalColor += vec3(edgeHighlight);
    
    // Output final color
    gl_FragColor = vec4(finalColor, baseColor.a);
} 