#version 330 core
in vec4 Color;
in float Radius;

out vec4 FragColor;

void main() {
    // Calculate normalized coordinates from center of point
    vec2 circCoord = 2.0 * gl_PointCoord - 1.0;
    float dist = length(circCoord);
    
    // Create a much stronger metaball effect
    // This will help particles blend together into a single fluid mass
    float metaball = 1.0 - smoothstep(0.0, 0.9, dist);
    
    // Square the metaball value to create a sharper falloff at the edges
    // This helps create a more cohesive surface appearance
    metaball = pow(metaball, 2.0);
    
    // Apply a threshold for the fluid surface
    // This creates a more defined boundary for the fluid
    float fluidSurface = smoothstep(0.3, 0.4, metaball);
    
    // Create a water-like blue color base
    vec3 deepWater = vec3(0.05, 0.1, 0.4);  // Dark blue for depth
    vec3 shallowWater = vec3(0.1, 0.3, 0.7);  // Lighter blue for surface
    vec3 waterColor = mix(deepWater, shallowWater, metaball);
    
    // Add subtle inner highlight for fluid volume effect
    float innerHighlight = smoothstep(0.5, 0.0, dist) * 0.3;
    waterColor += vec3(innerHighlight);
    
    // Add edge highlight for surface tension effect
    float edgeHighlight = smoothstep(0.8, 0.95, dist) * 0.2;
    waterColor += vec3(edgeHighlight);
    
    // Calculate alpha for smooth blending
    // A higher alpha threshold creates more cohesive-looking fluid
    float alpha = smoothstep(0.2, 0.5, metaball);
    alpha = alpha * 0.95; // Allow slight transparency even at center
    
    // Discard fragments outside the maximum radius, but with soft edge
    if (dist > 1.0) {
        discard;
    }
    
    // Output final color - use a higher alpha near the center for better cohesion
    FragColor = vec4(waterColor, alpha);
}