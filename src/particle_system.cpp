#include "particle_system.h"
#include <algorithm>
#include <chrono>

ParticleSystem::ParticleSystem(int maxParticles, float width, float height)
    : containerWidth(width), containerHeight(height),
      cellSize(SMOOTHING_RADIUS),
      rng(std::chrono::system_clock::now().time_since_epoch().count()) {
    
    // Initialize particles in a more structured grid pattern
    // This creates a more cohesive initial fluid body
    const float particleSpacing = SMOOTHING_RADIUS * 0.2f; // Even closer spacing for much denser fluid
    
    // Calculate how many particles to create in each dimension
    int particlesWidth = static_cast<int>(width * 0.35f / particleSpacing);  // More concentrated particles
    int particlesHeight = static_cast<int>(height * 0.35f / particleSpacing); // More square-like initial shape
    
    const float startX = width * 0.35f;  // Start more towards the center
    const float startY = height * 0.6f;  // Start higher up
    
    // Create particles in a grid with small random offsets for natural look
    particles.reserve(maxParticles);
    for (int i = 0; i < particlesWidth && particles.size() < maxParticles; ++i) {
        for (int j = 0; j < particlesHeight && particles.size() < maxParticles; ++j) {
            // Add very small random offset for more natural look
            std::uniform_real_distribution<float> offset(-0.002f, 0.002f);
            
            // Position in a grid pattern with offset
            float x = startX + i * particleSpacing + offset(rng);
            float y = startY + j * particleSpacing + offset(rng);
            
            glm::vec2 pos(x, y);
            
            // Water-like blue color with subtle variation for depth
            float blueVariation = 0.03f * (float)rand() / RAND_MAX;
            glm::vec4 color(0.1f, 0.3f + blueVariation, 0.9f, 0.98f);
            
            // Use consistent radius for all particles
            float radius = 0.03f;  // Even smaller radius for better density
            
            particles.push_back(Particle(pos, color, radius));
        }
    }
    
    // Fill remaining capacity with random particles very close to the main body
    std::uniform_real_distribution<float> distX(startX - particleSpacing * 0.5f, startX + particlesWidth * particleSpacing * 1.1f);
    std::uniform_real_distribution<float> distY(startY - particleSpacing * 0.5f, startY + particlesHeight * particleSpacing * 1.1f);
    
    while (particles.size() < maxParticles) {
        glm::vec2 pos(distX(rng), distY(rng));
        float blueVariation = 0.03f * (float)rand() / RAND_MAX;
        glm::vec4 color(0.1f, 0.3f + blueVariation, 0.9f, 0.98f);
        float radius = 0.03f;
        particles.push_back(Particle(pos, color, radius));
    }
    
    // Initialize the grid for spatial partitioning
    grid.resize(GRID_SIZE * GRID_SIZE);
    
    // Setup OpenGL buffers
    setupBuffers();
}

ParticleSystem::~ParticleSystem() {
    // *** TODO: Clean up OpenGL resources ***
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
}

void ParticleSystem::update(float dt, glm::vec2 mousePos, bool mousePressed) {
    // Start timing for performance measurement
    auto startTime = std::chrono::high_resolution_clock::now();
    
    // *** TODO: Implement neighbor finding with spatial partitioning ***
    updateGrid();
    findNeighbors();
    
    // *** TODO: Calculate density and pressure for each particle ***
    calculateDensityPressure();
    
    // *** TODO: Calculate forces (pressure, viscosity, gravity) ***
    calculateForces();
    
    // *** TODO: Handle mouse interaction ***
    if (mousePressed) {
        applyMouseForce(mousePos, mousePressed);
    }
    
    // *** TODO: Update positions with numerical integration ***
    integrate(dt);
    
    // *** TODO: Handle boundary conditions ***
    handleBoundaries();
    
    // End timing
    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count();
    
    // Only print timing every 100 frames to avoid spamming
    static int frameCount = 0;
    static float totalTime = 0.0f;
    
    totalTime += duration / 1000.0f;  // Convert to milliseconds
    frameCount++;
    
    if (frameCount >= 100) {
        std::cout << "Average update time: " << totalTime / frameCount << " ms" << std::endl;
        totalTime = 0.0f;
        frameCount = 0;
    }
}

void ParticleSystem::render(Shader &shader) {
    // *** TODO: Update particle data for rendering ***
    std::vector<ParticleData> particleData(particles.size());
    
    for (size_t i = 0; i < particles.size(); i++) {
        particleData[i].position = particles[i].position;
        particleData[i].color = particles[i].color;
        particleData[i].radius = particles[i].radius;
    }
    
    // *** TODO: Upload data to GPU and render particles ***
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, particleData.size() * sizeof(ParticleData), particleData.data(), GL_DYNAMIC_DRAW);
    
    glBindVertexArray(VAO);
    glDrawArrays(GL_POINTS, 0, particles.size());
    glBindVertexArray(0);
}

void ParticleSystem::reset() {
    // Clear existing particles to create a fresh state
    particles.clear();
    
    // Use the same parameters as constructor for consistency
    const float particleSpacing = SMOOTHING_RADIUS * 0.2f; // Even closer spacing for much denser fluid
    
    // Calculate how many particles to create in each dimension
    int particlesWidth = static_cast<int>(containerWidth * 0.35f / particleSpacing);  // More concentrated particles
    int particlesHeight = static_cast<int>(containerHeight * 0.35f / particleSpacing); // More square-like initial shape
    
    const float startX = containerWidth * 0.35f;  // Start more towards the center
    const float startY = containerHeight * 0.6f;  // Start higher up
    
    // Calculate how many particles we can create
    int maxParticles = particles.capacity();
    
    // Create particles in a grid with small random offsets
    for (int i = 0; i < particlesWidth && particles.size() < maxParticles; ++i) {
        for (int j = 0; j < particlesHeight && particles.size() < maxParticles; ++j) {
            // Add very small random offset for more natural look
            std::uniform_real_distribution<float> offset(-0.002f, 0.002f);
            
            // Position in a grid pattern with offset
            float x = startX + i * particleSpacing + offset(rng);
            float y = startY + j * particleSpacing + offset(rng);
            
            glm::vec2 pos(x, y);
            
            // Water-like blue color with subtle variation for depth
            float blueVariation = 0.03f * (float)rand() / RAND_MAX;
            glm::vec4 color(0.1f, 0.3f + blueVariation, 0.9f, 0.98f);
            
            // Use consistent radius for all particles
            float radius = 0.03f;  // Even smaller radius for better density
            
            particles.push_back(Particle(pos, color, radius));
        }
    }
    
    // Fill remaining capacity with random particles very close to the main body
    std::uniform_real_distribution<float> distX(startX - particleSpacing * 0.5f, startX + particlesWidth * particleSpacing * 1.1f);
    std::uniform_real_distribution<float> distY(startY - particleSpacing * 0.5f, startY + particlesHeight * particleSpacing * 1.1f);
    
    while (particles.size() < maxParticles) {
        glm::vec2 pos(distX(rng), distY(rng));
        float blueVariation = 0.03f * (float)rand() / RAND_MAX;
        glm::vec4 color(0.1f, 0.3f + blueVariation, 0.9f, 0.98f);
        float radius = 0.03f;
        particles.push_back(Particle(pos, color, radius));
    }
    
    std::cout << "Simulation reset with " << particles.size() << " particles" << std::endl;
}

void ParticleSystem::addParticles(int count) {
    // Find the current center of mass to add particles near existing fluid
    glm::vec2 centerOfMass(0.0f);
    
    if (!particles.empty()) {
        for (const auto& p : particles) {
            centerOfMass += p.position;
        }
        centerOfMass /= particles.size();
    } else {
        // Default to upper middle if no particles
        centerOfMass = glm::vec2(containerWidth * 0.5f, containerHeight * 0.75f);
    }
    
    // Add particles in a small radius around the center of mass
    float radius = SMOOTHING_RADIUS * 5.0f;
    
    for (int i = 0; i < count; i++) {
        // Random angle and distance from center
        float angle = static_cast<float>(rand()) / RAND_MAX * 2.0f * 3.14159f;
        float distance = static_cast<float>(rand()) / RAND_MAX * radius;
        
        // Convert to Cartesian coordinates
        float x = centerOfMass.x + cos(angle) * distance;
        float y = centerOfMass.y + sin(angle) * distance;
        
        // Keep within boundaries
        x = std::max(0.1f, std::min(containerWidth - 0.1f, x));
        y = std::max(0.1f, std::min(containerHeight - 0.1f, y));
        
        // Create new particle
        glm::vec2 pos(x, y);
        float blueVariation = 0.03f * (float)rand() / RAND_MAX;
        glm::vec4 color(0.1f, 0.3f + blueVariation, 0.9f, 0.98f);
        float particleRadius = 0.03f;  // Match the radius used in initialization
        
        particles.push_back(Particle(pos, color, particleRadius));
    }
    
    std::cout << "Particle count: " << particles.size() << std::endl;
}

void ParticleSystem::removeParticles(int count) {
    // *** TODO: Implement removing particles ***
    count = std::min(count, static_cast<int>(particles.size()));
    if (count > 0) {
        particles.resize(particles.size() - count);
        std::cout << "Particle count: " << particles.size() << std::endl;
    }
}

void ParticleSystem::setupBuffers() {
    // *** TODO: Generate and configure VAO and VBO for particles ***
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    
    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    
    // Allocate memory but don't upload any data yet
    glBufferData(GL_ARRAY_BUFFER, particles.size() * sizeof(ParticleData), nullptr, GL_DYNAMIC_DRAW);
    
    // Position attribute (0)
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(ParticleData), (void*)0);
    glEnableVertexAttribArray(0);
    
    // Color attribute (1)
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(ParticleData), (void*)(2 * sizeof(float)));
    glEnableVertexAttribArray(1);
    
    // Radius attribute (2)
    glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, sizeof(ParticleData), (void*)(6 * sizeof(float)));
    glEnableVertexAttribArray(2);
    
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

void ParticleSystem::updateGrid() {
    // Clear the grid
    for (auto& cell : grid) {
        cell.clear();
    }
    
    // Add particles to the grid
    for (size_t i = 0; i < particles.size(); i++) {
        int cellX = particles[i].position.x / cellSize;
        int cellY = particles[i].position.y / cellSize;
        
        // Clamp to grid bounds
        cellX = std::max(0, std::min(cellX, GRID_SIZE - 1));
        cellY = std::max(0, std::min(cellY, GRID_SIZE - 1));
        
        int index = cellY * GRID_SIZE + cellX;
        grid[index].push_back(i);
    }
}

void ParticleSystem::findNeighbors() {
    // Implement neighbor finding using the grid ***
    for (auto& p : particles) {
        p.neighbors.clear();
    }
    
    for (size_t i = 0; i < particles.size(); i++) {
        int cellX = particles[i].position.x / cellSize;
        int cellY = particles[i].position.y / cellSize;
        
        // Clamp to grid bounds
        cellX = std::max(0, std::min(cellX, GRID_SIZE - 1));
        cellY = std::max(0, std::min(cellY, GRID_SIZE - 1));
        
        // Check 3x3 neighborhood of cells
        for (int dy = -1; dy <= 1; dy++) {
            for (int dx = -1; dx <= 1; dx++) {
                int nx = cellX + dx;
                int ny = cellY + dy;
                
                // Skip out-of-bounds cells
                if (nx < 0 || nx >= GRID_SIZE || ny < 0 || ny >= GRID_SIZE) {
                    continue;
                }
                
                int index = ny * GRID_SIZE + nx;
                
                // Check all particles in this cell
                for (int j : grid[index]) {
                    if (i == j) continue;  // Skip self
                    
                    float dist = glm::length(particles[i].position - particles[j].position);
                    if (dist < SMOOTHING_RADIUS) {
                        particles[i].neighbors.push_back(j);
                    }
                }
            }
        }
    }
}

void ParticleSystem::calculateDensityPressure() {
    // Calculate density and pressure for each particle ***
    for (auto& p : particles) {
        p.density = 0.0f;
        
        // Self-contribution to density
        p.density += PARTICLE_MASS * kernelPoly6(0.0f, SMOOTHING_RADIUS);
        
        // Contribution from neighbors
        for (int j : p.neighbors) {
            float dist = glm::length(p.position - particles[j].position);
            p.density += PARTICLE_MASS * kernelPoly6(dist, SMOOTHING_RADIUS);
        }
        
        // Calculate pressure using equation of state
        p.pressure = PRESSURE_CONSTANT * (p.density - REST_DENSITY);
        if (p.pressure < 0.0f) p.pressure = 0.0f;  // Prevent negative pressure
    }
}

void ParticleSystem::calculateForces() {
    // Calculate forces for each particle
    for (auto& p : particles) {
        // Start with reduced gravity for more stable fluid behavior
        p.force = glm::vec2(0.0f, -GRAVITY * 0.8f);
        
        // Surface tension force
        glm::vec2 surfaceTensionForce(0.0f);
        float surfaceTensionCoeff = 0.8f;
        
        for (int j : p.neighbors) {
            Particle& neighbor = particles[j];
            
            glm::vec2 directionToNeighbor = p.position - neighbor.position;
            float dist = glm::length(directionToNeighbor);
            
            // Avoid zero division
            if (dist < 0.0001f) {
                continue;
            }
            
            // Normalize the direction
            glm::vec2 normalizedDirection = directionToNeighbor / dist;
            
            // Calculate and apply pressure force with increased strength
            float pressureGradient = (p.pressure + neighbor.pressure) / (2.0f * neighbor.density);
            float pressureMagnitude = pressureGradient * kernelSpikyGradient(dist, SMOOTHING_RADIUS) * 1.2f;
            p.force += normalizedDirection * pressureMagnitude;
            
            // Calculate and apply viscosity force with increased strength
            glm::vec2 velocityDiff = neighbor.velocity - p.velocity;
            float viscosityStrength = VISCOSITY * kernelViscosity(dist, SMOOTHING_RADIUS) / neighbor.density * 1.5f;
            p.force += velocityDiff * viscosityStrength;
            
            // Calculate surface tension force
            float surfaceTensionMagnitude = surfaceTensionCoeff * kernelPoly6(dist, SMOOTHING_RADIUS);
            surfaceTensionForce += normalizedDirection * surfaceTensionMagnitude;
        }
        
        // Apply surface tension force
        p.force += surfaceTensionForce;
        
        // Add cohesion force to pull particles together
        float cohesionStrength = 0.3f;
        for (int j : p.neighbors) {
            Particle& neighbor = particles[j];
            glm::vec2 directionToNeighbor = neighbor.position - p.position;
            float dist = glm::length(directionToNeighbor);
            
            if (dist > 0.0001f) {
                float cohesionMagnitude = cohesionStrength * kernelPoly6(dist, SMOOTHING_RADIUS);
                p.force += glm::normalize(directionToNeighbor) * cohesionMagnitude;
            }
        }

        // Limit force magnitude to prevent explosion but allow slightly higher forces
        float maxForce = 120.0f;
        float forceMagnitude = glm::length(p.force);
        if (forceMagnitude > maxForce) {
            p.force = glm::normalize(p.force) * maxForce;
        }
    }
}

void ParticleSystem::integrate(float dt) {
    // Improved numerical integration for fluid stability
    for (auto& p : particles) {
        // Acceleration = force / density
        glm::vec2 acceleration = p.force / std::max(p.density, 0.0001f);
        
        // Limit maximum acceleration to prevent instability
        float maxAccel = 50.0f;
        if (glm::length(acceleration) > maxAccel) {
            acceleration = glm::normalize(acceleration) * maxAccel;
        }
        
        // Semi-implicit Euler integration
        p.velocity += acceleration * dt;
        
        // Apply stronger velocity damping for more stable fluid
        p.velocity *= 0.95f;
        
        // Limit maximum velocity to prevent explosive behavior
        float maxVel = 3.0f;
        if (glm::length(p.velocity) > maxVel) {
            p.velocity = glm::normalize(p.velocity) * maxVel;
        }
        
        // Update position
        p.position += p.velocity * dt;
    }
}

void ParticleSystem::handleBoundaries() {
    // Implement boundary handling
    for (auto& p : particles) {
        // Check left and right boundaries
        if (p.position.x < p.radius) {
            p.position.x = p.radius;
            p.velocity.x *= -RESTITUTION;
        } else if (p.position.x > containerWidth - p.radius) {
            p.position.x = containerWidth - p.radius;
            p.velocity.x *= -RESTITUTION;
        }
        
        // Check bottom and top boundaries
        if (p.position.y < p.radius) {
            p.position.y = p.radius;
            p.velocity.y *= -RESTITUTION;
        } else if (p.position.y > containerHeight - p.radius) {
            p.position.y = containerHeight - p.radius;
            p.velocity.y *= -RESTITUTION;
        }
    }
}

void ParticleSystem::applyMouseForce(glm::vec2 mousePos, bool mousePressed) {
    if (!mousePressed) return;
    
    // Track previous mouse position to calculate velocity
    static glm::vec2 prevMousePos = mousePos;
    glm::vec2 mouseVelocity = (mousePos - prevMousePos) * 5.0f;
    prevMousePos = mousePos;
    
    const float influenceRadius = 0.3f;
    const float maxForce = 1.5f;
    
    for (auto& p : particles) {
        float dist = glm::length(p.position - mousePos);
        
        if (dist < influenceRadius) {
            // Smooth falloff based on distance
            float falloff = 1.0f - (dist / influenceRadius);
            falloff = falloff * falloff; // Quadratic falloff
            
            // Direction from mouse to particle
            glm::vec2 dir = glm::normalize(p.position - mousePos);
            
            // Create a smooth force that's stronger close to the mouse
            float strength = falloff * maxForce;
            
            // Apply a gentler force in the direction of mouse movement
            // plus a slight push away from the mouse position
            glm::vec2 force = mouseVelocity * 0.7f + dir * strength * 0.3f;
            
            // Apply force with dampening to prevent extreme velocities
            p.velocity += force * 0.1f;
            
            // Limit maximum velocity
            float maxVel = 2.0f;
            if (glm::length(p.velocity) > maxVel) {
                p.velocity = glm::normalize(p.velocity) * maxVel;
            }
        }
    }
}

float ParticleSystem::kernelPoly6(float r, float h) {
    // Poly6 kernel for density calculations ***
    if (r >= h) return 0.0f;
    float term = (1.0f - (r * r) / (h * h));
    return term * term * term;
}

float ParticleSystem::kernelSpikyGradient(float r, float h) {
    // Spiky kernel for pressure forces ***
    if (r >= h) return 0.0f;
    float term = (1.0f - r / h);
    return term * term;
}

float ParticleSystem::kernelViscosity(float r, float h) {
    // Implement viscosity kernel ***
    if (r > h) return 0.0f;

    return (1.0f - r / h);

}