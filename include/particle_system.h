#ifndef PARTICLE_SYSTEM_H
#define PARTICLE_SYSTEM_H

#ifdef __APPLE__
#define GL_SILENCE_DEPRECATION
#endif

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <vector>
#include <random>
#include "shader.h"

// Simulation constants
const float GRAVITY = 5.5f;           // Reduced gravity for slower movement
const float VISCOSITY = 4.5f;         // Increased viscosity for more cohesion
const float PRESSURE_CONSTANT = 5.0f;  // Increased pressure for stronger particle interaction
const float REST_DENSITY = 45.0f;     // Significantly increased rest density for much tighter particle packing
const float PARTICLE_MASS = 1.0f;
const float SMOOTHING_RADIUS = 0.08f;  // Reduced radius for tighter clustering
const float RESTITUTION = 0.1f;       // Very low restitution for minimal bouncing

// Particle struct
struct Particle {
    glm::vec2 position;
    glm::vec2 velocity;
    glm::vec2 force;
    float density;
    float pressure;
    glm::vec4 color;
    float radius;
    std::vector<int> neighbors;
    
    // Default constructor (required for std::vector::resize)
    Particle() 
        : position(0.0f), velocity(0.0f), force(0.0f),
          density(0.0f), pressure(0.0f), color(1.0f, 1.0f, 1.0f, 1.0f), 
          radius(0.01f), neighbors() {}
    
    Particle(glm::vec2 pos, glm::vec4 col, float rad) 
        : position(pos), velocity(glm::vec2(0.0f)), force(glm::vec2(0.0f)),
          density(0.0f), pressure(0.0f), color(col), radius(rad), neighbors() {}
};

class ParticleSystem {
public:
    ParticleSystem(int maxParticles, float width, float height);
    ~ParticleSystem();
    
    void update(float dt, glm::vec2 mousePos, bool mousePressed);
    void render(Shader &shader);
    void reset();
    void addParticles(int count);
    void removeParticles(int count);
    size_t getParticleCount() const { return particles.size(); }
    
private:
    std::vector<Particle> particles;
    float containerWidth;
    float containerHeight;
    bool useCUDA = false;  // Set to false by default, can be enabled if CUDA is available
    
    // OpenGL objects
    GLuint VAO, VBO;
    
    // Particle data for rendering
    struct ParticleData {
        glm::vec2 position;
        glm::vec4 color;
        float radius;
    };
    
    // Physics methods
    void findNeighbors();
    void calculateDensityPressure();
    void calculateForces();
    void integrate(float dt);
    void handleBoundaries();
    void applyMouseForce(glm::vec2 mousePos, bool mousePressed);
    
    // SPH Kernel functions
    float kernelPoly6(float r, float h);
    float kernelSpikyGradient(float r, float h);
    float kernelViscosity(float r, float h);
    
    // Spatial partitioning
    const int GRID_SIZE = 20;
    float cellSize;
    std::vector<std::vector<int>> grid;
    void updateGrid();
    
    // Initialize rendering data
    void setupBuffers();
    
    // Random number generation
    std::mt19937 rng;
};

#endif