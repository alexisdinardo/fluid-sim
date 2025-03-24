#include "cuda_kernels.h"
#include <stdio.h>

// Device memory pointers
CUDAParticle* d_particles = nullptr;
int* d_neighbors = nullptr;
int* d_neighborCounts = nullptr;
int maxNeighbors = 64;  // Maximum number of neighbors per particle

// Initialize CUDA
void initCUDA(int maxParticles) {
    cudaMalloc(&d_particles, maxParticles * sizeof(CUDAParticle));
    cudaMalloc(&d_neighbors, maxParticles * maxNeighbors * sizeof(int));
    cudaMalloc(&d_neighborCounts, maxParticles * sizeof(int));
    
    // Check for errors
    cudaError_t err = cudaGetLastError();
    if (err != cudaSuccess) {
        printf("CUDA initialization error: %s\n", cudaGetErrorString(err));
    }
}

// Clean up CUDA memory
void cleanupCUDA() {
    if (d_particles) cudaFree(d_particles);
    if (d_neighbors) cudaFree(d_neighbors);
    if (d_neighborCounts) cudaFree(d_neighborCounts);
    
    d_particles = nullptr;
    d_neighbors = nullptr;
    d_neighborCounts = nullptr;
}

// Upload particles to GPU
void updateCUDAParticles(CUDAParticle* hostParticles, int numParticles) {
    cudaMemcpy(d_particles, hostParticles, numParticles * sizeof(CUDAParticle), cudaMemcpyHostToDevice);
}

// Download particles from GPU
void downloadCUDAParticles(CUDAParticle* hostParticles, int numParticles) {
    cudaMemcpy(hostParticles, d_particles, numParticles * sizeof(CUDAParticle), cudaMemcpyDeviceToHost);
}

// CUDA kernel implementations for the physics functions
// These are simple placeholder implementations - you would need to expand these
// with the actual SPH algorithm logic

__global__ void findNeighborsKernel(CUDAParticle* particles, int* neighbors, int* neighborCounts, 
                                    int numParticles, float smoothingRadius) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    
    if (i >= numParticles) return;
    
    int count = 0;
    for (int j = 0; j < numParticles; j++) {
        if (i == j) continue;
        
        float2 diff;
        diff.x = particles[i].position.x - particles[j].position.x;
        diff.y = particles[i].position.y - particles[j].position.y;
        
        float distSq = diff.x * diff.x + diff.y * diff.y;
        
        if (distSq < smoothingRadius * smoothingRadius && count < 64) {
            neighbors[i * 64 + count] = j;
            count++;
        }
    }
    
    neighborCounts[i] = count;
}

__global__ void calculateDensityPressureKernel(CUDAParticle* particles, int* neighbors, int* neighborCounts,
                                             int numParticles, float smoothingRadius, 
                                             float particleMass, float restDensity, float pressureConstant) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    
    if (i >= numParticles) return;
    
    // Simple density calculation
    float density = 0.0f;
    int count = neighborCounts[i];
    
    for (int j = 0; j < count; j++) {
        int neighborIdx = neighbors[i * 64 + j];
        
        float2 diff;
        diff.x = particles[i].position.x - particles[neighborIdx].position.x;
        diff.y = particles[i].position.y - particles[neighborIdx].position.y;
        
        float distSq = diff.x * diff.x + diff.y * diff.y;
        float dist = sqrtf(distSq);
        
        if (dist < smoothingRadius) {
            // Simple poly6 kernel
            float term = 1.0f - (dist * dist) / (smoothingRadius * smoothingRadius);
            density += particleMass * term * term * term;
        }
    }
    
    particles[i].density = density;
    
    // Calculate pressure using equation of state
    particles[i].pressure = pressureConstant * (density - restDensity);
    if (particles[i].pressure < 0.0f) particles[i].pressure = 0.0f;
}

__global__ void calculateForcesKernel(CUDAParticle* particles, int* neighbors, int* neighborCounts,
                                    int numParticles, float smoothingRadius, float viscosity, float gravity) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    
    if (i >= numParticles) return;
    
    float2 force;
    force.x = 0.0f;
    force.y = -gravity;  // Apply gravity
    
    int count = neighborCounts[i];
    
    for (int j = 0; j < count; j++) {
        int neighborIdx = neighbors[i * 64 + j];
        
        float2 diff;
        diff.x = particles[i].position.x - particles[neighborIdx].position.x;
        diff.y = particles[i].position.y - particles[neighborIdx].position.y;
        
        float distSq = diff.x * diff.x + diff.y * diff.y;
        float dist = sqrtf(distSq);
        
        if (dist > 0.0001f && dist < smoothingRadius) {
            // Simple pressure and viscosity force calculation
            float2 dir;
            dir.x = diff.x / dist;
            dir.y = diff.y / dist;
            
            // Basic pressure force
            float pressureGradient = (particles[i].pressure + particles[neighborIdx].pressure) / 
                                     (2.0f * particles[neighborIdx].density);
            
            float pressureTerm = pressureGradient * (1.0f - dist / smoothingRadius) * (1.0f - dist / smoothingRadius);
            
            force.x += dir.x * pressureTerm;
            force.y += dir.y * pressureTerm;
            
            // Basic viscosity force
            float2 velDiff;
            velDiff.x = particles[neighborIdx].velocity.x - particles[i].velocity.x;
            velDiff.y = particles[neighborIdx].velocity.y - particles[i].velocity.y;
            
            float viscosityTerm = viscosity * (1.0f - dist / smoothingRadius) / particles[neighborIdx].density;
            
            force.x += velDiff.x * viscosityTerm;
            force.y += velDiff.y * viscosityTerm;
        }
    }
    
    // Limit force magnitude to prevent explosion
    float forceSq = force.x * force.x + force.y * force.y;
    float maxForce = 100.0f;
    
    if (forceSq > maxForce * maxForce) {
        float scale = maxForce / sqrtf(forceSq);
        force.x *= scale;
        force.y *= scale;
    }
    
    particles[i].force = force;
}

__global__ void integrateKernel(CUDAParticle* particles, int numParticles, float deltaTime, 
                             float width, float height, float restitution) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    
    if (i >= numParticles) return;
    
    // Calculate acceleration
    float2 acceleration;
    acceleration.x = particles[i].force.x / fmaxf(particles[i].density, 0.0001f);
    acceleration.y = particles[i].force.y / fmaxf(particles[i].density, 0.0001f);
    
    // Update velocity
    particles[i].velocity.x += acceleration.x * deltaTime;
    particles[i].velocity.y += acceleration.y * deltaTime;
    
    // Apply damping
    particles[i].velocity.x *= 0.998f;
    particles[i].velocity.y *= 0.998f;
    
    // Update position
    particles[i].position.x += particles[i].velocity.x * deltaTime;
    particles[i].position.y += particles[i].velocity.y * deltaTime;
    
    // Handle boundaries
    float radius = particles[i].radius;
    
    if (particles[i].position.x < radius) {
        particles[i].position.x = radius;
        particles[i].velocity.x *= -restitution;
    } else if (particles[i].position.x > width - radius) {
        particles[i].position.x = width - radius;
        particles[i].velocity.x *= -restitution;
    }
    
    if (particles[i].position.y < radius) {
        particles[i].position.y = radius;
        particles[i].velocity.y *= -restitution;
    } else if (particles[i].position.y > height - radius) {
        particles[i].position.y = height - radius;
        particles[i].velocity.y *= -restitution;
    }
}

__global__ void applyMouseForceKernel(CUDAParticle* particles, int numParticles, 
                                    float2 mousePos, bool mousePressed, 
                                    float mouseRadius, float mouseForce) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    
    if (i >= numParticles || !mousePressed) return;
    
    float2 diff;
    diff.x = particles[i].position.x - mousePos.x;
    diff.y = particles[i].position.y - mousePos.y;
    
    float distSq = diff.x * diff.x + diff.y * diff.y;
    float dist = sqrtf(distSq);
    
    if (dist < mouseRadius) {
        // Calculate direction from mouse to particle
        float2 dir;
        if (dist > 0.0001f) {
            dir.x = diff.x / dist;
            dir.y = diff.y / dist;
        } else {
            dir.x = 0.0f;
            dir.y = 1.0f;
        }
        
        // Force strength decreases with distance
        float strength = (1.0f - dist / mouseRadius) * mouseForce;
        
        // Apply force
        particles[i].velocity.x += dir.x * strength;
        particles[i].velocity.y += dir.y * strength;
    }
}

// Wrapper functions to launch the CUDA kernels
void findNeighborsCUDA(int numParticles, float smoothingRadius) {
    int blockSize = 256;
    int numBlocks = (numParticles + blockSize - 1) / blockSize;
    
    findNeighborsKernel<<<numBlocks, blockSize>>>(d_particles, d_neighbors, d_neighborCounts, 
                                                numParticles, smoothingRadius);
    cudaDeviceSynchronize();
}

void calculateDensityPressureCUDA(int numParticles, float smoothingRadius, 
                               float particleMass, float restDensity, 
                               float pressureConstant) {
    int blockSize = 256;
    int numBlocks = (numParticles + blockSize - 1) / blockSize;
    
    calculateDensityPressureKernel<<<numBlocks, blockSize>>>(d_particles, d_neighbors, d_neighborCounts,
                                                         numParticles, smoothingRadius, 
                                                         particleMass, restDensity, pressureConstant);
    cudaDeviceSynchronize();
}

void calculateForcesCUDA(int numParticles, float smoothingRadius, 
                      float viscosity, float gravity) {
    int blockSize = 256;
    int numBlocks = (numParticles + blockSize - 1) / blockSize;
    
    calculateForcesKernel<<<numBlocks, blockSize>>>(d_particles, d_neighbors, d_neighborCounts,
                                                numParticles, smoothingRadius, viscosity, gravity);
    cudaDeviceSynchronize();
}

void integrateCUDA(int numParticles, float deltaTime, 
                float width, float height, float restitution) {
    int blockSize = 256;
    int numBlocks = (numParticles + blockSize - 1) / blockSize;
    
    integrateKernel<<<numBlocks, blockSize>>>(d_particles, numParticles, deltaTime, 
                                           width, height, restitution);
    cudaDeviceSynchronize();
}

void applyMouseForceCUDA(int numParticles, float2 mousePos, 
                      bool mousePressed, float mouseRadius, float mouseForce) {
    int blockSize = 256;
    int numBlocks = (numParticles + blockSize - 1) / blockSize;
    
    applyMouseForceKernel<<<numBlocks, blockSize>>>(d_particles, numParticles, 
                                                mousePos, mousePressed, 
                                                mouseRadius, mouseForce);
    cudaDeviceSynchronize();
}