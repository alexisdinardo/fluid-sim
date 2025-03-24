#ifndef CUDA_KERNELS_H
#define CUDA_KERNELS_H

#include <cuda_runtime.h>
#include <device_launch_parameters.h>

// Struct for particle data on GPU
struct CUDAParticle {
    float2 position;
    float2 velocity;
    float2 force;
    float density;
    float pressure;
    float radius;
};

// *** TODO: Function declarations for CUDA kernels ***
void initCUDA(int maxParticles);
void cleanupCUDA();

void updateCUDAParticles(CUDAParticle* hostParticles, int numParticles);
void downloadCUDAParticles(CUDAParticle* hostParticles, int numParticles);

void findNeighborsCUDA(int numParticles, float smoothingRadius);
void calculateDensityPressureCUDA(int numParticles, float smoothingRadius, 
                               float particleMass, float restDensity, 
                               float pressureConstant);
void calculateForcesCUDA(int numParticles, float smoothingRadius, 
                      float viscosity, float gravity);
void integrateCUDA(int numParticles, float deltaTime, 
                float width, float height, float restitution);
void applyMouseForceCUDA(int numParticles, float2 mousePos, 
                      bool mousePressed, float mouseRadius, float mouseForce);

#endif