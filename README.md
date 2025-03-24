# Real-Time Fluid Simulation

A real-time fluid simulation implementation using OpenGL and C++, based on Jos Stam's stable fluids approach. This project demonstrates fluid dynamics through particle-based visualization with interactive controls.

## Overview

This simulation implements a grid-based fluid solver that handles:
- Diffusion of particles
- Advection of velocity fields
- Pressure and viscosity forces
- Real-time user interaction
- Particle visualization with OpenGL

## Features

- Real-time interactive fluid simulation
- Particle-based visualization with customizable parameters
- Stable fluid dynamics using semi-Lagrangian advection
- Pressure solver for incompressibility
- Mouse-based force interaction
- Dynamic particle count adjustment
- Configurable simulation parameters (viscosity, pressure, density)

## Controls

- **Left Mouse Button**: Interact with fluid
- **Space**: Pause/resume simulation
- **R**: Reset simulation
- **G**: Toggle CPU/GPU computation (if CUDA enabled)
- **+/-**: Add/remove particles

## Technical Implementation

The simulation is built on several key components:

1. **Particle System**
   - Grid-based spatial partitioning
   - Density and pressure calculations
   - Velocity field updates

2. **Fluid Dynamics**
   - Semi-Lagrangian advection scheme
   - Pressure projection for incompressibility
   - Viscosity diffusion

3. **Rendering**
   - OpenGL-based particle rendering
   - Dynamic color variation based on velocity
   - Smooth particle transitions

## Building the Project

### Prerequisites
- CMake (3.10 or higher)
- OpenGL
- GLFW
- GLM

### Build Instructions

```bash
mkdir build
cd build
cmake ..
make
```

### Running the Simulation

```bash
./FluidSimulation
```

## Parameters

Key simulation parameters can be adjusted in `include/particle_system.h`:

```cpp
const float GRAVITY = 5.5f;
const float VISCOSITY = 4.5f;
const float PRESSURE_CONSTANT = 5.0f;
const float REST_DENSITY = 45.0f;
const float PARTICLE_MASS = 1.0f;
const float SMOOTHING_RADIUS = 0.08f;
const float RESTITUTION = 0.1f;
```

## Performance

The simulation is optimized for real-time performance:
- Efficient spatial partitioning
- Optimized pressure solver
- Configurable particle count
- Frame time typically under 16ms for 5000 particles

## Future Improvements

- [ ] Multi-grid solver implementation
- [ ] CUDA acceleration support
- [ ] Advanced boundary handling
- [ ] 3D simulation support
- [ ] Enhanced visualization effects
- [ ] Improved pressure-to-displacement mapping

## References

1. Shahrabi, S. (2021). "Gentle Introduction to Fluid Simulation for Programmers and Technical Artists." [Medium Article](https://shahriyarshahrabi.medium.com/gentle-introduction-to-fluid-simulation-for-programmers-and-technical-artists-7c0045c40bac)

2. Stam, J. (1999). "Stable Fluids." In Proceedings of SIGGRAPH 99, Annual Conference Series, 121-128. [Research Paper](https://www.researchgate.net/publication/2486965_Stable_Fluids)
