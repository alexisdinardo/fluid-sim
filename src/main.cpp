#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <iostream>
#include <vector>
#include <random>
#include <chrono>

#include "../include/shader.h"
#include "../include/particle_system.h"

// Window dimensions
const unsigned int SCR_WIDTH = 1280;
const unsigned int SCR_HEIGHT = 720;

// Function prototypes
GLFWwindow* initializeWindow();
void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void mouse_button_callback(GLFWwindow* window, int button, int action, int mods);
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);
void processInput(GLFWwindow* window);

// Global variables for mouse interaction
bool mousePressed = false;
glm::vec2 mousePos(0.0f);
float mouseRadius = 0.2f;
float mouseForce = 5.0f;

// Global variables for simulation control
bool pauseSimulation = false;
bool useCPU = true;  // Default to CPU implementation

// Particle system
ParticleSystem* particleSystem = nullptr;

// Timing
float deltaTime = 0.0f;
float lastFrame = 0.0f;

int main() {
    std::cout << "Starting Fluid Simulation..." << std::endl;
    
    // Initialize window
    GLFWwindow* window = initializeWindow();
    if (!window) {
        std::cerr << "Failed to create window!" << std::endl;
        return -1;
    }
    
    std::cout << "Window created successfully!" << std::endl;
    
    // Create shader program for rendering particles
    std::cout << "Loading shaders..." << std::endl;
    Shader particleShader("shaders/particle.vert", "shaders/particle.frag");
    std::cout << "Shaders loaded successfully!" << std::endl;
    
    // Initialize particle system
    std::cout << "Initializing particle system..." << std::endl;
    particleSystem = new ParticleSystem(5000, 2.0f, 2.0f * SCR_HEIGHT / SCR_WIDTH);
    std::cout << "Particle system initialized with " << particleSystem->getParticleCount() << " particles" << std::endl;
    
    // Print instructions
    std::cout << "\nControls:" << std::endl;
    std::cout << "  Left mouse button: Interact with fluid" << std::endl;
    std::cout << "  Space: Pause/resume simulation" << std::endl;
    std::cout << "  R: Reset simulation" << std::endl;
    std::cout << "  G: Toggle CPU/GPU computation (if CUDA is enabled)" << std::endl;
    std::cout << "  +/-: Add/remove particles" << std::endl;
    std::cout << "\nStarting simulation loop..." << std::endl;
    
    // Main rendering loop
    while (!glfwWindowShouldClose(window)) {
        // Calculate delta time
        float currentFrame = static_cast<float>(glfwGetTime());
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;
        
        // Limit delta time to prevent instability
        deltaTime = std::min(deltaTime, 0.016f);  // Cap at ~60 FPS
        
        // Process input
        processInput(window);
        
        // Convert mouse position from screen space to simulation space
        glm::vec2 simMousePos = glm::vec2(
            mousePos.x / SCR_WIDTH * 2.0f,
            (SCR_HEIGHT - mousePos.y) / SCR_HEIGHT * 2.0f * SCR_HEIGHT / SCR_WIDTH
        );
        
        // Update simulation if not paused
        if (!pauseSimulation) {
            particleSystem->update(deltaTime, simMousePos, mousePressed);
        }
        
        // Clear the screen
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        
        // Setup projection matrix (orthographic for 2D)
        glm::mat4 projection = glm::ortho(0.0f, 2.0f, 0.0f, 2.0f * SCR_HEIGHT / SCR_WIDTH);
        glm::mat4 view = glm::mat4(1.0f);
        
        // Set shader uniforms
        particleShader.use();
        particleShader.setMat4("projection", projection);
        particleShader.setMat4("view", view);
        
        // Render the particle system
        particleSystem->render(particleShader);
        
        // Swap buffers and poll events
        glfwSwapBuffers(window);
        glfwPollEvents();
    }
    
    std::cout << "Cleaning up..." << std::endl;
    
    // Cleanup
    delete particleSystem;
    
    glfwTerminate();
    std::cout << "Simulation ended successfully!" << std::endl;
    return 0;
}

GLFWwindow* initializeWindow() {
    // Initialize GLFW
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return nullptr;
    }
    
    // Configure GLFW
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif
    
    // Create window
    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "Fluid Simulation", NULL, NULL);
    if (!window) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return nullptr;
    }
    
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetCursorPosCallback(window, mouse_callback);
    glfwSetMouseButtonCallback(window, mouse_button_callback);
    glfwSetKeyCallback(window, key_callback);
    
    // Initialize GLAD
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "Failed to initialize GLAD" << std::endl;
        return nullptr;
    }
    
    // Configure OpenGL
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_PROGRAM_POINT_SIZE);
    
    return window;
}

// *** TODO: Implement callback functions ***
void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
}

void mouse_callback(GLFWwindow* window, double xpos, double ypos) {
    mousePos.x = static_cast<float>(xpos);
    mousePos.y = static_cast<float>(ypos);
}

void mouse_button_callback(GLFWwindow* window, int button, int action, int mods) {
    if (button == GLFW_MOUSE_BUTTON_LEFT) {
        if (action == GLFW_PRESS) {
            mousePressed = true;
        } else if (action == GLFW_RELEASE) {
            mousePressed = false;
        }
    }
}

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (action == GLFW_PRESS) {
        switch (key) {
            case GLFW_KEY_SPACE:
                pauseSimulation = !pauseSimulation;
                break;
            case GLFW_KEY_R:
                // Reset the particle system
                particleSystem->reset();
                std::cout << "Simulation reset" << std::endl;
                break;
            case GLFW_KEY_G:
                useCPU = !useCPU;
                std::cout << "Using " << (useCPU ? "CPU" : "GPU") << " implementation" << std::endl;
                break;
            case GLFW_KEY_EQUAL:  // +
                // Add particles
                particleSystem->addParticles(100);
                break;
            case GLFW_KEY_MINUS:  // -
                // Remove particles
                particleSystem->removeParticles(100);
                break;
        }
    }
}

void processInput(GLFWwindow* window) {
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);
}