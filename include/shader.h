#ifndef SHADER_H
#define SHADER_H

#ifdef __APPLE__
#define GL_SILENCE_DEPRECATION
#endif

#include <glad/glad.h>
#include <glm/glm.hpp>

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

class Shader {
public:
    // Program ID
    unsigned int ID;
    
    // Constructor
    Shader(const char* vertexPath, const char* fragmentPath) {
        // *** TODO: Implement shader loading and compilation ***
        // 1. Retrieve vertex/fragment source code from file paths
        std::string vertexCode;
        std::string fragmentCode;
        std::ifstream vShaderFile;
        std::ifstream fShaderFile;
        
        // Ensure ifstream objects can throw exceptions
        vShaderFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);
        fShaderFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);
        
        try {
            // Open files
            vShaderFile.open(vertexPath);
            fShaderFile.open(fragmentPath);
            
            // Read file's buffer contents into streams
            std::stringstream vShaderStream, fShaderStream;
            vShaderStream << vShaderFile.rdbuf();
            fShaderStream << fShaderFile.rdbuf();
            
            // Close file handlers
            vShaderFile.close();
            fShaderFile.close();
            
            // Convert stream into string
            vertexCode = vShaderStream.str();
            fragmentCode = fShaderStream.str();
        }
        catch (std::ifstream::failure& e) {
            std::cerr << "ERROR::SHADER::FILE_NOT_SUCCESSFULLY_READ: " << e.what() << std::endl;
        }
        
        const char* vShaderCode = vertexCode.c_str();
        const char* fShaderCode = fragmentCode.c_str();
        
        // 2. Compile shaders
        unsigned int vertex, fragment;
        
        // Vertex shader
        vertex = glCreateShader(GL_VERTEX_SHADER);
        glShaderSource(vertex, 1, &vShaderCode, NULL);
        glCompileShader(vertex);
        checkCompileErrors(vertex, "VERTEX");
        
        // Fragment shader
        fragment = glCreateShader(GL_FRAGMENT_SHADER);
        glShaderSource(fragment, 1, &fShaderCode, NULL);
        glCompileShader(fragment);
        checkCompileErrors(fragment, "FRAGMENT");
        
        // Shader program
        ID = glCreateProgram();
        glAttachShader(ID, vertex);
        glAttachShader(ID, fragment);
        glLinkProgram(ID);
        checkCompileErrors(ID, "PROGRAM");
        
        // Delete shaders as they're linked into the program and no longer necessary
        glDeleteShader(vertex);
        glDeleteShader(fragment);
    }
    
    // Use/activate the shader
    void use() {
        glUseProgram(ID);
    }
    
    // Utility uniform functions
    void setBool(const std::string &name, bool value) const {
        // Cast to float because we don't have glUniform1i
        glUniform1f(glGetUniformLocation(ID, name.c_str()), value ? 1.0f : 0.0f);
    }
    
    void setInt(const std::string &name, int value) const {
        // Cast to float because we don't have glUniform1i
        glUniform1f(glGetUniformLocation(ID, name.c_str()), static_cast<float>(value));
    }
    
    void setFloat(const std::string &name, float value) const {
        glUniform1f(glGetUniformLocation(ID, name.c_str()), value);
    }
    
    void setVec2(const std::string &name, const glm::vec2 &value) const {
        // Use individual component setter instead of vector version
        glUniform2f(glGetUniformLocation(ID, name.c_str()), value.x, value.y);
    }
    
    void setVec3(const std::string &name, const glm::vec3 &value) const {
        // Use individual component setter instead of vector version
        glUniform3f(glGetUniformLocation(ID, name.c_str()), value.x, value.y, value.z);
    }
    
    void setVec4(const std::string &name, const glm::vec4 &value) const {
        // Use individual component setter instead of vector version
        glUniform4f(glGetUniformLocation(ID, name.c_str()), value.x, value.y, value.z, value.w);
    }
    
    void setMat4(const std::string &name, const glm::mat4 &mat) const {
        glUniformMatrix4fv(glGetUniformLocation(ID, name.c_str()), 1, GL_FALSE, &mat[0][0]);
    }
    
private:
    // Utility function for checking shader compilation/linking errors
    void checkCompileErrors(unsigned int shader, std::string type) {
        int success;
        char infoLog[1024];
        
        if (type != "PROGRAM") {
            glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
            if (!success) {
                glGetShaderInfoLog(shader, 1024, NULL, infoLog);
                std::cerr << "ERROR::SHADER_COMPILATION_ERROR of type: " << type << "\n" << infoLog 
                          << "\n -- --------------------------------------------------- -- " << std::endl;
            }
        }
        else {
            glGetProgramiv(shader, GL_LINK_STATUS, &success);
            if (!success) {
                glGetProgramInfoLog(shader, 1024, NULL, infoLog);
                std::cerr << "ERROR::PROGRAM_LINKING_ERROR of type: " << type << "\n" << infoLog 
                          << "\n -- --------------------------------------------------- -- " << std::endl;
            }
        }
    }
};

#endif