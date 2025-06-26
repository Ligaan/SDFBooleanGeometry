#pragma once
#include "glad/glad.h"
#include "glm.hpp"
#include "gtc/epsilon.hpp"
#include <vector>
#include <unordered_map>

struct Mesh {
    GLuint VAO;
    GLuint VBO;
    GLuint EBO;
    GLsizei indexCount;
    std::vector<float> vertices;
    std::vector<unsigned int> indices;
};

struct Vec3Hash {
    size_t operator()(const glm::vec3& v) const {
        size_t hx = std::hash<float>()(v.x);
        size_t hy = std::hash<float>()(v.y);
        size_t hz = std::hash<float>()(v.z);
        return hx ^ (hy << 1) ^ (hz << 2);
    }
};

struct Vec3Equal {
    bool operator()(const glm::vec3& a, const glm::vec3& b) const {
        return glm::all(glm::epsilonEqual(a, b, 1e-6f));
    }
};

struct Face {
    std::vector<glm::vec3> facePoints;
    glm::vec3 normal;
    std::vector<unsigned int> indeces;
};

class Shapes
{
public:
    static void ExtractUniquePositionsAndIndices(
        const Mesh& mesh,
        std::vector<glm::vec3>& outPositions,
        std::vector<unsigned int>& outIndices
    );
    static void ExtractUniquePositionsAndIndicesWorld(const Mesh& mesh, std::vector<glm::vec3>& outPositions, std::vector<unsigned int>& outIndices, const glm::mat4& model);
    static Mesh CreateSphere(float radius, unsigned int sectorCount, unsigned int stackCount, glm::vec3 color);
    static Mesh CreateBox(float width, float height, float length, glm::vec3 color);
    static Mesh CreateCylinder(float radius, float height, unsigned int sectorCount, glm::vec3 color);
    static Mesh FaceToMesh(Face& face, glm::vec3 color);
    static Mesh OpenGLDataInitialize(std::vector<float>& vertices, std::vector<unsigned int>& indices);
    static void ProjectOntoAxis(
        const std::vector<float>& vertices,
        const glm::vec3& axis,
        const glm::mat4& modelMatrix,
        float& min,
        float& max
    );
    static bool AreMeshesIntersectingSAT(
        const Mesh& meshA, const glm::mat4& modelA,
        const Mesh& meshB, const glm::mat4& modelB
    );
    static std::vector<glm::vec3> CalculateFaceNormals(const Mesh& mesh, const glm::mat4& modelMatrix);
};

