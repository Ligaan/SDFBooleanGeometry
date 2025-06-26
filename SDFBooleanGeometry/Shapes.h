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
    std::vector<double> vertices;
    std::vector<unsigned int> indices;
};

struct Vec3Hash {
    size_t operator()(const glm::dvec3& v) const {
        size_t hx = std::hash<double>()(v.x);
        size_t hy = std::hash<double>()(v.y);
        size_t hz = std::hash<double>()(v.z);
        return hx ^ (hy << 1) ^ (hz << 2);
    }
};

struct Vec3Equal {
    bool operator()(const glm::dvec3& a, const glm::dvec3& b) const {
        return glm::all(glm::epsilonEqual(a, b, 1e-6));
    }
};

struct Face {
    std::vector<glm::dvec3> facePoints;
    glm::dvec3 normal;
    std::vector<unsigned int> indeces;
};

class Shapes
{
public:
    static void ExtractUniquePositionsAndIndices(
        const Mesh& mesh,
        std::vector<glm::dvec3>& outPositions,
        std::vector<unsigned int>& outIndices
    );
    static void ExtractUniquePositionsAndIndicesWorld(const Mesh& mesh, std::vector<glm::dvec3>& outPositions, std::vector<unsigned int>& outIndices, const glm::dmat4& model);
    static Mesh CreateSphere(double radius, unsigned int sectorCount, unsigned int stackCount, glm::dvec3 color);
    static Mesh CreateBox(double width, double height, double length, glm::dvec3 color);
    static Mesh CreateCylinder(double radius, double height, unsigned int sectorCount, glm::dvec3 color);
    static Mesh FaceToMesh(Face& face, glm::dvec3 color);
    static Mesh OpenGLDataInitialize(std::vector<double>& vertices, std::vector<unsigned int>& indices);
    static void ProjectOntoAxis(
        const std::vector<double>& vertices,
        const glm::dvec3& axis,
        const glm::dmat4& modelMatrix,
        double& min,
        double& max
    );
    static bool AreMeshesIntersectingSAT(
        const Mesh& meshA, const glm::dmat4& modelA,
        const Mesh& meshB, const glm::dmat4& modelB
    );
    static std::vector<glm::dvec3> CalculateFaceNormals(const Mesh& mesh, const glm::dmat4& modelMatrix);
};

