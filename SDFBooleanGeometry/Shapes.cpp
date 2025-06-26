
#include "Shapes.h"
#include <array>
#include <gtc/matrix_transform.hpp>
#include <gtc/type_ptr.hpp>
#include <unordered_set>
#include <limits>

#include <string>
#define GLM_ENABLE_EXPERIMENTAL
#include "gtx/string_cast.hpp"
#include <iostream>
#include <algorithm>
#include <cmath>

void DebugPrintTriangleNormals(const std::vector<glm::dvec3>& points, const std::vector<unsigned int>& indices, glm::dvec3 normalT) {
    std::cout << glm::to_string(normalT) << "\n\n";
    for (size_t i = 0; i + 2 < indices.size(); i += 3) {
        glm::dvec3 v0 = points[indices[i]];
        glm::dvec3 v1 = points[indices[i + 1]];
        glm::dvec3 v2 = points[indices[i + 2]];

        glm::dvec3 normal = glm::normalize(glm::cross(v1 - v0, v2 - v0));

        std::cout << glm::to_string(normal) << "\n";
        std::cout << glm::to_string(v0) << "\n";
        std::cout << glm::to_string(v1) << "\n";
        std::cout << glm::to_string(v2) << "\n\n";
    }
}

void BuildVertexBufferFromPositionsAndIndices(
    const std::vector<glm::dvec3>& positions,
    const std::vector<unsigned int>& indices,
    std::vector<double>& outVertexBuffer,
    const glm::dvec3& color)
{
    const size_t vertexCount = positions.size();
    std::vector<glm::dvec3> normals(vertexCount, glm::dvec3(0.0f));

    // Step 1: Accumulate triangle normals per vertex
    for (size_t i = 0; i < indices.size(); i += 3) {
        unsigned int i0 = indices[i];
        unsigned int i1 = indices[i + 1];
        unsigned int i2 = indices[i + 2];

        const glm::dvec3& v0 = positions[i0];
        const glm::dvec3& v1 = positions[i1];
        const glm::dvec3& v2 = positions[i2];

        glm::dvec3 normal = glm::normalize(glm::cross(v1 - v0, v2 - v0));

        normals[i0] += normal;
        normals[i1] += normal;
        normals[i2] += normal;
    }

    // Step 2: Normalize the accumulated normals
    for (glm::dvec3& n : normals) {
        n = glm::normalize(n);
    }

    // Step 3: Build the vertex buffer (pos + normal + color)
    outVertexBuffer.clear();
    outVertexBuffer.reserve(vertexCount * 9);

    for (size_t i = 0; i < vertexCount; ++i) {
        const glm::dvec3& pos = positions[i];
        const glm::dvec3& norm = normals[i];

        // Position
        outVertexBuffer.push_back(pos.x);
        outVertexBuffer.push_back(pos.y);
        outVertexBuffer.push_back(pos.z);

        // Normal
        outVertexBuffer.push_back(norm.x);
        outVertexBuffer.push_back(norm.y);
        outVertexBuffer.push_back(norm.z);

        // Color
        outVertexBuffer.push_back(color.r);
        outVertexBuffer.push_back(color.g);
        outVertexBuffer.push_back(color.b);
    }
}

bool LineIntersectsTriangle2(
    const glm::dvec3& p0, const glm::dvec3& p1,
    const glm::dvec3& v0, const glm::dvec3& v1, const glm::dvec3& v2,
    glm::dvec3& intersectionStart, glm::dvec3& intersectionEnd,
    bool& isSegmentIntersection)
{
    const glm::dvec3 dir = p1 - p0;
    const glm::dvec3 e1 = v1 - v0;
    const glm::dvec3 e2 = v2 - v0;

    const glm::dvec3 normal = glm::normalize(glm::cross(e1, e2));
    const double denom = glm::dot(normal, dir);

    // Check if line and triangle are parallel
    if (fabs(denom) < std::numeric_limits<double>::epsilon()) {
        // Coplanar check
        const double dist = glm::dot(normal, p0 - v0);
        if (fabs(dist) > std::numeric_limits<double>::epsilon()) {
            return false; // Parallel but not coplanar
        }

        // Segment and triangle are coplanar – check for 2D overlap
        // Project onto best 2D plane
        int axis = 0;
        glm::dvec3 n = glm::abs(normal);
        if (n.y > n.x) axis = 1;
        if (n.z > n[axis]) axis = 2;

        auto project2D = [axis](const glm::dvec3& p) -> glm::dvec2 {
            switch (axis) {
            case 0: return glm::dvec2(p.y, p.z); // drop x
            case 1: return glm::dvec2(p.x, p.z); // drop y
            case 2: return glm::dvec2(p.x, p.y); // drop z
            }
            return glm::dvec2(0); // should never happen
            };

        glm::dvec2 segA = project2D(p0);
        glm::dvec2 segB = project2D(p1);
        glm::dvec2 tri0 = project2D(v0);
        glm::dvec2 tri1 = project2D(v1);
        glm::dvec2 tri2 = project2D(v2);

        // Check if either endpoint is inside the triangle
        auto PointInTri = [](const glm::dvec2& pt, const glm::dvec2& a, const glm::dvec2& b, const glm::dvec2& c) {
            auto sign = [](const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& p3) {
                return (p1.x - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p3.y);
                };
            double d1 = sign(pt, a, b);
            double d2 = sign(pt, b, c);
            double d3 = sign(pt, c, a);
            bool hasNeg = (d1 < 0) || (d2 < 0) || (d3 < 0);
            bool hasPos = (d1 > 0) || (d2 > 0) || (d3 > 0);
            return !(hasNeg && hasPos);
            };

        std::vector<glm::dvec3> insidePoints;
        if (PointInTri(segA, tri0, tri1, tri2)) insidePoints.push_back(p0);
        if (PointInTri(segB, tri0, tri1, tri2)) insidePoints.push_back(p1);

        // Also check for segment-triangle edge intersections in 2D
        auto SegmentIntersect = [](glm::dvec2 p, glm::dvec2 r, glm::dvec2 q, glm::dvec2 s, glm::dvec2& out) -> bool {
            double rxs = r.x * s.y - r.y * s.x;
            if (fabs(rxs) < std::numeric_limits<double>::epsilon()) return false; // parallel
            glm::dvec2 qp = q - p;
            double t = (qp.x * s.y - qp.y * s.x) / rxs;
            double u = (qp.x * r.y - qp.y * r.x) / rxs;
            if (t >= 0 && t <= 1 && u >= 0 && u <= 1) {
                out = p + t * r;
                return true;
            }
            return false;
            };

        glm::dvec2 segVec = segB - segA;
        std::vector<glm::dvec2> triEdges[3] = {
            {tri0, tri1},
            {tri1, tri2},
            {tri2, tri0}
        };

        for (auto& edge : triEdges) {
            glm::dvec2 ip;
            if (SegmentIntersect(segA, segVec, edge[0], edge[1] - edge[0], ip)) {
                glm::dvec3 full = p0 + dir * glm::length(ip - segA) / glm::length(segVec);
                insidePoints.push_back(full);
            }
        }

        if (insidePoints.size() >= 2) {
            intersectionStart = insidePoints[0];
            intersectionEnd = insidePoints[1];
            isSegmentIntersection = true;
            return true;
        }
        else if (insidePoints.size() == 1) {
            intersectionStart = insidePoints[0];
            intersectionEnd = insidePoints[0];
            isSegmentIntersection = false;
            return true;
        }

        return false;
    }

    // Not coplanar – use Möller–Trumbore for intersection point
    const glm::dvec3 h = glm::cross(dir, e2);
    const double a = glm::dot(e1, h);
    const double f = 1.0f / a;
    const glm::dvec3 s = p0 - v0;
    const double u = f * glm::dot(s, h);
    if (u < 0.0f || u > 1.0f) return false;

    const glm::dvec3 q = glm::cross(s, e1);
    const double v = f * glm::dot(dir, q);
    if (v < 0.0f || u + v > 1.0f) return false;

    const double t = f * glm::dot(e2, q);
    if (t < 0.0f || t > 1.0f) return false;

    intersectionStart = p0 + t * dir;
    intersectionEnd = intersectionStart;
    isSegmentIntersection = false;
    return true;
}

glm::dvec3 CalculateCentroid(const std::vector<glm::dvec3>& points) {
    glm::dvec3 centroid(0.0f);
    for (const auto& point : points) {
        centroid += point;
    }
    return centroid / static_cast<double>(points.size());
}

// Function to calculate the angle between the point and the centroid
double AngleBetweenPoints(const glm::dvec3& point, const glm::dvec3& centroid) {
    glm::dvec2 diff = glm::dvec2(point.x - centroid.x, point.y - centroid.y);
    return std::atan2(diff.y, diff.x); // Use atan2 to get angle in 2D
}

// Sort the points by angle relative to the centroid
void SortPointsByAngle(std::vector<glm::dvec3>& points) {
    glm::dvec3 centroid = CalculateCentroid(points);
    if (points.size() < 3)
        return;
    // Sort the points based on their angle relative to the centroid
    std::sort(points.begin(), points.end(), [&centroid](const glm::dvec3& a, const glm::dvec3& b) {
        return AngleBetweenPoints(a, centroid) < AngleBetweenPoints(b, centroid);
        });
    points.push_back(centroid);
}

void SortPointsOnPlaneByAngle(std::vector<glm::dvec3>& points, const glm::dvec3& normal) {
    glm::dvec3 centroid = CalculateCentroid(points);

    // Create a 2D basis on the plane
    glm::dvec3 refAxis = glm::normalize(glm::cross(normal, glm::dvec3(0, 1, 0)));
    if (glm::length(refAxis) < 0.01f)
        refAxis = glm::normalize(glm::cross(normal, glm::dvec3(1, 0, 0)));
    glm::dvec3 upAxis = glm::normalize(glm::cross(normal, refAxis));

    std::sort(points.begin(), points.end(), [&](const glm::dvec3& a, const glm::dvec3& b) {
        glm::dvec3 da = a - centroid;
        glm::dvec3 db = b - centroid;

        double angleA = atan2(glm::dot(da, upAxis), glm::dot(da, refAxis));
        double angleB = atan2(glm::dot(db, upAxis), glm::dot(db, refAxis));

        return angleA < angleB;
        });
}

////////////////////////////////
void Shapes::ExtractUniquePositionsAndIndices(const Mesh& mesh, std::vector<glm::dvec3>& outPositions, std::vector<unsigned int>& outIndices)
{
    std::unordered_map<glm::dvec3, unsigned int, Vec3Hash, Vec3Equal> positionToIndex;
    outPositions.clear();
    outIndices.clear();

    for (size_t i = 0; i < mesh.indices.size(); ++i) {
        unsigned int originalIndex = mesh.indices[i];
        glm::dvec3 position(
            mesh.vertices[originalIndex * 9 + 0],
            mesh.vertices[originalIndex * 9 + 1],
            mesh.vertices[originalIndex * 9 + 2]
        );

        if (positionToIndex.count(position) == 0) {
            // New unique position
            unsigned int newIndex = static_cast<unsigned int>(outPositions.size());
            outPositions.push_back(position);
            positionToIndex[position] = newIndex;
        }

        outIndices.push_back(positionToIndex[position]);
    }
}

void Shapes::ExtractUniquePositionsAndIndicesWorld(const Mesh& mesh, std::vector<glm::dvec3>& outPositions, std::vector<unsigned int>& outIndices, const glm::dmat4& model)
{
    std::unordered_map<glm::dvec3, unsigned int, Vec3Hash, Vec3Equal> positionToIndex;
    outPositions.clear();
    outIndices.clear();

    for (size_t i = 0; i < mesh.indices.size(); ++i) {
        unsigned int originalIndex = mesh.indices[i];

        glm::dvec3 localPosition(
            mesh.vertices[originalIndex * 9 + 0],
            mesh.vertices[originalIndex * 9 + 1],
            mesh.vertices[originalIndex * 9 + 2]
        );

        glm::dvec3 worldPosition = glm::dvec3(model * glm::vec4(localPosition, 1.0f));

        auto it = positionToIndex.find(worldPosition);
        if (it == positionToIndex.end()) {
            // New unique position
            unsigned int newIndex = static_cast<unsigned int>(outPositions.size());
            outPositions.push_back(worldPosition);
            positionToIndex[worldPosition] = newIndex;
            outIndices.push_back(newIndex);
        }
        else {
            outIndices.push_back(it->second);
        }
    }
}


Mesh Shapes::CreateSphere(double radius, unsigned int sectorCount, unsigned int stackCount, glm::dvec3 color) {
    std::vector<double> vertices;
    std::vector<unsigned int> indices;
    sectorCount -= 1;
    stackCount -= 1;
    for (unsigned int i = 0; i <= stackCount; ++i) {
        double stackAngle = glm::pi<double>() / 2 - i * glm::pi<double>() / stackCount; // from pi/2 to -pi/2
        double xy = radius * cosf(stackAngle); // r * cos(phi)
        double z = radius * sinf(stackAngle);  // r * sin(phi)

        for (unsigned int j = 0; j <= sectorCount; ++j) {
            double sectorAngle = j * 2 * glm::pi<double>() / sectorCount; // from 0 to 2pi
            double x = xy * cosf(sectorAngle);
            double y = xy * sinf(sectorAngle);

            // Vertex position
            vertices.push_back(x);
            vertices.push_back(y);
            vertices.push_back(z);

            // Normalized normal (for a sphere centered at origin, normal = position normalized)
            glm::dvec3 normal = glm::normalize(glm::dvec3(x, y, z));
            vertices.push_back(normal.x);
            vertices.push_back(normal.y);
            vertices.push_back(normal.z);

            // Vertex color
            vertices.push_back(color.r);
            vertices.push_back(color.g);
            vertices.push_back(color.b);
        }
    }

    // Indices
    for (unsigned int i = 0; i < stackCount; ++i) {
        unsigned int k1 = i * (sectorCount + 1);
        unsigned int k2 = k1 + sectorCount + 1;

        for (unsigned int j = 0; j < sectorCount; ++j, ++k1, ++k2) {
            if (i != 0) {
                indices.push_back(k1);
                indices.push_back(k2);
                indices.push_back(k1 + 1);
            }

            if (i != (stackCount - 1)) {
                indices.push_back(k1 + 1);
                indices.push_back(k2);
                indices.push_back(k2 + 1);
            }
        }
    }

    return OpenGLDataInitialize(vertices, indices);
}

Mesh Shapes::CreateBox(double width, double height, double length, glm::dvec3 color) {
    std::vector<double> vertices;
    std::vector<unsigned int> indices;

    double w = width / 2.0f;
    double h = height / 2.0f;
    double l = length / 2.0f;

    struct Face {
        glm::dvec3 normal;
        glm::dvec3 v0, v1, v2, v3;
    };

    std::vector<Face> faces = {
        // Front face
        {{0, 0, 1},  {-w, -h,  l},  { w, -h,  l},  { w,  h,  l},  {-w,  h,  l}},
        // Back face
        {{0, 0, -1}, {-w, -h, -l},  {-w,  h, -l},  { w,  h, -l},  { w, -h, -l}},
        // Left face
        {{-1, 0, 0}, {-w, -h, -l},  {-w, -h,  l},  {-w,  h,  l},  {-w,  h, -l}},
        // Right face
        {{1, 0, 0},  { w, -h, -l},  { w,  h, -l},  { w,  h,  l},  { w, -h,  l}},
        // Top face
        {{0, 1, 0},  {-w,  h, -l},  {-w,  h,  l},  { w,  h,  l},  { w,  h, -l}},
        // Bottom face
        {{0, -1, 0}, {-w, -h, -l},  { w, -h, -l},  { w, -h,  l},  {-w, -h,  l}},
    };

    unsigned int index = 0;
    for (const auto& face : faces) {
        std::array<unsigned int, 6> faceIndices = {
            index, index + 1, index + 2,
            index, index + 2, index + 3
        };

        for (int i = 0; i < 6; ++i)
            indices.push_back(faceIndices[i]);

        std::vector<glm::dvec3> corners = { face.v0, face.v1, face.v2, face.v3 };
        for (const auto& v : corners) {
            // Position
            vertices.push_back(v.x);
            vertices.push_back(v.y);
            vertices.push_back(v.z);

            // Normal
            vertices.push_back(face.normal.x);
            vertices.push_back(face.normal.y);
            vertices.push_back(face.normal.z);

            // Color
            vertices.push_back(color.r);
            vertices.push_back(color.g);
            vertices.push_back(color.b);
        }

        index += 4;
    }

    return OpenGLDataInitialize(vertices, indices);
}

Mesh Shapes::CreateCylinder(double radius, double height, unsigned int sectorCount, glm::dvec3 color) {
    std::vector<double> vertices;
    std::vector<unsigned int> indices;

    double halfHeight = height / 2.0f;
    double sectorStep = 2 * glm::pi<double>() / sectorCount;

    // Side surface
    for (unsigned int i = 0; i <= sectorCount; ++i) {
        double angle = i * sectorStep;
        double x = cos(angle);
        double y = sin(angle);
        glm::dvec3 normal = glm::normalize(glm::dvec3(x, y, 0.0f));

        // Bottom vertex
        vertices.insert(vertices.end(), {
            radius * x, radius * y, -halfHeight,
            normal.x, normal.y, normal.z,
            color.r, color.g, color.b
            });

        // Top vertex
        vertices.insert(vertices.end(), {
            radius * x, radius * y, halfHeight,
            normal.x, normal.y, normal.z,
            color.r, color.g, color.b
            });
    }

    // Side indices (CCW winding from outside view)
    for (unsigned int i = 0; i < sectorCount; ++i) {
        unsigned int k1 = i * 2;
        unsigned int k2 = k1 + 2;

        indices.push_back(k1);
        indices.push_back(k2);
        indices.push_back(k1 + 1);

        indices.push_back(k1 + 1);
        indices.push_back(k2);
        indices.push_back(k2 + 1);
    }

    // Add center vertices for caps
    unsigned int baseIndex = static_cast<unsigned int>(vertices.size() / 9); // each vertex = 9 floats
    unsigned int bottomCenterIndex = baseIndex;
    unsigned int topCenterIndex = baseIndex + 1;

    glm::dvec3 bottomNormal(0, 0, -1), topNormal(0, 0, 1);

    vertices.insert(vertices.end(), {
        0.0f, 0.0f, -halfHeight, bottomNormal.x, bottomNormal.y, bottomNormal.z, color.r, color.g, color.b,
        0.0f, 0.0f,  halfHeight, topNormal.x,    topNormal.y,    topNormal.z,    color.r, color.g, color.b
        });

    // Bottom + top caps
    for (unsigned int i = 0; i < sectorCount; ++i) {
        double angle = i * sectorStep;
        double nextAngle = (i + 1) * sectorStep;

        double x0 = cos(angle), y0 = sin(angle);
        double x1 = cos(nextAngle), y1 = sin(nextAngle);

        // Bottom triangle (CCW from bottom view)
        unsigned int i0 = static_cast<unsigned int>(vertices.size() / 9);
        vertices.insert(vertices.end(), {
            radius * x1, radius * y1, -halfHeight, bottomNormal.x, bottomNormal.y, bottomNormal.z, color.r, color.g, color.b,
            radius * x0, radius * y0, -halfHeight, bottomNormal.x, bottomNormal.y, bottomNormal.z, color.r, color.g, color.b
            });

        indices.insert(indices.end(), {
            bottomCenterIndex, i0, i0 + 1
            });

        // Top triangle (CCW from top view)
        unsigned int i1 = static_cast<unsigned int>(vertices.size() / 9);
        vertices.insert(vertices.end(), {
            radius * x0, radius * y0, halfHeight, topNormal.x, topNormal.y, topNormal.z, color.r, color.g, color.b,
            radius * x1, radius * y1, halfHeight, topNormal.x, topNormal.y, topNormal.z, color.r, color.g, color.b
            });

        indices.insert(indices.end(), {
            topCenterIndex, i1, i1 + 1
            });
    }


    return OpenGLDataInitialize(vertices, indices);
}

Mesh Shapes::FaceToMesh(Face& face, glm::dvec3 color) {
    Mesh mesh;
    std::vector<double> vertices;

    // Add the 4 vertices of the face, along with their normals and colors
    for (const auto& v : face.facePoints) {
        // Position
        vertices.push_back(v.x);
        vertices.push_back(v.y);
        vertices.push_back(v.z);

        // Normal (same for all vertices of this face)
        vertices.push_back(face.normal.x);
        vertices.push_back(face.normal.y);
        vertices.push_back(face.normal.z);

        // Color
        vertices.push_back(color.r);
        vertices.push_back(color.g);
        vertices.push_back(color.b);
    }

    // Initialize OpenGL buffers and return the Mesh
    return OpenGLDataInitialize(vertices, face.indeces);
}

Mesh Shapes::OpenGLDataInitialize(std::vector<double>& vertices, std::vector<unsigned int>& indices)
{
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glFrontFace(GL_CCW);

    // OpenGL buffer setup
    GLuint VAO, VBO, EBO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    glBindVertexArray(VAO);

    // VBO
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(double), vertices.data(), GL_DYNAMIC_DRAW);

    // EBO
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_DYNAMIC_DRAW);

    // Position: location = 0
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(double), (void*)0);
    glEnableVertexAttribArray(0);

    // Normal: location = 1
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(double), (void*)(3 * sizeof(double)));
    glEnableVertexAttribArray(1);

    // Color: location = 2
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(double), (void*)(6 * sizeof(double)));
    glEnableVertexAttribArray(2);

    glBindVertexArray(0); // Unbind

    return { VAO, VBO, EBO, static_cast<GLsizei>(indices.size()),vertices,indices };
}

void Shapes::ProjectOntoAxis(
    const std::vector<double>& vertices,
    const glm::dvec3& axis,
    const glm::dmat4& modelMatrix,
    double& min,
    double& max
) {
    min = std::numeric_limits<double>::infinity();
    max = -std::numeric_limits<double>::infinity();
    for (int i = 0;i < vertices.size() / 9; i++) {
        glm::vec4 localPos(vertices[i * 9], vertices[i * 9 + 1], vertices[i * 9 + 2], 1.0f);
        glm::dvec3 worldPos = glm::dvec3(modelMatrix * localPos);
        double projection = glm::dot(worldPos, axis);

        min = std::min(min, projection);
        max = std::max(max, projection);
    }
}

bool Shapes::AreMeshesIntersectingSAT(
    const Mesh& meshA, const glm::dmat4& modelA,
    const Mesh& meshB, const glm::dmat4& modelB
) {
    const std::vector<glm::dvec3>& faceNormalsA = CalculateFaceNormals(meshA, modelA);
    const std::vector<glm::dvec3>& faceNormalsB = CalculateFaceNormals(meshB, modelB);

    std::vector<glm::dvec3> axes = faceNormalsA;
    axes.insert(axes.end(), faceNormalsB.begin(), faceNormalsB.end());


    // Optional: Add edge cross-products if using polyhedra like cylinders and boxes

    for (const glm::dvec3& axis : axes) {
        if (glm::length(axis) < 1e-6f) continue; // skip tiny vectors

        double minA, maxA, minB, maxB;
        ProjectOntoAxis(meshA.vertices, axis, modelA, minA, maxA);
        ProjectOntoAxis(meshB.vertices, axis, modelB, minB, maxB);

        if (maxA < minB || maxB < minA) {
            return false; // Separating axis found
        }
    }

    return true; // No separating axis => Intersection
}

std::vector<glm::dvec3> Shapes::CalculateFaceNormals(const Mesh& mesh, const glm::dmat4& modelMatrix) {
    std::vector<glm::dvec3> normals;

    for (size_t i = 0; i < mesh.indices.size(); i += 3) {
        unsigned int idx0 = mesh.indices[i];
        unsigned int idx1 = mesh.indices[i + 1];
        unsigned int idx2 = mesh.indices[i + 2];

        glm::vec4 p0(mesh.vertices[idx0 * 9], mesh.vertices[idx0 * 9 + 1], mesh.vertices[idx0 * 9 + 2], 1.0f);
        glm::vec4 p1(mesh.vertices[idx1 * 9], mesh.vertices[idx1 * 9 + 1], mesh.vertices[idx1 * 9 + 2], 1.0f);
        glm::vec4 p2(mesh.vertices[idx2 * 9], mesh.vertices[idx2 * 9 + 1], mesh.vertices[idx2 * 9 + 2], 1.0f);

        glm::dvec3 v0 = glm::dvec3(modelMatrix * p0);
        glm::dvec3 v1 = glm::dvec3(modelMatrix * p1);
        glm::dvec3 v2 = glm::dvec3(modelMatrix * p2);

        glm::dvec3 edge1 = v1 - v0;
        glm::dvec3 edge2 = v2 - v0;
        glm::dvec3 normal = glm::normalize(glm::cross(edge1, edge2));

        normals.push_back(normal);
    }

    return normals;
}
