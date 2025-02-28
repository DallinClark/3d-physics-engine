#include "../include/collisions.h"

// Checks for type of bodies and calls the respective intersection function
bool Collisions::collide(std::shared_ptr<RigidBody> bodyA, std::shared_ptr<RigidBody> bodyB, glm::vec3& normal, float& depth) {
    ShapeType shapeTypeA = bodyA->getType();
    ShapeType shapeTypeB = bodyB->getType();

    if (shapeTypeA == ShapeType::Cube) {
        if (shapeTypeB == ShapeType::Cube) {
            return Collisions::intersectPolygons(bodyA->getTransformedVertices(), bodyB->getTransformedVertices(),bodyA->getPosition(),bodyB->getPosition(), normal, depth);
        }
        if (shapeTypeB == ShapeType::Sphere) {
            bool result = Collisions::intersectPolygons(bodyA->getTransformedVertices(), bodyB->getTransformedVertices(), bodyA->getPosition(), bodyB->getPosition(), normal, depth);
            normal = -normal;
            return result;
        }
    }
    if (shapeTypeA == ShapeType::Sphere) {
        if (shapeTypeB == ShapeType::Cube) {
            return Collisions::intersectPolygons(bodyA->getTransformedVertices(), bodyB->getTransformedVertices(), bodyA->getPosition(), bodyB->getPosition(), normal, depth);
        }
        if (shapeTypeB == ShapeType::Sphere) {
            return Collisions::intersectPolygons(bodyA->getTransformedVertices(), bodyB->getTransformedVertices(), bodyA->getPosition(), bodyB->getPosition(), normal, depth);
        }
        return false;
    }
}

bool Collisions::intersectCircles(glm::vec3 centerA, float radiusA, glm::vec3 centerB, float radiusB, glm::vec3 & normal, float& depth) {

	float distance = glm::distance(centerA, centerB);
	float radii = radiusA + radiusB;

	if (distance >= radii) {
		return false;
	}

	normal = glm::normalize(centerB - centerA);
	depth = radii - distance;

	return true;
}

bool Collisions::intersectCirclePolygon(const glm::vec3& circleCenter, const float& circleRadius, const vector<glm::vec3>& vertices, glm::vec3 polyCenter, glm::vec3& normal, float& depth) {
    vector<glm::vec3> simplex;
    glm::vec3 direction = polyCenter - circleCenter;
    glm::vec3 firstPoint = minkowskiDifference(vertices, circleCenter, circleRadius, direction);

    simplex.push_back(firstPoint);
    direction = -simplex[0];

    for (int i = 0; i < GJK_MAX_NUM_ITERATIONS; ++i) {
        glm::vec3 newPoint = minkowskiDifference(vertices, circleCenter, circleRadius, direction);

        if (glm::dot(newPoint, direction) <= 0) {
            return false;  // No collision
        }

        simplex.push_back(newPoint);

        if (nextSimplex(simplex, direction)) {
            std::vector<glm::vec3> emptyVertices;
            EPA(simplex, vertices, emptyVertices, circleCenter, circleRadius, true, depth, normal);
            return true;
        }
    }
    return false;
}

bool Collisions::intersectPolygons(const vector<glm::vec3>& verticesA, const vector<glm::vec3>& verticesB, const glm::vec3& centerA, const glm::vec3& centerB, glm::vec3& normal, float& depth) {
    vector<glm::vec3> simplex;
    glm::vec3 direction = centerA - centerB;
    glm::vec3 firstPoint = minkowskiDifference(verticesA, verticesB, direction);

    simplex.push_back(firstPoint);
    direction = -simplex[0];

    for (int i = 0; i < GJK_MAX_NUM_ITERATIONS; ++i) {
        glm::vec3 newPoint = minkowskiDifference(verticesA, verticesB, direction);

        if (glm::dot(newPoint, direction) <= 0) {
            return false;  // No collision
        }

        simplex.push_back(newPoint);

        if (nextSimplex(simplex, direction)) {
            EPA(simplex, verticesA, verticesB,glm::vec3(0.0,0.0,0.0),0.0f, false, depth, normal);
            return true;
        }
    }
    return false;
}

bool Collisions::line(std::vector<glm::vec3>& simplex, glm::vec3& direction) {
    glm::vec3 a = simplex[1];
    glm::vec3 b = simplex[0];

    glm::vec3 ab = b - a;
    glm::vec3 ao = -a;

    if (glm::dot(ab, ao) > 0) {
        direction = glm::cross(glm::cross(ab, ao), ab);
        if (glm::length(direction) < 1e-6f) {
            // Fallback direction, e.g., perpendicular to ab
            direction = glm::normalize(glm::cross(ab, glm::vec3(0.0f, 1.0f, 0.0f)));
        }
    }
    else {
        simplex = { a };
        direction = ao;
    }
    return false;
}
bool Collisions::triangle(std::vector<glm::vec3>& simplex, glm::vec3& direction) {
    glm::vec3 a = simplex[2];
    glm::vec3 b = simplex[1];
    glm::vec3 c = simplex[0];

    glm::vec3 ab = b - a;
    glm::vec3 ac = c - a;
    glm::vec3 ao = -a;

    glm::vec3 abc = glm::cross(ab, ac);  // Triangle normal

    // Check if origin is outside AC
    if (glm::dot(glm::cross(abc, ac), ao) > 0.001f) {
        if (glm::dot(ac, ao) > 0.001f) {
            simplex = { a ,c };
            direction = glm::cross(glm::cross(ac, ao), ac);
        }
        else {
            simplex = { a,b };
            return line(simplex, direction);
        }
    }
    // Check if origin is outside AB
    else {
        if (glm::dot(glm::cross(ab, abc), ao) > 0.001f) {
            simplex = { a , b };
            return line(simplex, direction);
        }
        else {
            if (glm::dot(abc, ao) > 0.001f) {
                direction = abc;
            }
            else {
                simplex = { a, c, b };
                direction = -abc;
            }
        }
    }
    return false;
}

bool Collisions::tetrahedron(std::vector<glm::vec3>& simplex, glm::vec3& direction) {
    glm::vec3 a = simplex[3];
    glm::vec3 b = simplex[2];
    glm::vec3 c = simplex[1];
    glm::vec3 d = simplex[0];

    glm::vec3 ab = b - a;
    glm::vec3 ac = c - a;
    glm::vec3 ad = d - a;
    glm::vec3 ao = -a;

    // Compute face normals
    glm::vec3 abc = glm::cross(ab, ac);
    glm::vec3 acd = glm::cross(ac, ad);
    glm::vec3 adb = glm::cross(ad, ab);

    // Determine which face the origin is outside of and reduce simplex
    float dot = glm::dot(abc, ao);
    if (dot > 0) {
        simplex = { a , b ,c };
        return triangle(simplex, direction);
    }
    else if (glm::dot(acd, ao) > 0) {
        simplex = { a, c ,d };
        return triangle(simplex, direction);
    }
    else if (glm::dot(adb, ao) > 0) {
        simplex = { a , d, b };
        return triangle(simplex , direction);
    }
    return true;
}

bool Collisions::nextSimplex(std::vector<glm::vec3>& simplex, glm::vec3& direction) {
    switch (simplex.size()) {
        case 2: return line(simplex, direction);
        case 3: return triangle(simplex, direction);
        case 4: return tetrahedron(simplex, direction);
    }

    // never should be here
    return false;
}

/* Helper for GJK, takes a direction and a set of points
   and finds the difference of the points farthest apart
   in that direction, used for creating a convex hull */

glm::vec3 Collisions::minkowskiDifference(const vector<glm::vec3>& verticesA, const vector<glm::vec3>& verticesB, const glm::vec3& direction) {
    glm::vec3 maxPointA;
    glm::vec3 maxPointB;
    float maxDistanceA = -FLT_MAX;
    float maxDistanceB = -FLT_MAX;

    for (auto& vertex : verticesA) {
        float distance = glm::dot(vertex, direction);
        if (distance > maxDistanceA) {
            maxDistanceA = distance;
            maxPointA = vertex;
        }
    }

    for (auto& vertex : verticesB) {
        glm::vec3 negDirection = -direction;
        float distance = glm::dot(vertex, negDirection);
        if (distance > maxDistanceB) {
            maxDistanceB = distance;
            maxPointB = vertex;
        }
    }

    glm::vec3 difference = maxPointA - maxPointB;

    return difference;
}
glm::vec3 Collisions::minkowskiDifference(const vector<glm::vec3>& vertices, const glm::vec3& circleCenter, const float radius, glm::vec3& direction) {
    glm::vec3 maxPointA;
    glm::vec3 maxPointB;
    float maxDistance = -FLT_MAX;

    for (auto& vertex : vertices) {
        float distance = glm::dot(vertex, direction);
        if (distance > maxDistance) {
            maxDistance = distance;
            maxPointA = vertex;
        }
    }
    glm::vec3 normalizedDirection = glm::normalize(direction);

    maxPointB = circleCenter - (radius * normalizedDirection);
    return maxPointA - maxPointB;
}




void Collisions::addIfUniqueEdge(std::vector<std::pair<int, int>>& edges,int a, int b) {
    auto reverse = std::find(edges.begin(), edges.end(), std::make_pair(b, a));

    if (reverse != edges.end()) {
        edges.erase(reverse);
    }
    else {
        edges.emplace_back(a, b);
    }
}

bool Collisions::EPA(std::vector<glm::vec3> simplex, const std::vector<glm::vec3>& verticesA, const std::vector<glm::vec3>& verticesB,
    const glm::vec3& circleCenter, const float& radius, bool isCircle, float& depth, glm::vec3& normal) {

    std::vector<glm::vec3> polytope(simplex.begin(), simplex.end());
    std::vector<int> faces = { 0, 1, 2, 0, 3, 1, 0, 2, 3, 1, 3, 2 };
    std::vector<glm::vec4> normals;

    int minTriangle = -1;
    float minDistance = FLT_MAX;

    // Compute face normals
    for (int i = 0; i < faces.size(); i += 3) {
        glm::vec3 a = polytope[faces[i]];
        glm::vec3 b = polytope[faces[i + 1]];
        glm::vec3 c = polytope[faces[i + 2]];

        glm::vec3 faceNormal = glm::normalize(glm::cross(b - a, c - a));
        glm::vec3 centroid = (a + b + c) / 3.0f;
        float distance = glm::dot(faceNormal, centroid);

        // Ensure normal points outward
        if (distance < 0) {
            faceNormal = -faceNormal;
            distance = -distance;
        }

        normals.emplace_back(faceNormal, distance);

        if (distance < minDistance) {
            minTriangle = i / 3;
            minDistance = distance;
        }
    }

    glm::vec3 minNormal;

    // Main EPA loop
    for (int i = 0; i < EPA_MAX_NUM_ITERATIONS; ++i) {
        minNormal = glm::vec3(normals[minTriangle]);
        minDistance = normals[minTriangle].w;

        // Get a new support point in the direction of the closest face normal
        glm::vec3 newPoint;
        newPoint = minkowskiDifference(verticesA, verticesB, minNormal);

        /*if (isCircle) {
            newPoint = minkowskiDifference(verticesA, circleCenter, radius, minNormal);
        }
        else {
            newPoint = minkowskiDifference(verticesA, verticesB, minNormal);
        }*/

        float newPointDistance = glm::dot(minNormal, newPoint);

        if (fabs(newPointDistance - minDistance) <= EPA_TOLERANCE) {
            normal = minNormal;
            depth = minDistance + 0.001f;
            return true;
        }

        std::vector<std::pair<int, int>> uniqueEdges;

        // Find and remove faces facing towards newPoint
        for (int i = faces.size() / 3 - 1; i >= 0; i--) {
            int f = i * 3;
            if (glm::dot(glm::vec3(normals[i]), newPoint) > 0) {
                addIfUniqueEdge(uniqueEdges, faces[f], faces[f + 1]);
                addIfUniqueEdge(uniqueEdges, faces[f + 1], faces[f + 2]);
                addIfUniqueEdge(uniqueEdges, faces[f + 2], faces[f]);

                // Remove face and normal
                faces.erase(faces.begin() + f, faces.begin() + f + 3);
                normals.erase(normals.begin() + i);
            }
        }

        std::vector<int> newFaces;
        for (const auto& edge : uniqueEdges) {
            newFaces.push_back(edge.first);
            newFaces.push_back(edge.second);
            newFaces.push_back(polytope.size());
        }

        polytope.push_back(newPoint);

        std::vector<glm::vec4> newNormals;
        int newMinTriangle = -1;

        // Compute new face normals
        for (int i = 0; i < newFaces.size(); i += 3) {
            glm::vec3 a = polytope[newFaces[i]];
            glm::vec3 b = polytope[newFaces[i + 1]];
            glm::vec3 c = polytope[newFaces[i + 2]];

            glm::vec3 faceNormal = glm::normalize(glm::cross(b - a, c - a));
            glm::vec3 centroid = (a + b + c) / 3.0f;
            float distance = glm::dot(faceNormal, centroid);

            if (distance < 0) {
                faceNormal = -faceNormal;
                distance = -distance;
            }

            newNormals.emplace_back(faceNormal, distance);

            if (distance < minDistance) {
                newMinTriangle = i / 3;
                minDistance = distance;
            }
        }

        // Find the new closest normal
        float oldMinDistance = FLT_MAX;
        for (int i = 0; i < normals.size(); ++i) {
            if (normals[i].w < oldMinDistance) {
                oldMinDistance = normals[i].w;
                minTriangle = i;
            }
        }
        if (newMinTriangle != -1 && newNormals[newMinTriangle].w < oldMinDistance) {
            minTriangle = newMinTriangle + normals.size();
        }

        faces.insert(faces.end(), newFaces.begin(), newFaces.end());
        normals.insert(normals.end(), newNormals.begin(), newNormals.end());
    }

    normal = minNormal;
    depth = minDistance + 0.001f;
    return true;
}
