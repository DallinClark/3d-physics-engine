#include "../include/collisions.h"

// Checks for type of bodies and calls the respective intersection function
bool Collisions::collide(std::shared_ptr<RigidBody> bodyA, std::shared_ptr<RigidBody> bodyB, glm::vec3& normal, float& depth) {
    ShapeType shapeTypeA = bodyA->getType();
    ShapeType shapeTypeB = bodyB->getType();

    if (shapeTypeA == ShapeType::Cube) {
        if (shapeTypeB == ShapeType::Cube) {
            return Collisions::intersectPolygons(bodyA->getTransformedVertices(), bodyB->getTransformedVertices(),bodyA->getPosition(),bodyB->getPosition(),bodyA->getFaces(), bodyB->getFaces(),bodyA->getEdges(),bodyB->getEdges(), normal, depth);
        }
        if (shapeTypeB == ShapeType::Sphere) {
            return Collisions::intersectCirclePolygon(bodyB->getPosition(), bodyB->getRadius(), bodyA->getTransformedVertices(),bodyA->getFaces(), bodyA->getPosition(), normal, depth);
        }
    }
    if (shapeTypeA == ShapeType::Sphere) {
        if (shapeTypeB == ShapeType::Cube) {
            bool result = Collisions::intersectCirclePolygon(bodyA->getPosition(), bodyA->getRadius(), bodyB->getTransformedVertices(),bodyB->getFaces(), bodyB->getPosition(), normal, depth);
            normal = -normal;
            return result;
        }
        if (shapeTypeB == ShapeType::Sphere) {
            return Collisions::intersectCircles(bodyA->getPosition(), bodyA->getRadius(), bodyB->getPosition(), bodyB->getRadius(), normal, depth);
        }
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

bool Collisions::intersectCirclePolygon(const glm::vec3& circleCenter, const float& circleRadius, const std::vector<glm::vec3>& vertices, vector<Face> faces, glm::vec3 polyCenter, glm::vec3& normal, float& depth) {
    // Get the face normals (axes) from the polygon.
    std::vector<glm::vec3> axes;
    for (Face face : faces) {
        axes.push_back(face.normal);
    }
    float minOverlap = FLT_MAX;
    glm::vec3 smallestAxis;

    // Additionally, check the axis from the circle center to the closest polygon vertex.
    float minDist = FLT_MAX;
    glm::vec3 closestVertex;
    for (const auto& v : vertices) {
        float dist = glm::distance(circleCenter, v);
        if (dist < minDist) {
            minDist = dist;
            closestVertex = v;
        }
    }
    axes.push_back(glm::normalize(closestVertex - circleCenter));


    // Check overlap on each polygon face normal.
    for (const auto& axis : axes) {
        float minPoly, maxPoly, minCircle, maxCircle;
        projectVerticesOntoAxis(vertices, axis, minPoly, maxPoly);
        projectCircleOntoAxis(circleRadius, circleCenter, axis, minCircle, maxCircle);

        if (maxPoly < minCircle || maxCircle < minPoly) {
            return false; // Separating axis found.
        }

        float overlap = std::min(maxPoly, maxCircle) - std::max(minPoly, minCircle);
        if (overlap < minOverlap) {
            minOverlap = overlap;
            smallestAxis = axis;

            // Make sure the axis points from the polygon toward the circle.
            if (glm::dot(circleCenter - polyCenter, smallestAxis) < 0)
                smallestAxis = -smallestAxis;
        }
    }

    normal = smallestAxis;
    depth = minOverlap;
    return true;
}

bool Collisions::intersectPolygons(const vector<glm::vec3>& verticesA, const vector<glm::vec3>& verticesB, const glm::vec3& centerA, const glm::vec3& centerB, vector<Face> facesA, vector<Face> facesB, vector<int> edgesA, vector<int> edgesB, glm::vec3& normal, float& depth) {
    std::vector<glm::vec3> axes;
    // Get axes from both polygons.
    std::vector<glm::vec3> axesA;
    std::vector<glm::vec3> axesB;
    for (Face face : facesA) {
        axesA.push_back(face.normal);
    }
    for (Face face : facesB) {
        axesB.push_back(face.normal);
    }
    axes.insert(axes.end(), axesA.begin(), axesA.end());
    axes.insert(axes.end(), axesB.begin(), axesB.end());

    float minOverlap = FLT_MAX;
    glm::vec3 smallestAxis;

    // Test each axis.
    for (const auto& axis : axes) {
        float minA, maxA, minB, maxB;
        projectVerticesOntoAxis(verticesA, axis, minA, maxA);
        projectVerticesOntoAxis(verticesB, axis, minB, maxB);

        // If there's a gap, then no collision exists.
        if (maxA < minB || maxB < minA)
            return false;

        float overlap = std::min(maxA, maxB) - std::max(minA, minB);
        if (overlap < minOverlap) {
            minOverlap = overlap;
            smallestAxis = axis;
            // Ensure the axis points from A to B.
            if (glm::dot(centerB - centerA, smallestAxis) < 0)
                smallestAxis = -smallestAxis;
        }
    }

    axes.clear();

    // now check along the cross product of all edges
    for (int i = 0; i < edgesA.size(); i += 2) {
        for (int j = 0; j < edgesB.size(); j += 2) {
            glm::vec3 edgeAVec = verticesA[edgesA[i]] - verticesA[edgesA[i + 1]];
            glm::vec3 edgeBVec = verticesB[edgesB[j]] - verticesB[edgesB[j + 1]];
            glm::vec3 crossProduct = glm::cross(edgeAVec, edgeBVec);

            if (glm::length(crossProduct) > 1e-6f)
                axes.push_back(glm::normalize(crossProduct));
        }
    }

    for (const auto& axis : axes) {
        float minA, maxA, minB, maxB;
        projectVerticesOntoAxis(verticesA, axis, minA, maxA);
        projectVerticesOntoAxis(verticesB, axis, minB, maxB);

        // If there's a gap, then no collision exists.
        if (maxA < minB || maxB < minA)
            return false;

        float overlap = std::min(maxA, maxB) - std::max(minA, minB);
        if (overlap < minOverlap) {
            minOverlap = overlap;
            smallestAxis = axis;

            // Ensure the axis points from A to B.
            if (glm::dot(centerB - centerA, smallestAxis) < 0)
                smallestAxis = -smallestAxis;
        }
    }
    normal = smallestAxis;
    depth = minOverlap;
    return true;
}

void Collisions::projectVerticesOntoAxis(const std::vector<glm::vec3>& vertices, const glm::vec3& axis, float& min, float& max) {
    min = FLT_MAX;
    max = -FLT_MAX;
    for (const auto& v : vertices) {
        float p = glm::dot(v, axis);
        if (p < min) min = p;
        if (p > max) max = p;
    }
}

void Collisions::projectCircleOntoAxis(float radius, glm::vec3 center, glm::vec3 axis, float& min, float& max) {
    float projection = glm::dot(center, axis);
    min = projection - radius;
    max = projection + radius;
}

// Helper: Find the index of the vertex that is farthest in the direction of the normal
static int findDeepestVertex(const std::vector<glm::vec3>& vertices, const glm::vec3& axis) {
    int bestIndex = -1;
    float bestProjection = -FLT_MAX;
    for (size_t i = 0; i < vertices.size(); ++i) {
        float proj = glm::dot(vertices[i], axis);
        if (proj > bestProjection) {
            bestProjection = proj;
            bestIndex = (int)i;
        }
    }
    return bestIndex;
}

// Helper: Selects the best face (as a Face struct) from a polyhedron that contains the target vertex 
// and whose face normal is most aligned with the given axis.
static Face selectBestFace(const std::vector<glm::vec3>& vertices,
    const std::vector<Face>& faces,
    int targetIndex,
    const glm::vec3& axis) {

    Face bestFace;
    float bestAlignment = -FLT_MAX;

    // Assume each face is defined by 4 indices.
    for (Face currFace : faces) {

        // Check if this face contains the target vertex.
        bool containsTarget = false;
        std::vector<int> faceIndices;
        for (auto faceIndice : currFace.indices) {
            if (faceIndice == targetIndex) {
                containsTarget = true;
            }
        }
        if (!containsTarget) {
            continue;
        }

        float alignment = glm::dot(currFace.normal, axis);
        if (alignment > bestAlignment) {
            bestAlignment = alignment;
            bestFace = currFace;
        }
    }
    return bestFace;
}

// Helper: Clips a polygon (subjectPolygon) against a single clip plane.
// The clip plane is defined by planeNormal and planeD (points P satisfy dot(P, planeNormal) >= planeD to be inside).
// Standard Sutherland-Hodgman Clipping against a single plane.
// Here the clip plane is defined by planeNormal and planePoint.
// A point is considered "inside" if: dot(P - planePoint, planeNormal) <= 0.
static std::vector<glm::vec3> clipPolygonAgainstPlane(
    const std::vector<glm::vec3>& subjectPolygon,
    const glm::vec3& planeNormal,
    const glm::vec3& planePoint) {

    std::vector<glm::vec3> output;
    int numVertices = subjectPolygon.size();
    if (numVertices == 0) return output;

    // Tolerance for floating-point precision issues
    const float tolerance = 1e-5f;

    for (int i = 0; i < numVertices; ++i) {
        glm::vec3 currVertex = subjectPolygon[i];
        glm::vec3 prevVertex = subjectPolygon[(i - 1 + numVertices) % numVertices]; // Wrap around
        glm::vec3 nextVertex = subjectPolygon[(i + 1) % numVertices]; // Wrap around

        float currDistance = glm::dot(currVertex - planePoint, planeNormal);
        float prevDistance = glm::dot(prevVertex - planePoint, planeNormal);
        float nextDistance = glm::dot(nextVertex - planePoint, planeNormal);

        bool currInside = (currDistance <= tolerance);
        bool prevInside = (prevDistance <= tolerance);
        bool nextInside = (nextDistance <= tolerance);

        glm::vec3 newVertex = currVertex; // Default to keeping the vertex

        if (!currInside) { // If current vertex is outside
            if (prevInside) {
                // Clip against previous edge
                float t = prevDistance / (prevDistance - currDistance);
                newVertex = prevVertex + t * (currVertex - prevVertex);
            }
            else if (nextInside) {
                // Clip against next edge
                float t = currDistance / (currDistance - nextDistance);
                newVertex = currVertex + t * (nextVertex - currVertex);
            }
            else {
                // Last resort: Project onto the plane
                newVertex = currVertex - currDistance * planeNormal;
            }
        }

        output.push_back(newVertex);
    }

    return output;
}

void Collisions::findContactPoints(
    std::shared_ptr<RigidBody> bodyA,
    std::shared_ptr<RigidBody> bodyB,
    const glm::vec3& collisionNormal,  // points from A to B
    float penetrationDepth,
    std::vector<glm::vec3>& outContactPoints,
    int& outContactCount) {
    if (bodyA->shapeType == ShapeType::Cube ) {
        if (bodyB->shapeType == ShapeType::Cube) {
            findContactPointsPolygonToPolygon(bodyA, bodyB, collisionNormal, penetrationDepth, outContactPoints, outContactCount);
            return;
        }
        if (bodyB->shapeType == ShapeType::Sphere) {
            findContactPointsSphereToPolygon(bodyB, bodyA, collisionNormal, penetrationDepth, outContactPoints, outContactCount);
            return;
        }
    }
    if (bodyA->shapeType == ShapeType::Sphere) {
        if (bodyB->shapeType == ShapeType::Cube) {
            findContactPointsSphereToPolygon(bodyA, bodyB, -collisionNormal, penetrationDepth, outContactPoints, outContactCount);
            return;
        }
        if (bodyB->shapeType == ShapeType::Sphere) {
            findContactPointsSphereToSphere(bodyB, bodyA, collisionNormal, penetrationDepth, outContactPoints, outContactCount);
            return;
        }
    }
}



void Collisions::findContactPointsPolygonToPolygon(
    std::shared_ptr<RigidBody> bodyA,
    std::shared_ptr<RigidBody> bodyB,
    const glm::vec3& collisionNormal,  // points from A to B
    float penetrationDepth,
    std::vector<glm::vec3>& outContactPoints,
    int& outContactCount) {

    vector<glm::vec3> verticesA = bodyA->getTransformedVertices();
    vector<glm::vec3> verticesB = bodyB->getTransformedVertices();
    vector<Face> facesA = bodyA->getFaces(); 
    vector<Face> facesB = bodyB->getFaces();

    int minIndexA = findDeepestVertex(verticesA, collisionNormal);
    int minIndexB = findDeepestVertex(verticesB, -collisionNormal);

    // Select best faces.
    Face faceA = selectBestFace(verticesA, facesA, minIndexA, collisionNormal);  
    Face faceB = selectBestFace(verticesB, facesB, minIndexB, -collisionNormal);

    // Choose reference and incident faces.
    float alignmentA = glm::dot(faceA.normal, collisionNormal);
    float alignmentB = glm::dot(faceB.normal, -collisionNormal);

    Face referenceFace, incidentFace;
    std::vector<Face> referencePolygonFaces;
    std::vector<glm::vec3> referencePolygonVertices;
    glm::vec3 newClippingNormal;

    if (alignmentA < alignmentB) {
        referenceFace = faceA;
        incidentFace = faceB;
        referencePolygonVertices = verticesA;
        referencePolygonFaces = facesA;
        newClippingNormal = collisionNormal;
    }
    else {
        referenceFace = faceB;
        incidentFace = faceA;
        referencePolygonVertices = verticesB;
        referencePolygonFaces = facesB;
        newClippingNormal = -collisionNormal;
    }

    // --- Clipping Stage ---
    // We clip the incident face polygon against each edge of the reference face.
    // For each edge of the reference face, define a clip plane:
    // The plane’s normal is computed as the cross product of the reference face normal and the edge vector.
    std::vector<glm::vec3> clippedPolygon = incidentFace.vertices;

    
    for (Face currAdjacentFace : referencePolygonFaces) {

        // checks if the current face is adjacent to the reference face
        bool isSameFace = true;
        bool isAdjacent = false;
        for (int referenceFaceIndex : referenceFace.indices) {
            for (int adjacentFaceIndex : currAdjacentFace.indices) {
                if (referenceFaceIndex == adjacentFaceIndex) {
                    isAdjacent = true;
                    break;
                }
            }
            if (std::find(currAdjacentFace.indices.begin(), currAdjacentFace.indices.end(), referenceFaceIndex) == currAdjacentFace.indices.end()) { // TODO combine with logic above for better efficiency
                isSameFace = false;
            }
        }

        if (isSameFace || !isAdjacent) {
            continue;
        }

        glm::vec3 clipNormal = currAdjacentFace.normal;

        // The plane is defined such that points on the reference face are inside:
        clippedPolygon = clipPolygonAgainstPlane(clippedPolygon, clipNormal, currAdjacentFace.vertices[0]);
    }

    glm::vec3 planePoint = referenceFace.vertices[0];
    std::vector<glm::vec3> finalContacts;

    for (const auto& pt : clippedPolygon) {
        // Compute the distance from the point to the reference plane.
        float dist = glm::dot((pt - planePoint), referenceFace.normal);

        if (dist < 0) {
            finalContacts.push_back(pt);
        }
    }

    outContactPoints = finalContacts;
    outContactCount = static_cast<int>(finalContacts.size());
    if (outContactCount == 0) {
        std::cerr << "NO CONTACT POINTS FOUND\n";
        //outContactPoints = { clippedPolygon[0] };
        //outContactCount = 1;
    }
}
void Collisions::findContactPointsSphereToSphere(
    std::shared_ptr<RigidBody> bodyA,     // the sphere body
    std::shared_ptr<RigidBody> bodyB,         // the polygon (convex polyhedron) body
    const glm::vec3& collisionNormal,        // collision normal from sphere to polygon
    float penetrationDepth,
    std::vector<glm::vec3>& outContactPoints,
    int& outContactCount) {

    float radiusA = bodyA->getRadius();
    float radiusB = bodyB->getRadius();

    glm::vec3 centerA = bodyA->getPosition();
    glm::vec3 centerB = bodyB->getPosition();

    outContactPoints.push_back(centerA + (radiusA * collisionNormal));
    outContactPoints.push_back(centerB - (radiusB * collisionNormal));

    outContactCount = 2;
}


void Collisions::findContactPointsSphereToPolygon(
    std::shared_ptr<RigidBody> sphere,     // the sphere body
    std::shared_ptr<RigidBody> poly,         // the polygon (convex polyhedron) body
    const glm::vec3& collisionNormal,        // collision normal from sphere to polygon
    float penetrationDepth,
    std::vector<glm::vec3>& outContactPoints,
    int& outContactCount) {

    glm::vec3 sphereCenter = sphere->getPosition();
    float sphereRadius = sphere->radius;  

    std::vector<glm::vec3> verticesPoly = poly->getTransformedVertices();
    std::vector<Face> facesPoly = poly->getFaces();

    int minIndex = findDeepestVertex(verticesPoly, collisionNormal);

    // Select best faces.
    Face referenceFace = selectBestFace(verticesPoly, facesPoly, minIndex, collisionNormal);

    // --- Step 2: Project the sphere’s center onto the polygon’s plane ---
    // The plane is defined by a point (using the first vertex of the face) and the face normal.
    glm::vec3 planePoint = referenceFace.vertices[0];
    float planeD = glm::dot(planePoint, referenceFace.normal);
    float centerToPlaneDist = glm::dot(sphereCenter, referenceFace.normal) - planeD;
    glm::vec3 projectedPoint = sphereCenter - centerToPlaneDist * referenceFace.normal;

    // --- Step 3: Clamp the projected point to the polygon (if necessary) ---
    bool inside = true;
    int numVerts = referenceFace.vertices.size();
    for (int i = 0; i < numVerts; i++) {
        glm::vec3 v0 = referenceFace.vertices[i];
        glm::vec3 v1 = referenceFace.vertices[(i + 1) % numVerts];
        glm::vec3 edge = v1 - v0;
        // Create an outward edge normal (perpendicular to edge and face normal)
        glm::vec3 edgeNormal = glm::cross(referenceFace.normal, edge);
        // If the point lies outside this edge, it will have a negative dot product.
        if (glm::dot(projectedPoint - v0, edgeNormal) < 0) {
            inside = false;
            break;
        }
    }

    glm::vec3 polyContactPoint;
    if (inside) {
        // The projection lies within the polygon.
        polyContactPoint = projectedPoint;
    }
    else {
        // Clamp: Find the closest point on the polygon’s perimeter.
        float minDist = FLT_MAX;
        glm::vec3 closestPoint;
        for (int i = 0; i < numVerts; i++) {
            glm::vec3 v0 = referenceFace.vertices[i];
            glm::vec3 v1 = referenceFace.vertices[(i + 1) % numVerts];
            glm::vec3 edge = v1 - v0;
            float t = glm::clamp(glm::dot(sphereCenter - v0, edge) / glm::dot(edge, edge), 0.0f, 1.0f);
            glm::vec3 pointOnEdge = v0 + t * edge;
            float d = glm::length(sphereCenter - pointOnEdge);
            if (d < minDist) {
                minDist = d;
                closestPoint = pointOnEdge;
            }
        }
        polyContactPoint = closestPoint;
    }

    // --- Step 4: Determine the sphere's contact point ---
    // The contact point on the sphere is typically at its surface along -collisionNormal.
    glm::vec3 sphereContactPoint = sphereCenter - sphereRadius * collisionNormal;

    outContactPoints.clear();
    outContactPoints.push_back(polyContactPoint);
    outContactPoints.push_back(sphereContactPoint);
    outContactCount = 2;

    // Debug output if no contact points are found (should not happen in a proper collision)
    if (outContactCount == 0) {
        std::cerr << "NO CONTACT POINTS FOUND\n";
    }
}

