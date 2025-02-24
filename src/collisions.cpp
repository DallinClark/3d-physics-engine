#include "../include/collisions.h"
// Checks for type of bodies and calls the respective intersection function
bool Collisions::Collide(std::shared_ptr<RigidBody2D> bodyA, std::shared_ptr<RigidBody2D> bodyB, glm::vec2& normal, float& depth) {
    ShapeType shapeTypeA = bodyA->getType();
    ShapeType shapeTypeB = bodyB->getType();

    if (shapeTypeA == ShapeType::Square) {
        if (shapeTypeB == ShapeType::Square) {
            return Collisions::IntersectPolygons(bodyA->getTransformedVertices(), bodyB->getTransformedVertices(), bodyA->getPosition(), bodyB->getPosition(), normal, depth);
        }
        if (shapeTypeB == ShapeType::Circle) {
            bool result = Collisions::IntersectCirclePolygon(bodyB->getPosition(), bodyB->getRadius(), bodyA->getTransformedVertices(), bodyA->getPosition(), normal, depth);
            normal = -normal;
            return result;
        }
    }
    else if (shapeTypeA == ShapeType::Circle) {
        if (shapeTypeB == ShapeType::Square) {
            return Collisions::IntersectCirclePolygon(bodyA->getPosition(), bodyA->getRadius(), bodyB->getTransformedVertices(), bodyB->getPosition(), normal, depth);
        }
        if (shapeTypeB == ShapeType::Circle) {
            return Collisions::IntersectCircles(bodyA->getPosition(), bodyA->getRadius(), bodyB->getPosition(), bodyB->getRadius(), normal, depth);
        }
    }

}

bool Collisions::IntersectCircles(glm::vec2 centerA, float radiusA, glm::vec2 centerB, float radiusB,
	glm::vec2& normal, float& depth) {
	
	float distance = glm::distance(centerA, centerB);
	float radii = radiusA + radiusB;

	if (distance >= radii) {
		return false;
	}

	normal = glm::normalize(centerB - centerA);
	depth = radii - distance;

	return true;
}
bool Collisions::IntersectCirclePolygon(glm::vec2 circleCenter, float circleRadius, vector<glm::vec4> vertices, glm::vec2 polyCenter, glm::vec2& normal, float& depth) {
    normal = glm::vec2(0.0f, 0.0f);
    depth = FLT_MAX;
    float axisDepth = 0;
    float minA, maxA, minB, maxB;
    glm::vec2 axis = glm::vec2(0.0f, 0.0f);

    // Check all edges of the polygon
    for (int i = 0; i < vertices.size(); ++i) {
        glm::vec2 va = glm::vec2(vertices[i].x, vertices[i].y);
        glm::vec2 vb = glm::vec2(vertices[(i + 1) % vertices.size()].x, vertices[(i + 1) % vertices.size()].y);

        glm::vec2 edge = vb - va;
        axis = glm::normalize(glm::vec2(-edge.y, edge.x));  // Perpendicular axis to the edge

        // Project vertices onto the axis
        Collisions::ProjectVertices(vertices, axis, minA, maxA);
        Collisions::ProjectCircle(circleCenter, circleRadius, axis, minB, maxB);

        if (minA >= maxB || minB >= maxA) {
            return false;  // No intersection on this axis, return false
        }

        // Calculate the overlap on the axis
        axisDepth = std::min(maxB - minA, maxA - minB);

        if (axisDepth < depth) {
            depth = axisDepth;  // Update depth if the current axis gives a smaller overlap
            normal = axis;  // Update normal to the current axis
        }
    }

    // Find the closest point on the polygon to the circle's center
    int cpIndex = Collisions::FindClosestPointOnPolygon(circleCenter, vertices);
    glm::vec2 cp = vertices[cpIndex];

    // Calculate the normal from the circle's center to the closest point
    axis = cp - circleCenter;
    axis = glm::normalize(axis);

    // Project vertices onto the axis for the closest point check
    Collisions::ProjectVertices(vertices, axis, minA, maxA);
    Collisions::ProjectCircle(circleCenter, circleRadius, axis, minB, maxB);

    if (minA >= maxB || minB >= maxA) {
        return false;  // No intersection, return false
    }

    axisDepth = std::min(maxB - minA, maxA - minB);

    // Update the depth and normal if a closer overlap is found
    if (axisDepth < depth) {
        depth = axisDepth;
        normal = axis;
    }

    // Check the orientation of the normal (if necessary)
    //glm::vec2 polygonCenter = Collisions::FindArethmeticMean(vertices);
    glm::vec2 polygonCenter = polyCenter;
    glm::vec2 direction = polygonCenter - circleCenter;

    // If the normal is pointing inward, reverse it
    if (glm::dot(direction, normal) < 0.0f) {
        normal = -normal;
    }

    return true;  // Return true if intersection is detected
}
bool Collisions::IntersectPolygons(vector<glm::vec4> verticesA, vector<glm::vec4> verticesB, glm::vec2 polyCenterA, glm::vec2 polyCenterB, glm::vec2& normal, float& depth) {
    normal = glm::vec2(0.0f, 0.0f);
    depth = FLT_MAX;

    for (int i = 0; i < verticesA.size(); ++i) {
        glm::vec2 va = glm::vec2(verticesA[i].x, verticesA[i].y);
        glm::vec2 vb = glm::vec2(verticesA[(i + 1) % verticesA.size()].x, verticesA[(i + 1) % verticesA.size()].y);

        glm::vec2 edge = vb - va;
        glm::vec2 axis = glm::vec2(-edge.y, edge.x);  // Perpendicular axis to the edge

        // Normalize axis
        axis = glm::normalize(axis);

        float minA, maxA, minB, maxB;

        // Project vertices onto the axis
        Collisions::ProjectVertices(verticesA, axis, minA, maxA);
        Collisions::ProjectVertices(verticesB, axis, minB, maxB);

        if (minA >= maxB || minB >= maxA) {
            return false;  // No intersection on this axis, return false
        }

        // Calculate the overlap on the axis
        float axisDepth = std::min(maxB - minA, maxA - minB);

        if (axisDepth < depth) {
            depth = axisDepth;  // Update depth if the current axis gives a smaller overlap
            normal = axis;  // Update normal to the current axis
        }
    }

    // Check for each edge in polygon B
    for (int i = 0; i < verticesB.size(); ++i) {
        glm::vec2 va = glm::vec2(verticesB[i].x, verticesB[i].y);
        glm::vec2 vb = glm::vec2(verticesB[(i + 1) % verticesB.size()].x, verticesB[(i + 1) % verticesB.size()].y);

        glm::vec2 edge = vb - va;
        glm::vec2 axis = glm::vec2(-edge.y, edge.x);  // Perpendicular axis to the edge

        // Normalize axis
        axis = glm::normalize(axis);

        float minA, maxA, minB, maxB;

        // Project vertices onto the axis
        Collisions::ProjectVertices(verticesA, axis, minA, maxA);
        Collisions::ProjectVertices(verticesB, axis, minB, maxB);

        if (minA >= maxB || minB >= maxA) {
            return false;  // No intersection on this axis, return false
        }

        // Calculate the overlap on the axis
        float axisDepth = std::min(maxB - minA, maxA - minB);

        if (axisDepth < depth) {
            depth = axisDepth;  // Update depth if the current axis gives a smaller overlap
            normal = axis;  // Update normal to the current axis
        }
    }

    //glm::vec2 centerA = Collisions::FindArethmeticMean(verticesA);
    //glm::vec2 centerB = Collisions::FindArethmeticMean(verticesB);
    glm::vec2 centerA = polyCenterA;
    glm::vec2 centerB = polyCenterB;

    glm::vec2 direction = centerB - centerA;

    if (glm::dot(direction, normal) < 0) {
        normal = -normal;
    }

    return true;  // Return true if intersection was found
}

// Helper methods for intersection detection
int Collisions::FindClosestPointOnPolygon(glm::vec2 circleCenter, vector<glm::vec4> vertices) {
    int result = -1;
    float minDistance = FLT_MAX;

    // Iterate over all vertices to find the closest one to the circle center
    for (int i = 0; i < vertices.size(); ++i) {
        glm::vec2 v = vertices[i];
        float distance = glm::distance(v, circleCenter);

        // Update the closest point if a smaller distance is found
        if (distance < minDistance) {
            minDistance = distance;
            result = i;
        }
    }
    return result;
}
void Collisions::ProjectCircle(glm::vec2 center, float radius, glm::vec2 axis, float& min, float& max) {
    glm::vec2 direction = glm::normalize(axis);
    glm::vec2 directionAndRadius = direction * radius;

    glm::vec2 p1 = center + directionAndRadius;
    glm::vec2 p2 = center - directionAndRadius;

    min = glm::dot(p1, axis);
    max = glm::dot(p2, axis);

    if (min > max) {
        float t = min;
        min = max;
        max = t;
    }
}
void Collisions::ProjectVertices(vector<glm::vec4> vertices, glm::vec2 axis, float& min, float& max) {
	float proj = glm::dot(glm::vec2(vertices[0].x, vertices[0].y), axis);
	min = max = proj;  

	for (int i = 1; i < vertices.size(); ++i) {
		glm::vec2 v = glm::vec2(vertices[i].x, vertices[i].y);
		proj = glm::dot(v, axis);

		if (proj < min) { min = proj; }
		if (proj > max) { max = proj; }
	}
}

// Method for finding where two bodies collide, finds body type and calls respective function
void Collisions::FindContactPoints(std::shared_ptr<RigidBody2D> bodyA, std::shared_ptr<RigidBody2D> bodyB, glm::vec2& contactOne, glm::vec2& contactTwo, int& contactCount) {
    ShapeType shapeTypeA = bodyA->getType();
    ShapeType shapeTypeB = bodyB->getType();

    if (shapeTypeA == ShapeType::Square) {
        if (shapeTypeB == ShapeType::Square) {
            Collisions::FindContactPoint(bodyA->getTransformedVertices(), bodyB->getTransformedVertices(), contactOne, contactTwo, contactCount);
        }
        if (shapeTypeB == ShapeType::Circle) {
            Collisions::FindContactPoint(bodyB->getPosition(), bodyB->getRadius(), bodyA->getPosition(), bodyA->getTransformedVertices(), contactOne);
            contactCount = 1;
        }
    }
    else if (shapeTypeA == ShapeType::Circle) {
        if (shapeTypeB == ShapeType::Square) {
            Collisions::FindContactPoint(bodyA->getPosition(), bodyA->getRadius(), bodyB->getPosition(), bodyB->getTransformedVertices(), contactOne);
            contactCount = 1;
        }
        if (shapeTypeB == ShapeType::Circle) {
            Collisions::FindContactPoint(bodyA->getPosition(), bodyA->getRadius(), bodyB->getPosition(), contactOne);
            contactCount = 1;
        }
    }
}

//Polygon to Polygon collision points
void Collisions::FindContactPoint(vector<glm::vec4> verticesA, vector<glm::vec4> verticesB, glm::vec2& contact1, glm::vec2& contact2, int& contactCount) {
    contact1 = glm::vec2(0.0f, 0.0f);
    contact2 = glm::vec2(0.0f, 0.0f);
    contactCount = 0;

    float minDistSq = FLT_MAX;

    for (int i = 0; i < verticesA.size(); ++i) {
        glm::vec2 p = verticesA[i];

        for (int j = 0; j < verticesB.size(); ++j) {
            glm::vec2 va = verticesB[j];
            glm::vec2 vb = verticesB[(j + 1) % verticesB.size()];
            float distSq;
            glm::vec2 cp;

            Collisions::PointSegmentDistance(p, va, vb, distSq, cp);

            if (std::abs(distSq - minDistSq) < 0.01f) {
                //cout << " cp.x is " << cp.x << " contact1.x is " << contact1.x << " cp.y is " << cp.y << " contact1.y is " << contact1.y << endl;
                if (std::abs(cp.x - contact1.x) > .01 || std::abs(cp.y - contact1.y) > .01){
                    contact2 = cp;
                    contactCount = 2;
                }
            }
            else if (distSq < minDistSq) {
                minDistSq = distSq;
                contactCount = 1;
                contact1 = cp;
            }
        }
    }

    for (int i = 0; i < verticesB.size(); ++i) {
        glm::vec2 p = verticesB[i];

        for (int j = 0; j < verticesA.size(); ++j) {
            glm::vec2 va = verticesA[j];
            glm::vec2 vb = verticesA[(j + 1) % verticesA.size()];
            float distSq;
            glm::vec2 cp;

            Collisions::PointSegmentDistance(p, va, vb, distSq, cp);

            if (std::abs(distSq - minDistSq) < .01f) {
                if (std::abs(cp.x - contact1.x) > .01 || std::abs(cp.y - contact1.y) > .01) {
                    contact2 = cp;
                    contactCount = 2;
                }
            }
            else if (distSq < minDistSq) {
                minDistSq = distSq;
                contactCount = 1;
                contact1 = cp;
            }
        }
    }
}

// Polygon to Circle collision point
void Collisions::FindContactPoint(glm::vec2 circleCenter, float circleRadius, glm::vec2 polygonCenter, vector<glm::vec4> polygonVertices, glm::vec2& collisionPoint) {
    float minDistSq = FLT_MAX;
    for (int i = 0; i < polygonVertices.size(); ++i) {
        glm::vec2 va = polygonVertices[i];
        glm::vec2 vb = polygonVertices[(i + 1) % polygonVertices.size()];

        float distanceSquared;
        glm::vec2 contact;
        Collisions::PointSegmentDistance(circleCenter, va, vb, distanceSquared, contact);

        if (distanceSquared < minDistSq) {
            minDistSq = distanceSquared;
            collisionPoint = contact;
        }
    }
}

// Helper Function
void Collisions::PointSegmentDistance(glm::vec2 p, glm::vec2 a, glm::vec2 b, float& distanceSquared, glm::vec2& contact) {
    glm::vec2 ab = b - a;
    glm::vec2 ap = p - a;

    float proj = glm::dot(ap, ab);
    float abLenSq = glm::dot(ab, ab);
    float d = proj / abLenSq;

    if (d <= 0.0f) {
        contact = a;
    }
    else if (d >= 1.0f) {
        contact = b;
    }
    else {
        contact = a + (ab * d);
    }
    distanceSquared = glm::distance(p, contact);
    distanceSquared = (distanceSquared * distanceSquared);
}


// Circle to Circle collision point 
void Collisions::FindContactPoint(glm::vec2 centerA, float radiusA, glm::vec2 centerB, glm::vec2& contactPoint) {
    glm::vec2 ab = centerB - centerA;
    glm::vec2 dir = glm::normalize(ab);
    contactPoint = centerA + (dir * radiusA);
}

bool Collisions::IntersectAABBs(AABB a, AABB b) {
    if (a.max.x <= b.min.x || b.max.x <= a.min.x) {
        return true;
    }
    if (a.max.y <= b.min.y || b.max.y <= a.min.y) {
        return true;
    }
    return false;
}


