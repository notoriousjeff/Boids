#include "boid.h"
#include "glm/glm.hpp"
#include "glm/gtc/constants.hpp"
#include "glut.h"
#include "glm/gtx/vector_angle.hpp"
#include <iostream>
#include <algorithm>


void Boid::draw() const {
    glPushMatrix();
    glTranslatef(position.x, position.y, position.z);
    
    // Rotate to match boid's velocity direction
    if (glm::length(velocity) > 0.001f) {
        glm::vec3 direction = glm::normalize(velocity);
        // Angle between Y-axis and velocity vector
        float angle = acos(direction.y);
        // Rotation axis
        glm::vec3 axis = glm::cross(glm::vec3(0.0f, 1.0f, 0.0f), direction);
        glRotatef(angle * 180.0f / glm::pi<float>(), axis.x, axis.y, axis.z);
    }

    glBegin(GL_TRIANGLES);
    // 3-sided pyramid
    // Base
    glColor3f(0.0f, 1.0f, 0.0f);    // Green
    glVertex3f(-0.5f, -0.5f, -0.5f);
    glVertex3f(0.5f, -0.5f, -0.5f);
    glVertex3f(0.0f, -0.5f, 0.5f);

    // Sides
    glColor3f(1.0f, 0.0f, 0.0f);    // Red
    glVertex3f(-0.5f, -0.5f, -0.5f);
    glVertex3f(0.5f, -0.5f, -0.5f);
    glVertex3f(0.0f, 0.5f, 0.0f);

    glColor3f(0.0f, 0.0f, 1.0f);    // Blue
    glVertex3f(0.5f, -0.5f, -0.5f);
    glVertex3f(0.0f, -0.5f, 0.5f);
    glVertex3f(0.0f, 0.5f, 0.0f);

    glColor3f(1.0f, 1.0f, 0.0f);    // Yellow
    glVertex3f(0.0f, -0.5f, 0.5f);
    glVertex3f(-0.5f, -0.5f, -0.5f);
    glVertex3f(0.0f, 0.5f, 0.0f);
    glEnd();

    glPopMatrix();
}

//Separation, Alignment, Cohesion
glm::vec3 Boid::separation(const std::vector<Boid>& boids, float radius) const {
    glm::vec3 steer(0.0f);
    int count = 0;

    for (const auto& other : boids) {
        float distance = glm::length(position - other.position);
        if (distance > 0 && distance < radius) {
            //exponential decay of separation force wrt distance
            glm::vec3 diff = (position - other.position) / (distance * distance + 0.01f);
            steer += diff;
            count++;
        }
    }

    if (count > 0) {
        steer /= (float)count;
    }

    if (glm::length(steer) > 0) {
        steer = glm::normalize(steer) * maxSpeed - velocity;
        steer = glm::length(steer) > maxForce ? glm::normalize(steer) * maxForce : steer;
    }
    
    return steer;
}

glm::vec3 Boid::alignment(const std::vector<Boid>& boids, float radius) const {
    glm::vec3 averageVelocity(0.0f);
    int count = 0;

    for (const auto& other : boids) {
        float distance = glm::length(position - other.position);
        if (distance > 0 && distance < radius) {
            averageVelocity += other.velocity;
            count++;
        }
    }

    if (count > 0) {
        averageVelocity /= (float)count;
        averageVelocity = glm::normalize(averageVelocity) * maxSpeed;
        glm::vec3 steer = averageVelocity - velocity;
        return glm::length(steer) > maxForce ? glm::normalize(steer) * maxForce : steer;
    }
    return glm::vec3(0.0f);
}

glm::vec3 Boid::cohesion(const std::vector<Boid>& boids, float radius) const {
    glm::vec3 averagePosition(0.0f);
    int count = 0;

    for (const auto& other : boids) {
        float distance = glm::length(position - other.position);
        if (distance > 0 && distance < radius) {
            averagePosition += other.position;
            count++;
        }
    }

    if (count > 0) {
        averagePosition /= (float)count;
        glm::vec3 desired = averagePosition - position;
        // Avoid normalization of zero vector
        if (glm::length(desired) > 0.0f) {
            desired = glm::normalize(desired) * maxSpeed;
        }
        glm::vec3 steer = desired - velocity;
        return glm::length(steer) > maxForce ? glm::normalize(steer) * maxForce : steer;
    }
    return glm::vec3(0.0f);
}

// Update the boid's state
void Boid::update(const std::vector<Boid>& boids, float deltaTime, float separationRadius, float alignmentRadius, float cohesionRadius) {
    // Reset acceleration to zero each frame to accumulate forces
    acceleration = glm::vec3(0.0f);

    // Apply behaviors and associated forces
    glm::vec3 separationForce = separation(boids, separationRadius);
    glm::vec3 alignmentForce = alignment(boids, alignmentRadius);
    glm::vec3 cohesionForce = cohesion(boids, cohesionRadius);
    applyForce(separationForce);
    applyForce(alignmentForce);
    applyForce(cohesionForce);

    // Update velocity and position based on the acceleration
    velocity += acceleration * deltaTime;
    if (glm::length(velocity) > maxSpeed) {
        velocity = glm::normalize(velocity) * maxSpeed;
    }
    position += velocity * deltaTime;

    // Literal edge case haha
    handleCollision();
}

// Helper to push it real good
void Boid::applyForce(const glm::vec3& force) {
    acceleration += force;
}

void Boid::handleCollision() {
    // Minimum velocity magnitude allowed after edge collision
    const float velocityThreshold = 0.1f;

    if (position.x > areaSize) {
        position.x = areaSize;                                   // Clamp position
        velocity.x = glm::max(-velocity.x, -velocityThreshold);  // Reflect and apply minimum vel threshold
    }
    else if (position.x < -areaSize) {
        position.x = -areaSize;
        velocity.x = glm::min(-velocity.x, velocityThreshold);   // Ditto, but opposite
    }

    if (position.y > areaSize) {
        position.y = areaSize;
        velocity.y = glm::max(-velocity.y, -velocityThreshold);
    }
    else if (position.y < -areaSize) {
        position.y = -areaSize;
        velocity.y = glm::min(-velocity.y, velocityThreshold);
    }

    if (position.z > areaSize) {
        position.z = areaSize;
        velocity.z = glm::max(-velocity.z, -velocityThreshold);
    }
    else if (position.z < -areaSize) {
        position.z = -areaSize;
        velocity.z = glm::min(-velocity.z, velocityThreshold);
    }

    // Clamp velocity magnitude to maxSpeed after collision
    if (glm::length(velocity) > maxSpeed) {
        velocity = glm::normalize(velocity) * maxSpeed;
    }
}