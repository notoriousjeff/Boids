#ifndef BOID_H
#define BOID_H

#include "glm/glm.hpp"
#include <vector>
#include <random>

class Boid {
public:
    glm::vec3 position;
    glm::vec3 velocity;
    glm::vec3 acceleration;
    float maxSpeed;
    float maxForce;
    float areaSize;

    Boid(glm::vec3 pos, glm::vec3 vel, float maxSpeed, float maxForce, float areaSize)
        : position(pos), velocity(vel), maxSpeed(maxSpeed), maxForce(maxForce), areaSize(areaSize) {}

    void draw() const;
    void update(const std::vector<Boid>& boids, float deltaTime, float separationRadius, float alignmentRadius, float cohesionRadius);
    void applyForce(const glm::vec3& force);
    
    glm::vec3 separation(const std::vector<Boid>& boids, float separationRadius) const;
    glm::vec3 alignment(const std::vector<Boid>& boids, float alignmentRadius) const;
    glm::vec3 cohesion(const std::vector<Boid>& boids, float cohesionRadius) const;

    void handleCollision();
};


#endif
