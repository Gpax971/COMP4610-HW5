#ifndef BOX_H
#define BOX_H

#include "collider.h"
#include <Eigen/Dense>
#include "mass.h"
#include <iostream>

class Box : public Collider {
public:
    Eigen::Vector3f center;  // Box center point
    Eigen::Vector3f dimensions;  // Dimensions for width, height, and depth

    Box(const Eigen::Vector3f& center, const Eigen::Vector3f& dimensions) 
        : center(center), dimensions(dimensions) {}

    bool checkCollision(Mass& mass) const override {
        Eigen::Vector3f position = mass.position;

        // TODO: Task 5 - Implement collision detection for a box

        return false;
    }

    void resolveCollision(Mass& mass) const override {

        // TODO: Task 5 - Resolve collision with a box

    }
};

#endif // BOX_H
