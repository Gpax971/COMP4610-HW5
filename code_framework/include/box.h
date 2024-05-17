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
        if (position.x() > center.x() + dimensions.x() / 2) return false;
        if (position.x() < center.x() - dimensions.x() / 2) return false;
        if (position.y() > center.y() + dimensions.y() / 2) return false;
        if (position.y() < center.y() - dimensions.y() / 2) return false;
        if (position.z() > center.z() + dimensions.z() / 2) return false;
        if (position.z() < center.z() - dimensions.z() / 2) return false;

        return true;
    }

    void resolveCollision(Mass& mass) const override {
        float top_x = center.x() + dimensions.x() / 2;
        float top_y = center.y() + dimensions.y() / 2;
        float top_z = center.z() + dimensions.z() / 2;
        float bottom_x = center.x() - dimensions.x() / 2;
        float bottom_y = center.y() - dimensions.y() / 2;
        float bottom_z = center.z() - dimensions.z() / 2;
        
        Eigen::Vector3f moveDir = mass.position - mass.last_position;

        if (mass.last_position.x() >= top_x && mass.position.x() <= top_x) {
            float t = (top_x - mass.last_position.x()) / moveDir.x();
            float y = t * moveDir.y() + mass.last_position.y();
            float z = t * moveDir.z() + mass.last_position.z();
            if (y < top_y && y > bottom_y && z < top_z && z > bottom_z) {
                // mass.position.x() = top_x;
                mass.position = {top_x, y, z};
                return;
            }
        }
        if (mass.last_position.x() <= bottom_x && mass.position.x() >= bottom_x) {
            float t = (bottom_x - mass.last_position.x()) / moveDir.x();
            float y = t * moveDir.y() + mass.last_position.y();
            float z = t * moveDir.z() + mass.last_position.z();
            if (y < top_y && y > bottom_y && z < top_z && z > bottom_z) {
                // mass.position.x() = bottom_x;
                mass.position = {bottom_x, y, z};
                return;
            }
        }
        if (mass.last_position.y() >= top_y && mass.position.y() <= top_y) {
            float t = (top_y - mass.last_position.y()) / moveDir.y();
            float x = t * moveDir.x() + mass.last_position.x();
            float z = t * moveDir.z() + mass.last_position.z();
            if (x < top_x && x > bottom_x && z < top_z && z > bottom_z) {
                // mass.position.y() = top_y;
                mass.position = {x, top_y, z};
                return;
            }
        }
        if (mass.last_position.y() <= bottom_y && mass.position.y() >= bottom_y) {
            float t = (bottom_y - mass.last_position.y()) / moveDir.y();
            float x = t * moveDir.x() + mass.last_position.x();
            float z = t * moveDir.z() + mass.last_position.z();
            if (x < top_x && x > bottom_x && z < top_z && z > bottom_z) {
                // mass.position.y() = bottom_y;
                mass.position = {x, bottom_y, z};
                return;
            }
        }
        if (mass.last_position.z() >= top_z && mass.position.z() <= top_z) {
            float t = (top_z - mass.last_position.z()) / moveDir.z();
            float x = t * moveDir.x() + mass.last_position.x();
            float y = t * moveDir.y() + mass.last_position.y();
            if (x < top_x && x > bottom_x && y < top_y && y > bottom_y) {
                // mass.position.z() = top_z;
                mass.position = {x, y, top_z};
                return;
            }
        }
        if (mass.last_position.z() <= bottom_z && mass.position.z() >= bottom_z) {
            float t = (bottom_z - mass.last_position.z()) / moveDir.z();
            float x = t * moveDir.x() + mass.last_position.x();
            float y = t * moveDir.y() + mass.last_position.y();
            if (x < top_x && x > bottom_x && y < top_y && y > bottom_y) {
                // mass.position.z() = bottom_z;
                mass.position = {x, y, bottom_z};
                return;
            }
        }
        mass.position = mass.last_position;
    }
};

#endif // BOX_H
