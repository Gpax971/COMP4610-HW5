#include "include/cloth.h"
#include <iostream>

Cloth::Cloth(const Eigen::Vector3f& center, const Eigen::Vector2f& size, int num_nodes_x, int num_nodes_z, float node_mass, float k, float damping_factor)
    : center(center), size(size), num_nodes_x(num_nodes_x), num_nodes_z(num_nodes_z), node_mass(node_mass), spring_constant(k), damping_factor(damping_factor) {
    initializeMasses();
    linkSprings();
}

void Cloth::initializeMasses() {
    // Calculate the top left and bottom right corners of the cloth
    Eigen::Vector3f top_left = Eigen::Vector3f(center.x() - size.x() / 2, center.y(), center.z() - size.y() / 2);
    Eigen::Vector3f bottom_right = Eigen::Vector3f(center.x() + size.x() / 2, center.y(), center.z() + size.y() / 2);


    // TODO: Task 4 - Initialize masses

}



void Cloth::linkSprings() {

    // TODO: Task 4 - Link springs

}


void Cloth::simulateVerlet(float delta_t, Eigen::Vector3f gravity) {
    // TODO: Task 5 - Implement Verlet integration with collision handling


    // After copying your verlet integration code from Task 3, use the following code for collision handling:

    // for (auto &mass : masses) {
    //     // Your other code here...

    //     for (auto collider : colliders) {
    //         if (collider->checkCollision(*mass)) {
    //             collider->resolveCollision(*mass); 
    //         }
    //     }
    // }

}

void Cloth::fixMass(int i) {

    if (i < 0 || i >= masses.size()) {
        std::cerr << "Invalid mass index" << std::endl;
        return;
    }
    
    masses[i]->is_fixed = true;
}

void Cloth::addCollider(Collider* collider) {
    colliders.push_back(collider);
}

