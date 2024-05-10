#include <iostream>
#include <vector>
#include "Eigen/Dense"

#include "include/rope.h"
#include "include/mass.h"
#include "include/spring.h"

Rope::Rope(Eigen::Vector3f start, Eigen::Vector3f end, int num_nodes, float node_mass, float k, float d, std::vector<int> fixed_nodes)
    : damping_factor(d) {
    initializeMasses(start, end, num_nodes, node_mass, fixed_nodes);
    linkSprings(k);
}

void Rope::initializeMasses(Eigen::Vector3f start, Eigen::Vector3f end, int num_nodes, float node_mass, std::vector<int> fixed_nodes) {

    // TODO: Task 1 - Initialize masses

}

void Rope::linkSprings(float k) {

    // TODO: Task 1 - Link springs

}

void Rope::simulateEuler(float delta_t, Eigen::Vector3f gravity, IntegrationMethod method) {

    // TODO: Task 2 - Implement Euler integration

}


void Rope::simulateVerlet(float delta_t, Eigen::Vector3f gravity) {

    // TODO: Task 3 - Implement Verlet integration
        
}