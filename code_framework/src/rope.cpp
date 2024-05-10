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

    Eigen::Vector3f diff = end - start;

    for (int i = 0; i < num_nodes; ++i) {
        float ratio = (float)i / (num_nodes - 1.f);
        Mass* mass = new Mass(start + ratio * diff, node_mass);
        masses.push_back(mass);
    }

    for (int n : fixed_nodes) {
        masses[n]->is_fixed = true;
    }

}

void Rope::linkSprings(float k) {

    if (masses.size() < 2) return;
    std::cout << "linking\n";
    for (int i = 0; i < masses.size() - 1; ++i) {
        Spring* spring = new Spring(masses[i], masses[i+1], k);
        springs.push_back(spring);
    }

}

void Rope::simulateEuler(float delta_t, Eigen::Vector3f gravity, IntegrationMethod method) {

    // TODO: Task 2 - Implement Euler integration
    for (auto i = masses.begin(); i < masses.end(); ++i) {
        Mass* m = *i;
        m->force = Eigen::Vector3f(0);
    }
    for (Spring* s : springs) {
        Eigen::Vector3f a = s->m1->position;
        Eigen::Vector3f b = s->m2->position;
        float ba_norm = (b - a).norm();
        Eigen::Vector3f fba = -s->k * (b - a) / ba_norm * (ba_norm - s->rest_length);
        s->m2->force += (s->m2->is_fixed) ? Eigen::Vector3f(0) : fba;
        // s->m1->force -= (s->m1->is_fixed) ? Eigen::Vector3f(0) : fba;
    }
    
    for (Mass* m : masses) {
        if (m->is_fixed) continue;
        Eigen::Vector3f accel = m->force / m->mass;// + gravity;
        Eigen::Vector3f nextVel = m->velocity + accel * delta_t;
        m->position = m->position + ((method == IntegrationMethod::IMPLICIT) ? nextVel : m->velocity) * delta_t;
        m->velocity = nextVel;
    }
}


void Rope::simulateVerlet(float delta_t, Eigen::Vector3f gravity) {

    // TODO: Task 3 - Implement Verlet integration
        
}