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
        float ratio = i / (num_nodes - 1.f);
        Mass* mass = new Mass(start + ratio * diff, node_mass);
        masses.push_back(mass);
    }

    for (int n : fixed_nodes) {
        masses[n]->is_fixed = true;
    }

}

void Rope::linkSprings(float k) {

    if (masses.size() < 2) return;
    for (int i = 0; i < masses.size() - 1; ++i) {
        Spring* spring = new Spring(masses[i], masses[i+1], k);
        springs.push_back(spring);
    }

}

void Rope::simulateEuler(float delta_t, Eigen::Vector3f gravity, IntegrationMethod method) {

    // TODO: Task 2 - Implement Euler integration

    for (Spring* s : springs) {
        Eigen::Vector3f a = s->m1->position;
        Eigen::Vector3f b = s->m2->position;
        float ba_norm = (b - a).norm();
        Eigen::Vector3f fba = s->k * (b - a) * (ba_norm - s->rest_length) / ba_norm;
        if (!s->m1->is_fixed) {
            s->m1->force += fba;
        }
        if (!s->m2->is_fixed) {
            s->m2->force -= fba;
        }
    }
    
    if (method == IntegrationMethod::IMPLICIT) {
        for (Mass* m : masses) {
            if (m->is_fixed) continue;
            m->last_position = m->position;
            m->force -= damping_factor * m->velocity;
            Eigen::Vector3f accel = m->force / m->mass + gravity;
            m->velocity += accel * delta_t;
            m->position += m->velocity * delta_t;
            m->force = {0, 0, 0};
        }
        return;
    }
    
    for (Mass* m : masses) {
        if (m->is_fixed) continue;
        m->last_position = m->position;
        m->force -= damping_factor * m->velocity;
        Eigen::Vector3f accel = m->force / m->mass + gravity;
        m->position += m->velocity * delta_t;
        m->velocity += accel * delta_t;
        m->force = {0, 0, 0};
    }
}


void Rope::simulateVerlet(float delta_t, Eigen::Vector3f gravity) {

    for (Spring* s : springs) {
        Eigen::Vector3f a = s->m1->position;
        Eigen::Vector3f b = s->m2->position;
        float ba_norm = (b - a).norm();
        Eigen::Vector3f fba = s->k * (b - a) * (ba_norm - s->rest_length) / ba_norm;
        if (!s->m1->is_fixed) {
            s->m1->force += fba;
        }
        if (!s->m2->is_fixed) {
            s->m2->force -= fba;
        }
    }
    
    for (Mass* m : masses) {
        if (m->is_fixed) continue;
        Eigen::Vector3f accel = m->force / m->mass + gravity;
        Eigen::Vector3f pos = m->position;
        m->position += (1.f - damping_factor) * (m->position - m->last_position) + accel * delta_t * delta_t; 
        m->last_position = pos;
        m->force = {0, 0, 0};
    }
}