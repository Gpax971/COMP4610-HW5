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
    float diff_x = top_left.x() - bottom_right.x();
    float diff_z = top_left.z() - bottom_right.z();

    for (int z = 0; z < num_nodes_z; ++z) {
        float ratio_z = z / (num_nodes_z - 1.f);
        for (int x = 0; x < num_nodes_x; ++x) {
            float ratio_x = x / (num_nodes_x - 1.f);
            Eigen::Vector3f ratio = {ratio_x, 1, ratio_z};
            Mass* mass = new Mass(top_left + ratio.cwiseProduct(bottom_right - top_left), node_mass, false);
            masses.push_back(mass);
        }
    }
}



void Cloth::linkSprings() {

    // TODO: Task 4 - Link springs
    // Structural Springs
    for (int x = 0; x < num_nodes_x - 1; ++x) {
        for (int z = 0; z < num_nodes_z - 1; ++z) {
            springs.push_back(new Spring(masses[getIndex(x, z)], masses[getIndex(x + 1, z)], spring_constant));
            springs.push_back(new Spring(masses[getIndex(x, z)], masses[getIndex(x, z + 1)], spring_constant));
            // Shear Springs
            springs.push_back(new Spring(masses[getIndex(x, z + 1)], masses[getIndex(x + 1, z)], spring_constant));
            springs.push_back(new Spring(masses[getIndex(x, z)], masses[getIndex(x + 1, z + 1)], spring_constant));
        } 
        springs.push_back(new Spring(masses[getIndex(x, num_nodes_z - 1)], masses[getIndex(x + 1,num_nodes_z - 1)], spring_constant));
    }
    for (int z = 0; z < num_nodes_z - 1; ++z) {
        springs.push_back(new Spring(masses[getIndex(num_nodes_x - 1, z)], masses[getIndex(num_nodes_x - 1, z + 1)], spring_constant));
    }

    // Flexion Springs
    for (int x = 0; x < num_nodes_x - 2; ++x) {
        for (int z = 0; z < num_nodes_z - 2; ++z) {
            springs.push_back(new Spring(masses[getIndex(x, z)], masses[getIndex(x + 2, z)], spring_constant));
            springs.push_back(new Spring(masses[getIndex(x, z)], masses[getIndex(x, z + 2)], spring_constant));
        } 
        springs.push_back(new Spring(masses[getIndex(x, num_nodes_z - 1)], masses[getIndex(x + 2, num_nodes_z - 1)], spring_constant));
    }
    for (int z = 0; z < num_nodes_z - 2; ++z) {
        springs.push_back(new Spring(masses[getIndex(num_nodes_x - 1, z)], masses[getIndex(num_nodes_x - 1, z + 2)], spring_constant));
    }
}


void Cloth::simulateVerlet(float delta_t, Eigen::Vector3f gravity) {
    // TODO: Task 5 - Implement Verlet integration with collision handling

    for (Mass* m : masses) {
        m->force = {0, 0, 0};
    }

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

        for (Collider* collider : colliders) {
            if (collider->checkCollision(*m)) {
                collider->resolveCollision(*m); 
            }
        }
    }
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

int Cloth::getIndex(int x, int z) {
    return z * num_nodes_x + x;
}