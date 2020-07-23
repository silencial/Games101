#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.
        Vector2D gap = (end - start) / (num_nodes - 1);
        Mass *prev = new Mass(start, node_mass, false);
        masses.emplace_back(prev);

        for (int i = 1; i < num_nodes; i++) {
            Mass *m = new Mass(start + i * gap, node_mass, false);
            masses.emplace_back(m);
            springs.emplace_back(new Spring(prev, m, k));
            prev = m;
        }
        // Comment-in this part when you implement the constructor
       for (auto &i : pinned_nodes) {
           masses[i]->pinned = true;
       }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            float l = s->rest_length;
            Vector2D diff = s->m2->position - s->m1->position;
            float l1 = diff.norm();
            Vector2D force = s->k * (diff / l1) * (l1 - l);
            s->m1->forces += force;
            s->m2->forces += -force;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                m->forces += gravity * m->mass;
                m->forces += -0.01 * m->velocity;

                Vector2D a = m->forces / m->mass;

                // Explicit Euler
                // m->position += m->velocity * delta_t;
                // m->velocity += a * delta_t;

                // Semi-implicit Euler
                m->velocity += a * delta_t;
                m->position += m->velocity * delta_t;

                // TODO (Part 2): Add global damping

            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            float l = s->rest_length;
            Vector2D diff = s->m2->position - s->m1->position;
            float l1 = diff.norm();
            Vector2D force = s->k * (diff / l1) * (l1 - l);
            s->m1->forces += force;
            s->m2->forces += -force;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass
                Vector2D last_positon = m->last_position;

                Vector2D a = m->forces / m->mass + gravity;

                // TODO (Part 4): Add global Verlet damping
                float damp = 0.00005;

                m->position = temp_position + (1 - damp) * (temp_position - last_positon) + a * delta_t * delta_t;
                m->last_position = temp_position;
            }
            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }
}
