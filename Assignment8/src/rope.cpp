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

//        Comment-in this part when you implement the constructor
//        for (auto &i : pinned_nodes) {
//            masses[i]->pinned = true;
//        }
        
        Vector2D segment = (end - start) / (num_nodes - 1.0f);
        Mass* mass = new Mass(start, node_mass, false);
        Mass* lastMass = mass;
        Spring* spring = nullptr;

        for (int i = 0; i < num_nodes; i++) {
            mass = new Mass(start + segment * i, node_mass, false);
            
            spring = new Spring(lastMass, mass, k);
            springs.push_back(spring);
            
            lastMass = mass;
            masses.push_back(mass);
        }

        for (auto &i : pinned_nodes) {
            masses[i]->pinned = true;
        }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            /*  
                f_b->a=-k_s*((b-a))/(||b-a||)*(||b-a||-l);
                F=ma;
                v(t+1)=v(t)+a(t)*dt
            */
            auto a = s->m1->position;
            auto b = s->m2->position;
            auto f = s->k * (b - a) / ((b - a).norm()) * ((b - a).norm() - s->rest_length);
            
            s->m1->forces += f;
            s->m2->forces -= f;


            // Internal Damping

            float k_d = 500;
            auto va = s->m1->velocity;
            auto vb = s->m2->velocity;
            auto fb = k_d * dot((b - a) / ((b - a).norm()), (vb - va)) * (b - a) / ((b - a).norm());

            s->m1->forces += fb;
            s->m2->forces -= fb;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                
                m->forces += gravity * m->mass;

                // TODO (Part 2): Add global damping
                float k_d_global = 0.01;
                m->forces -= k_d_global * m->velocity;

                auto a = m->forces / m->mass;
                m->position += m->velocity * delta_t;
                m->velocity += a * delta_t;
                
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateSemiImplicit(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            /*  
                f_b->a=-k_s*((b-a))/(||b-a||)*(||b-a||-l);
                F=ma;
                v(t+1)=v(t)+a(t)*dt
            */
            auto a = s->m1->position;
            auto b = s->m2->position;
            auto f = s->k * (b - a) / ((b - a).norm()) * ((b - a).norm() - s->rest_length);
            
            s->m1->forces += f;
            s->m2->forces -= f;


            // Internal Damping

            float k_d = 500;
            auto va = s->m1->velocity;
            auto vb = s->m2->velocity;
            auto fb = k_d * dot((b - a) / ((b - a).norm()), (vb - va)) * (b - a) / ((b - a).norm());

            s->m1->forces += fb;
            s->m2->forces -= fb;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                
                m->forces += gravity * m->mass;

                // TODO (Part 2): Add global damping
                float k_d_global = 0.01;
                m->forces -= k_d_global * m->velocity;

                auto a = m->forces / m->mass;
                m->velocity += a * delta_t;
                m->position += m->velocity * delta_t;                             
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
            auto a = s->m1->position;
            auto b = s->m2->position;
            auto f = s->k * (b - a) / ((b - a).norm()) * ((b - a).norm() - s->rest_length);
            
            s->m1->forces += f;
            s->m2->forces -= f;


            // Internal Damping

            float k_d = 500;
            auto va = s->m1->velocity;
            auto vb = s->m2->velocity;
            auto fb = k_d * dot((b - a) / ((b - a).norm()), (vb - va)) * (b - a) / ((b - a).norm());

            s->m1->forces += fb;
            s->m2->forces -= fb;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass
                /*
                    x(t+1) = x(t) + [x(t) - x(t-1)] + a(t) * dt * dt;
                */
                m->forces += gravity * m->mass;


                // TODO (Part 4): Add global Verlet damping
                /*
                    x(t+1) = x(t) + (1 - damping_factor) * [x(t) - x(t-1)] + a(t) * dt * dt;
                */
                auto a = m->forces / m->mass;
                float damping_factor = 0.000005f;
                m->position += (1 - damping_factor) * (m->position - m->last_position) + a * delta_t * delta_t;
                m->last_position = temp_position;
            }
            
            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }
}
