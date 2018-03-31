#pragma once

#include "include/Progression.h"
#include "include/prm.h"

class NavAgent : public Component {
    public:
        NavAgent();
        NavAgent(PRM* prm, float speed, float radius);

        void SetGoal(glm::vec3 g);
        bool FindPath();

        glm::vec3 GoalForce();
        glm::vec3 Separation();
        glm::vec3 Cohesion();
        glm::vec3 Alignment();

        void Update(float dt);
        void PostUpdate(float dt);
        void Start();
        void Stop();
        glm::vec3 Steer(glm::vec3 v);

        glm::vec3 GetPos() {
            glm::vec3 p = gameObject->transform.position;
            p.y = 0;
            return p;
        }
        
        glm::vec3 GetVel() { return velocity; }

        bool active;
        PRM* prm;
        glm::vec3 goal;
        glm::vec3 velocity;
        glm::vec3 steerForce;
        float maxForce;
        float maxSpeed;
        float radius;
        std::vector<glm::vec3> path;
        unsigned int currGoalNode;
};
