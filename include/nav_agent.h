#pragma once

#include "include/Progression.h"
#include "include/prm.h"

class NavAgent : public Component {
    public:
        NavAgent();
        NavAgent(PRM* prm, float speed);

        void SetGoal(glm::vec3 g);
        bool FindPath();
        void Update(float dt);
        void Start() {}
        void Stop() {}

        bool active;
        PRM* prm;
        glm::vec3 goal;
        glm::vec3 velocity;
        float speed;
        std::vector<glm::vec3> path;
        unsigned int currGoalNode;
};
