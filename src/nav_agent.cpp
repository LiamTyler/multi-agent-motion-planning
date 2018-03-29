#include "include/nav_agent.h"

NavAgent::NavAgent() {
    velocity = glm::vec3(0);
    active = false;
    currGoalNode = 0;
}

NavAgent::NavAgent(PRM* p, float s) {
    velocity = glm::vec3(0);
    active = false;
    currGoalNode = 0;
    prm = p;
    speed = s;
}

void NavAgent::SetGoal(glm::vec3 g) {
    goal = g;
}

bool NavAgent::FindPath() {
    active = false;
    path = prm->GeneratePath(gameObject->transform.position, goal);
    if (path.size()) {
        active = true;
        currGoalNode = 0;
    }
    return active;
}

void NavAgent::Update(float dt) {
    if (active) {
        glm::vec3 currPos = gameObject->transform.position;
        glm::vec3 diff = glm::normalize(path[currGoalNode] - currPos);
        glm::vec3 goalPos = path[currGoalNode];
        currPos += diff * dt * speed;
        gameObject->transform.position = currPos;
        if (glm::length(goalPos - currPos) < 0.05) {
            currGoalNode += 1;
            if (currGoalNode == path.size())
                active = false;
        }
    }
}
