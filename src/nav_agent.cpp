#include "include/nav_agent.h"

extern std::vector<NavAgent*> navAgentList;
extern std::vector<GameObject*> obstacles;

NavAgent::NavAgent() {
    velocity = glm::vec3(0);
    active = false;
    currGoalNode = 0;
    radius = 1;
}

NavAgent::NavAgent(PRM* p, float s, float r) {
    prm = p;
    maxSpeed = s;
    radius = r;

    velocity = glm::vec3(0);
    active = false;
    currGoalNode = 0;

    maxSpeed = 5;
    maxForce = 4;
}

void NavAgent::Start() {
    float a = rand() / (float) RAND_MAX;
    a *= 2 * M_PI;
    velocity = glm::vec3(std::cos(a), 0, -std::sin(a));
}

void NavAgent::Stop() {
}

void NavAgent::SetGoal(glm::vec3 g) {
    goal = g;
}

bool NavAgent::FindPath() {
    active = false;
    path.clear();
    path = prm->GeneratePath(GetPos(), goal);
    if (path.size()) {
        active = true;
        currGoalNode = 0;
    } else {
        return false;
    }
    return active;
}

void NavAgent::Update(float dt) {
    steerForce = glm::vec3(0);
    if (active) {
        steerForce += 2.5 * GoalForce();
        steerForce += 12 * ObstacleAvoid();
        steerForce += 4 * Separation();
        steerForce += 1 * Cohesion();
        steerForce += 1 * Alignment();

        velocity += steerForce * dt;
        if (glm::length(velocity) > maxSpeed)
            velocity = maxSpeed * glm::normalize(velocity);
        float heading = std::atan2(-velocity.z, velocity.x);
        gameObject->transform.rotation.y = heading;

        gameObject->transform.position += velocity * dt;

        glm::vec3 p = GetPos();
        float BX = 1.1 * 16 - .5;
        float BZ = 1.1 * 9 - .5;
        if (p.x < -BX) {
           gameObject->transform.position.x = -BX;
        } else if (p.x > BX) {
           gameObject->transform.position.x = BX;
        }
        if (p.z < -BZ) {
           gameObject->transform.position.z = -BZ;
        } else if (p.z > BZ) {
           gameObject->transform.position.z = BZ;
        }
    }
}

glm::vec3 NavAgent::SteerToPoint(glm::vec3 target) {
    glm::vec3 desired = target - GetPos();
    desired = maxSpeed * normalize(desired);
    glm::vec3 sub = desired - velocity;
    if (glm::length(sub) > maxForce)
        sub = glm::normalize(sub) * maxForce;
    return sub;
}

glm::vec3 NavAgent::SteerToVelocity(glm::vec3 targetVel) {
    if (glm::length(targetVel) > 0) {
        targetVel = maxSpeed * glm::normalize(targetVel);
        targetVel -= velocity;
        if (glm::length(targetVel) > maxForce)
            targetVel = maxForce * glm::normalize(targetVel);
    }
    return targetVel;
}

glm::vec3 NavAgent::GoalForce() {
    glm::vec3 currPos = GetPos();

    // check to see if next node is visible
    if (currGoalNode != path.size() - 1) {
        if (prm->cspace->ValidLine(currPos, path[currGoalNode + 1])) {
            currGoalNode++;
        }
    }
    glm::vec3 goalPos = path[currGoalNode];
    glm::vec3 desiredVel = maxSpeed * glm::normalize(goalPos - currPos);
    if (glm::length(goalPos - currPos) < 0.1) {
        currGoalNode += 1;
        if (currGoalNode == path.size())
            active = false;
    }
    glm::vec3 force = (desiredVel - velocity);
    if (glm::length(force) > maxForce)
        force = maxForce * glm::normalize(force);

    return force;
}

glm::vec3 NavAgent::ObstacleAvoid() {
    float desiredSep = 1;
    glm::vec3 sum(0);
    int count = 0;
    glm::vec3 pos = GetPos();
    float rad = gameObject->transform.scale.x;
    for (int i = 0; i < obstacles.size(); i++) {
        glm::vec3 opos = obstacles[i]->transform.position;
        opos.y = 0;
        float r = obstacles[i]->transform.scale.x + rad;
        // add spring force if too close
        glm::vec3 diff = pos - opos;
        float l = glm::length(diff);
        if (l < r + desiredSep) {
            float denom = 0.5 * std::fmax(0.01, l);
            sum += glm::normalize(diff) / denom;
        }
    }
    if (count)
        sum = SteerToVelocity(sum / count);

    return sum;
}

glm::vec3 NavAgent::Separation() {
    float sep = 1.25;
    glm::vec3 sum(0);
    int count = 0;
    glm::vec3 pos = GetPos();
    for (auto& agent : navAgentList) {
        glm::vec3 diff = pos - agent->GetPos();
        float dist = glm::length(diff);
        if (dist > 0 && dist < sep) {
            count++;
            sum += normalize(diff) / dist;
        }
    }
    if (count)
        sum = SteerToVelocity(sum / count);
    
    return sum;
}

glm::vec3 NavAgent::Cohesion() {
    float rad = 2.5;
    glm::vec3 sum(0);
    int count = 0;
    glm::vec3 pos = GetPos();
    for (auto& agent : navAgentList) {
        glm::vec3 diff = pos - agent->GetPos();
        float dist = glm::length(diff);
        if (dist > 0 && dist < rad) {
            count++;
            sum += agent->GetPos();
        }
    }
    if (count)
        sum = SteerToPoint(sum / count);

    return sum;
}

glm::vec3 NavAgent::Alignment() {
    float rad = 2.5;
    glm::vec3 sum(0);
    int count = 0;
    glm::vec3 pos = GetPos();
    for (auto& agent : navAgentList) {
        glm::vec3 diff = pos - agent->GetPos();
        float dist = glm::length(diff);
        if (dist > 0 && dist < rad) {
            count++;
            sum += agent->GetVel();
        }
    }

    if (count)
        sum = SteerToVelocity(sum / count);
    
    return sum;
}
