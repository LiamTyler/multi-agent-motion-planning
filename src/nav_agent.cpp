#include "include/nav_agent.h"

extern std::vector<NavAgent*> navAgentList;

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

    maxSpeed = 10;
    maxForce = 10;
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
    path = prm->GeneratePath(gameObject->transform.position, goal);
    if (path.size()) {
        active = true;
        currGoalNode = 0;
    }
    return active;
}

void NavAgent::Update(float dt) {
    steerForce = glm::vec3(0);
    if (active) {
        // steerForce += 1 * GoalForce();
        steerForce += 2 * Separation();
        steerForce += 1 * Cohesion();
        steerForce += 1 * Alignment();
    }
}

void NavAgent::PostUpdate(float dt) {
    velocity += steerForce * dt;
    if (glm::length(velocity) > maxSpeed)
        velocity = maxSpeed * glm::normalize(velocity);
    float heading = std::atan2(-velocity.z, velocity.x);
    gameObject->transform.rotation.y = M_PI + heading;

    gameObject->transform.position += velocity * dt;
    vec3 p = GetPos();
    float X = 16;
    float Z = 9;
    if (p.x < -X)
        gameObject->transform.position.x = X;
    else if (p.x > X)
        gameObject->transform.position.x = -X;
    if (p.z < -Z)
        gameObject->transform.position.z = Z;
    else if (p.z > Z)
        gameObject->transform.position.z = -Z;
}

glm::vec3 NavAgent::Steer(glm::vec3 target) {
    /*
    if (glm::length(v) > 0) {
        v = maxSpeed * normalize(v);
        v = v - velocity;
        if (glm::length(v) > maxForce)
            v = glm::normalize(v) * maxForce;
    } else {
        v = glm::vec3(0);
    }
    return v;
    */
    glm::vec3 desired = target - GetPos();
    desired = maxSpeed * normalize(desired);
    glm::vec3 sub = desired - velocity;
    if (glm::length(sub) > maxForce)
        sub = glm::normalize(sub) * maxForce;
    return sub;
}

glm::vec3 NavAgent::GoalForce() {
    glm::vec3 currPos = GetPos();
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
    if (count > 0)
        sum /= count;
    if (glm::length(sum) > 0) {
        sum = maxSpeed * glm::normalize(sum);
        sum -= velocity;
        if (glm::length(sum) > maxForce)
            sum = maxForce * glm::normalize(sum);
    }

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
    if (count) {
        sum /= count;
        return Steer(sum);
    } else {
        return glm::vec3(0);
    }
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
        sum /= count;
    if (glm::length(sum) > 0) {
        sum = maxSpeed * glm::normalize(sum);
        sum -= velocity;
        if (glm::length(sum) > maxForce)
            sum = maxForce * glm::normalize(sum);
    }

    return sum;
}
