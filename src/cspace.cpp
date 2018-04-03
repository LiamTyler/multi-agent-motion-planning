#include "include/cspace.h"

CSpace::CSpace(Transform t, std::vector<GameObject*>* o, float extent) {
    transform_ = t;
    obstacles_ = o;
    extent_ = extent;
}

bool CSpace::InSpace(const glm::vec3& point) const {
    float bx = transform_.scale.x / 2 - extent_;
    float bz = transform_.scale.z / 2 - extent_;
    if (point.x < -bx || point.x > bx || point.z < -bz || point.z > bz)
        return false;
    for (int i = 0; i < obstacles_->size(); i++) {
        glm::vec3 opoint = (*obstacles_)[i]->transform.position;
        opoint.y = 0;
        float oradius = (*obstacles_)[i]->transform.scale.x + extent_;
        glm::vec3 dvec = point - opoint;
        dvec.y = 0;
        if (glm::length(dvec) <= oradius)
            return false;
    }
    return true;
}

bool CSpace::IntersectsSphere(const glm::vec3& start, const glm::vec3& dir,
        const glm::vec3& opos, float r) {
    float t0 = -1, t1 = -1;
    glm::vec3 OC = start - opos;
    float b = 2*glm::dot(dir, OC);
    float c = glm::dot(OC, OC) - r*r;
    float disc = b*b - 4*c;
    if (disc < 0)
        return false;
    t0 = (-b + std::sqrt(disc)) / 2.0;
    t1 = (-b - std::sqrt(disc)) / 2.0;
    return (t0 > 0 || t1 > 0);
}

bool CSpace::ValidLine(const glm::vec3& start, const glm::vec3& end) {
    glm::vec3 dir = glm::normalize(end - start);
    for (int i = 0; i < obstacles_->size(); i++) {
        glm::vec3 opoint = (*obstacles_)[i]->transform.position;
        opoint.y = 0;
        float oradius = (*obstacles_)[i]->transform.scale.x + extent_;
        if (IntersectsSphere(start, dir, opoint, oradius))
            return false;
    }
    return true;
}
