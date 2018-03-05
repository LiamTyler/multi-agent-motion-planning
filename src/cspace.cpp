#include "include/cspace.h"

CSpace::CSpace(Transform t, std::vector<GameObject*>* o, float extent) {
    transform_ = t;
    obstacles_ = o;
    extent_ = extent;
}

bool CSpace::InSpace(const glm::vec3& point) const {
    for (int i = 0; i < obstacles_->size(); i++) {
        glm::vec3 opoint = (*obstacles_)[i]->transform.position;
        float oradius = (*obstacles_)[i]->transform.scale.x + extent_;
        glm::vec3 dvec = point - opoint;
        dvec.y = 0;
        if (glm::length(dvec) <= oradius)
            return false;
    }
    return true;
}
