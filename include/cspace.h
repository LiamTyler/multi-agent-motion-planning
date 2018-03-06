#pragma once

#include "include/Progression.h"
#include <vector>

class CSpace {
    public:
        CSpace(Transform t, std::vector<GameObject*>* o, float extent);

        bool InSpace(const glm::vec3& point) const;
        bool ValidLine(const glm::vec3& start, const glm::vec3& end);

        std::vector<GameObject*>* obstacles_;
        Transform transform_;
        float extent_;
};
