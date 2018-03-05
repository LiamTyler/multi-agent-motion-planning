#pragma once

#include "include/Progression.h"
#include "include/cspace.h"

class Node {
    public:
        glm::vec3 position;
        std::vector<Node*> neighbors;
};

class PRM {
    public:
        PRM();
        ~PRM();
        void GeneratePRM(const CSpace& c, int samples, float neighbor_radius);
        std::vector<Node*> nodes_;
        RenderComponent* nodeRenderer;
};
