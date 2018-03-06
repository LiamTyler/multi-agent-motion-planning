#pragma once

#include "include/Progression.h"
#include "include/cspace.h"
#include "include/line_renderer.h"

class Node {
    public:
        glm::vec3 position;
        std::vector<Node*> neighbors;
};

class PRM {
    public:
        PRM(CSpace* c, float r);
        ~PRM();
        void GeneratePRM(int samples);
        bool AddNode(glm::vec3 point);
        void FindNeighbors(Node* node);
        glm::vec3* GetLines() { return &lines_[0]; }
        int GetNumLines() { return lines_.size(); }
        std::vector<Node*> nodes_;
        RenderComponent* nodeRenderer;
        LineRenderer* lineRenderer;

    protected:
        float radius;
        CSpace* cspace;
        std::vector<glm::vec3> lines_;
};
