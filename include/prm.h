#pragma once

#include "include/Progression.h"
#include "include/cspace.h"
#include "include/line_renderer.h"

class PRMNode;

class PRMNeighbor {
    public:
        PRMNode* node;
        float cost;
};

class PRMNode {
    public:
        glm::vec3 position;
        std::vector<PRMNeighbor*> neighbors;
};

class PRM {
    public:
        PRM(CSpace* c, float r);
        ~PRM();
        void GeneratePRM(int samples);
        PRMNode* AddNode(glm::vec3 point);
        void FindNeighbors(PRMNode* node);
        glm::vec3* GetLines() { return &lines_[0]; }
        int GetNumLines() { return lines_.size(); }
        std::vector<PRMNode*> nodes_;
        RenderComponent* nodeRenderer;
        LineRenderer* lineRenderer;

    protected:
        float radius;
        CSpace* cspace;
        std::vector<glm::vec3> lines_;
};
