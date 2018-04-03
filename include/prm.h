#pragma once

#include "include/Progression.h"
#include "include/cspace.h"
#include "include/line_renderer.h"

class PRMNode;

class PRMNeighbor {
    public:
        PRMNeighbor() {
            node = nullptr;
            cost = 0;
        }
        PRMNeighbor(PRMNode* n, float c) {
            node = n;
            cost = c;
        }

        PRMNode* node;
        float cost;
};

class PRMNode {
    public:
        PRMNode() {
            position = glm::vec3(0);
        }
        PRMNode(glm::vec3 pos) {
            position = pos;
        }
        ~PRMNode() {
            for (auto& node : neighbors)
                delete node;
        }

        glm::vec3 position;
        std::vector<PRMNeighbor*> neighbors;
};

class AStarNode {
    public:
        AStarNode() {
            prmNode = nullptr;
            parent = nullptr;
        }
        AStarNode(PRMNode* pn, AStarNode* par) {
            prmNode = pn;
            parent = par;
        }

        bool operator==(const AStarNode& n) {
            return prmNode == n.prmNode;
        }

        PRMNode* prmNode;
        AStarNode* parent;
        float g;
        float f;
};

class PRM {
    public:
        PRM(CSpace* c, float r);
        ~PRM();
        void ClearPRM();
        void GeneratePRM(int samples);
        std::vector<glm::vec3> GeneratePath(glm::vec3 start, glm::vec3 goal);

        glm::vec3* GetLines() { return &lines_[0]; }
        int GetNumLines() { return lines_.size(); }

        std::vector<PRMNode*> nodes_;
        RenderComponent* nodeRenderer;
        LineRenderer* lineRenderer;
        CSpace* cspace;

    protected:
        // prm helpers
        void FindSearchNeighbors(PRMNode* node);
        PRMNode* AddSearchNode(glm::vec3 p);
        void RemoveSearchNode(PRMNode* n);
        void ClearSearch(PRMNode* start, PRMNode* end, std::vector<AStarNode*>& olist,
                std::vector<AStarNode*>& clist);

        // Astar helpers
        float Heuristic(const glm::vec3& point, const glm::vec3& goal);
        int InList(std::vector<AStarNode*>& list, AStarNode* node);
        int GetLowestCost(std::vector<AStarNode*>& list);
        std::vector<glm::vec3> ConstructPath(PRMNode* start, AStarNode* node);

        float radius;
        std::vector<glm::vec3> lines_;
};
