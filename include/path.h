#pragma once

#include "include/Progression.h"
#include "include/cspace.h"
#include "include/prm.h"

class PathNode {
    public:
        PathNode();
        PathNode(PRMNode*, PathNode* parent);

        bool operator==(const PathNode& pn);

        PRMNode* prmNode;
        PathNode* parent;
        float g;
        float f;
};

class Path {
    public:
        Path(CSpace* cspace, PRM* prm);
        int InList(std::vector<PathNode*>& list, PathNode* node);
        int GetLowestCost(std::vector<PathNode*>& list);
        float Heuristic(const glm::vec3& point, const glm::vec3& goal);
        bool GeneratePath(glm::vec3 start, glm::vec3 finish);
        void ConstructPath(PRMNode* start, PathNode* node);

        std::vector<PRMNode*> path_;
        CSpace* cspace_;
        PRM* prm_;
};
