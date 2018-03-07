#include "include/path.h"
#include <stack>

PathNode::PathNode() : PathNode(nullptr, nullptr) {}

PathNode::PathNode(PRMNode* pn, PathNode* par) {
    prmNode = pn;
    parent = par;
    g = 0;
    f = 0;
}

bool PathNode::operator==(const PathNode& pn) {
    // return prmNode->position == pn.prmNode->position;
    return prmNode == pn.prmNode;
}

Path::Path(CSpace* c, PRM* p) {
    cspace_ = c;
    prm_ = p;
}

float Path::Heuristic(const glm::vec3& point, const glm::vec3& goal) {
    return glm::length(goal - point);
}

int Path::InList(std::vector<PathNode*>& list, PathNode* node) {
    for (int i = 0; i < list.size(); i++) {
        if (*list[i] == *node)
            return i;
    }
    return -1;
}

int Path::GetLowestCost(std::vector<PathNode*>& list) {
    int lowest_cost = list[0]->f;
    int lowest_cost_index = 0;
    for (int i = 1; i < list.size(); i++) {
        if (list[i]->f < lowest_cost) {
            lowest_cost = list[i]->f;
            lowest_cost_index = i;
        }
    }
    return lowest_cost_index;
}

void Path::ConstructPath(PRMNode* start, PathNode* node) {
    std::stack<PRMNode*> reversePath;
    while (node->parent != nullptr) {
        reversePath.push(node->prmNode);
        node = node->parent;
    }
    while (reversePath.size()) {
        path_.push_back(reversePath.top());
        reversePath.pop();
    }
}

bool Path::GeneratePath(glm::vec3 start, glm::vec3 end) {
    PRMNode* startPRMNode = prm_->AddNode(start);
    PRMNode* endPRMNode = prm_->AddNode(end);
    PathNode* startNode = new PathNode(startPRMNode, nullptr);
    std::vector<PathNode*> openList;
    std::vector<PathNode*> closedList;
    openList.push_back(startNode);
    startNode->g = 0;
    startNode->f = startNode->g + Heuristic(start, end);
    while (openList.size() != 0) {
        int index = GetLowestCost(openList);
        PathNode* curr = openList[index];
        if (curr->prmNode == endPRMNode) {
            ConstructPath(startPRMNode, curr);
            return true;
        }
        openList.erase(openList.begin() + index);
        closedList.push_back(curr);
        for (auto& neighbor : curr->prmNode->neighbors) {
            PathNode* n = new PathNode;
            n->prmNode = neighbor->node;
            n->g = curr->g + neighbor->cost;
            n->parent = curr;
            // if neighbor in closed list
            if (InList(closedList, n) != -1)
                continue;
            int index = InList(openList, n);
            // if neighbor not in openlist
            if (index == -1) {
                n->f = n->g + Heuristic(n->prmNode->position, end);
                openList.push_back(n);
            } else {
                PathNode* oListNeighbor = openList[index];
                if (n->g < oListNeighbor->g) {
                    oListNeighbor->g = n->g;
                    oListNeighbor->f = n->g + Heuristic(n->prmNode->position, end);
                    oListNeighbor->parent = curr;
                }
            }
        }
    }
    return false;
}
