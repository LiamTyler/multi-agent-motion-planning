#include "include/prm.h"
#include <stack>

PRM::PRM(CSpace* c, float r) {
    cspace = c;
    radius = r;
}

PRM::~PRM() {
    for (int i = 0; i < nodes_.size(); i++) {
        delete nodes_[i];
    }
}

void PRM::ClearPRM() {
    for (int i = 0; i < nodes_.size(); i++) {
        delete nodes_[i];
    }
    lines_.clear();
}

void PRM::GeneratePRM(int samples) {
    int numPRMNodes = 0;
    nodes_.clear();
    nodes_.resize(samples);
    // find valid points
    while (numPRMNodes < samples) {
        float rx = rand() / (float)RAND_MAX;
        float rz = rand() / (float)RAND_MAX;
        rx = (cspace->transform_.scale.x - 2*cspace->extent_) * (rx - .5);
        rz = (cspace->transform_.scale.z - 2*cspace->extent_) * (rz - .5);
        glm::vec3 point(rx, 0, rz);
        if (cspace->InSpace(point)) {
            PRMNode* n = new PRMNode;
            n->position = point;
            nodes_[numPRMNodes] = n;
            numPRMNodes++;
        }
    }

    // find neighbors
    for (int i = 0; i < samples; i++) {
        PRMNode* node = nodes_[i];
        for (int ii = i + 1; ii < nodes_.size(); ii++) {
            PRMNode* neighbor = nodes_[ii];
            if (glm::length(node->position - neighbor->position) <= radius &&
                cspace->ValidLine(node->position, neighbor->position)) {
                glm::vec3 start = node->position;
                glm::vec3 end = neighbor->position;
                PRMNeighbor* n1 = new PRMNeighbor;
                PRMNeighbor* n2 = new PRMNeighbor;
                n1->cost = glm::length(end - start);
                n2->cost = glm::length(end - start);
                n1->node = neighbor;
                n2->node = node;
                node->neighbors.push_back(n1);
                neighbor->neighbors.push_back(n2);
                start.y = 0.01;
                end.y = 0.01;
                lines_.push_back(start);
                lines_.push_back(end);
            }
        }
    }
}

float PRM::Heuristic(const glm::vec3& point, const glm::vec3& goal) {
    return glm::length(goal - point);
}

int PRM::InList(std::vector<AStarNode*>& list, AStarNode* node) {
    for (int i = 0; i < list.size(); i++) {
        if (*list[i] == *node)
            return i;
    }
    return -1;
}

int PRM::GetLowestCost(std::vector<AStarNode*>& list) {
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

std::vector<glm::vec3> PRM::ConstructPath(PRMNode* start, AStarNode* node) {
    std::stack<PRMNode*> reversePath;
    while (node->parent != nullptr) {
        reversePath.push(node->prmNode);
        node = node->parent;
    }
    std::vector<glm::vec3> path;
    while (reversePath.size()) {
        path.push_back(reversePath.top()->position);
        reversePath.pop();
    }
    return path;
}

void PRM::FindSearchNeighbors(PRMNode* node) {
    for (int i = 0; i < nodes_.size(); i++) {
        PRMNode* neighbor = nodes_[i];
        if (neighbor == node)
            continue;
        glm::vec3 start = node->position;
        glm::vec3 end = neighbor->position;
        float l = glm::length(end - start);
        if (l <= radius && cspace->ValidLine(start, end)) {
            PRMNeighbor* n = new PRMNeighbor(neighbor, l);
            node->neighbors.push_back(n);
            neighbor->neighbors.push_back(new PRMNeighbor(node, l));
        }
    }
}

PRMNode* PRM::AddSearchNode(glm::vec3 p) {
    PRMNode* node = new PRMNode(p);
    FindSearchNeighbors(node);
    return node;
}

void PRM::RemoveSearchNode(PRMNode* n) {
    for (auto& neighbor : n->neighbors) {
        neighbor->node->neighbors.erase(neighbor->node->neighbors.end() - 1);
    }
    delete n;
}

void PRM::ClearSearch(PRMNode* start, PRMNode* end, std::vector<AStarNode*>& olist, 
        std::vector<AStarNode*>& clist) {
    RemoveSearchNode(start);
    RemoveSearchNode(end);
    for (auto& n: olist)
        delete n;
    for (auto& n: clist)
        delete n;
}

std::vector<glm::vec3> PRM::GeneratePath(glm::vec3 start, glm::vec3 end) {
    PRMNode* startPRMNode = AddSearchNode(start);
    PRMNode* endPRMNode = AddSearchNode(end);
    if (startPRMNode->neighbors.size() == 0)
        std::cout << "no neighbors for A*" << std::endl;

    AStarNode* startNode = new AStarNode(startPRMNode, nullptr);
    std::vector<AStarNode*> openList;
    std::vector<AStarNode*> closedList;
    openList.push_back(startNode);
    startNode->g = 0;
    startNode->f = startNode->g + Heuristic(start, end);
    while (openList.size() != 0) {
        int index = GetLowestCost(openList);
        AStarNode* curr = openList[index];
        if (curr->prmNode == endPRMNode) {
            if (startPRMNode->neighbors.size() == 0)
                std::cout << "no neighbors for A*, but path found" << std::endl;
            std::vector<glm::vec3> path = ConstructPath(startPRMNode, curr);
            ClearSearch(startPRMNode, endPRMNode, openList, closedList);
            return path;
        }
        openList.erase(openList.begin() + index);
        closedList.push_back(curr);
        for (auto& neighbor : curr->prmNode->neighbors) {
            AStarNode* n = new AStarNode;
            n->prmNode = neighbor->node;
            n->g = curr->g + neighbor->cost;
            n->parent = curr;
            // if neighbor in closed list
            if (InList(closedList, n) != -1) {
                delete n;
                continue;
            }
            int index = InList(openList, n);
            // if neighbor not in openlist
            if (index == -1) {
                n->f = n->g + Heuristic(n->prmNode->position, end);
                openList.push_back(n);
            } else {
                AStarNode* oListNeighbor = openList[index];
                if (n->g < oListNeighbor->g) {
                    oListNeighbor->g = n->g;
                    oListNeighbor->f = n->g + Heuristic(n->prmNode->position, end);
                    oListNeighbor->parent = curr;
                }
            }
        }
    }
    ClearSearch(startPRMNode, endPRMNode, openList, closedList);
    std::vector<glm::vec3> empty;
    return empty;
}
