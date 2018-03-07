#include "include/prm.h"

PRM::PRM(CSpace* c, float r) {
    cspace = c;
    radius = r;
}

PRM::~PRM() {
    for (int i = 0; i < nodes_.size(); i++) {
        delete nodes_[i];
    }
}

void PRM::FindNeighbors(PRMNode* node) {
    for (int i = 0; i < nodes_.size(); i++) {
        PRMNode* neighbor = nodes_[i];
        if (neighbor == node)
            continue;
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

PRMNode* PRM::AddNode(glm::vec3 point) {
    if (cspace->InSpace(point)) {
        PRMNode* n = new PRMNode;
        n->position = point;
        FindNeighbors(n);
        nodes_.push_back(n);
        return n;
    } else {
        return nullptr;
    }
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
