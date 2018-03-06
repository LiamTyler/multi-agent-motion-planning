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

void PRM::FindNeighbors(Node* node) {
    for (int i = 0; i < nodes_.size(); i++) {
        Node* neighbor = nodes_[i];
        if (neighbor == node)
            continue;
        if (glm::length(node->position - neighbor->position) <= radius &&
            cspace->ValidLine(node->position, neighbor->position)) {
            node->neighbors.push_back(neighbor);
            neighbor->neighbors.push_back(node);
            glm::vec3 start = node->position;
            glm::vec3 end = neighbor->position;
            start.y = 0.01;
            end.y = 0.01;
            lines_.push_back(start);
            lines_.push_back(end);
        }
    }
}

bool PRM::AddNode(glm::vec3 point) {
    if (cspace->InSpace(point)) {
        Node* n = new Node;
        n->position = point;
        FindNeighbors(n);
        nodes_.push_back(n);
        return true;
    } else {
        return false;
    }
}

void PRM::GeneratePRM(int samples) {
    int numNodes = 0;
    nodes_.clear();
    nodes_.resize(samples);
    // find valid points
    while (numNodes < samples) {
        float rx = rand() / (float)RAND_MAX;
        float rz = rand() / (float)RAND_MAX;
        rx = (cspace->transform_.scale.x - 2*cspace->extent_) * (rx - .5);
        rz = (cspace->transform_.scale.z - 2*cspace->extent_) * (rz - .5);
        glm::vec3 point(rx, 0, rz);
        if (cspace->InSpace(point)) {
            Node* n = new Node;
            n->position = point;
            nodes_[numNodes] = n;
            numNodes++;
        }
    }

    // find neighbors
    for (int i = 0; i < samples; i++) {
        Node* node = nodes_[i];
        for (int ii = i + 1; ii < nodes_.size(); ii++) {
            Node* neighbor = nodes_[ii];
            if (glm::length(node->position - neighbor->position) <= radius &&
                cspace->ValidLine(node->position, neighbor->position)) {
                node->neighbors.push_back(neighbor);
                neighbor->neighbors.push_back(node);
                glm::vec3 start = node->position;
                glm::vec3 end = neighbor->position;
                start.y = 0.01;
                end.y = 0.01;
                lines_.push_back(start);
                lines_.push_back(end);
            }
        }
        FindNeighbors(node);
    }
}
