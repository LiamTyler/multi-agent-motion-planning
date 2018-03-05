#include "include/prm.h"

PRM::PRM() {
}

PRM::~PRM() {
    for (int i = 0; i < nodes_.size(); i++) {
        delete nodes_[i];
    }
}

void PRM::GeneratePRM(const CSpace& c, int samples, float neighbor_radius) {
    int numNodes = 0;
    nodes_.clear();
    nodes_.resize(samples);
    // find valid points
    while (numNodes < samples) {
        float rx = rand() / (float)RAND_MAX;
        float rz = rand() / (float)RAND_MAX;
        rx = c.transform_.scale.x * (rx - .5);
        rz = c.transform_.scale.z * (rz - .5);
        glm::vec3 point(rx, 0, rz);
        if (c.InSpace(point)) {
            Node* n = new Node;
            n->position = point;
            nodes_[numNodes] = n;
            numNodes++;
        }
    }

    // find neighbors
    for (int i = 0; i < samples; i++) {
        Node* node = nodes_[i];
        for (int ii = i+1; ii < samples; ii++) {
            Node* neighbor = nodes_[ii];
            if (glm::length(node->position - neighbor->position) <= neighbor_radius) {
                node->neighbors.push_back(neighbor);
                neighbor->neighbors.push_back(node);
            }
        }
    }
    for (int i = 0; i < samples; i++) {
        Node* node = nodes_[i];
        std::cout << node->position << std::endl;
    }
}
