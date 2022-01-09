//
// Created by Rohan Behera on 11/27/20.
//

#ifndef GRAPHASSIGNMENT_EDGE_HPP
#define GRAPHASSIGNMENT_EDGE_HPP

class Node;

class Edge {
    int targetNode;
    int distanceFromNode;

public:
    explicit Edge(Node* node, double weight) : node_(node), weight_(weight) {
        assert(node != nullptr);
    }
    Edge(int targetNode, int distance) {
        this->targetNode = targetNode;
        this->distanceFromNode = distance;
    }
    ~Edge() {}

    inline void setWeight(double weight) { weight_ = weight; }

    inline Node* getNode() const { return node_; }

    inline double getWeight() const { return weight_; }

private:
    Node* node_;
    double weight_;
};

#endif //GRAPHASSIGNMENT_EDGE_HPP
