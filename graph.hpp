//
// Created by Rohan Behera on 11/27/20.
//

#ifndef GRAPHASSIGNMENT_GRAPH_HPP
#define GRAPHASSIGNMENT_GRAPH_HPP

#include <limits>
#include <string>
#include <assert.h>
#include <unordered_map>
#include <unordered_set>
#include <vector>

class Node;
class Edge;

class Graph {
public:
    constexpr static double INF = std::numeric_limits<double>::infinity();

    explicit Graph(bool directed);
    ~Graph();

    // Homework
    bool hasTripletClique() const;
    bool isConnected() const;
    double getMinDistance(const std::string &nid1, const std::string &nid2) const;
    // Optional, extra credit
    double getLongestSimplePath() const;

    // DO NOT CHANGE THE FOLLOWING METHODS
    bool isDirected() const;
    bool addNode(const std::string &nid);
    bool addEdge(const std::string &nid1, const std::string &nid2, double weight);
    std::string toString(const std::string &delimiter = "\n") const;
    void printGraph() const;
    size_t getNoOfNodes() const;
    size_t getNoOfEdges() const;
    void dfs();
    void recursive_dfs(Node* currNode, std::unordered_map<std::string, bool> & visited_list);
    Node* findNode(std::string id);
    std::vector<std::string> getAllNodes();
private:
    bool directed_;
    std::unordered_map<std::string, Node *> nodes_;
    std::unordered_set<Edge *> edges_;
//    std::vector<bool> visited;
    bool *visited;
    bool connected_;
    int *distance;
    double getLongestSimplePathHelper(const std::string &nid1) const;

};

#endif //GRAPHASSIGNMENT_GRAPH_HPP
