//
// Created by Rohan Behera on 11/27/20.
//

#include "graph.hpp"

#include <algorithm>
#include <cstdio>
#include <iostream>
#include <sstream>
#include <vector>
#include <queue>
#include <list>
#include<iterator>
#include "edge.hpp"
#include "node.hpp"

using namespace std;

// Complete this
bool Graph::hasTripletClique() const {
    if (nodes_.size() < 3) return false;
    // To do
    /**
     * iterate through the unordered set of nodes in the Graph class in a for loop,
     * then a nested for loop that iterates through the current node (node1)'s neighbor list,
     * then another nested for loop that iterates through that node2's neighbor list, then in
     * that third for loop it checks if node3 == node1, returns true if found, the function returns
     * false once it exits the for loop
     */
    for (auto it1 = nodes_.begin(); it1 != nodes_.end(); ++it1) {
        std::string key1 = it1->first;
        std::set<Node *> node1_neighbours = it1->second->getNeighbors();
        for (std::set<Node*>::const_iterator it2 = node1_neighbours.begin(); it2 != node1_neighbours.end(); ++it2) {
            std::string key2 = (*it2)->getID();
            std::set<Node*> node2_neighbours = (*it2)->getNeighbors();
            for (std::set<Node*>::const_iterator it3 = node2_neighbours.begin(); it3 != node2_neighbours.end(); ++it3) {
                std::string key3 = (*it3)->getID();
                if (key1 == key3) {
                    return true;
                }
                else return false;
            }
        }
    }
    return false;
}

// Complete this
bool Graph::isConnected() const {
    return connected_;
}

/**
 * Checks if all the nodes in the graph is connected
 * @param s
 */
void Graph::dfs() {
    size_t max_nodes = getNoOfNodes();
  //  std::cout << "inside dfs " << max_nodes << std::endl;
    int ctr = 0;
    std::unordered_map<std::string, bool> visited_list(max_nodes);
    for(auto it : nodes_) {
        //it.second->setVisited(false);
        visited_list.emplace(it.first, false);
    }
    // std::cout << "First Node: " << nodes_.at(0)->getID() << std::endl;
  //  std::cout << " after dfs before connected check " << std::endl;
    Node* firstNode = nodes_.begin()->second;
    recursive_dfs(firstNode, visited_list);

    // if any of the value in the visited_list is false set connected_ = false, otherwise set to true
    int connected_ctr = 0;
    for (auto it = visited_list.begin(); it != visited_list.end(); ++it){
        if (it->second == true) {
            connected_ctr++;
        }
    }
    if (max_nodes == connected_ctr)
        connected_ = true;
    else connected_ = false;
    firstNode = NULL;
    visited_list.clear();
}

void Graph::recursive_dfs(Node *currNode, std::unordered_map<std::string, bool> &visited_list) {
   // std::cout << "inside recursive_dijkstra " << std::endl;
    int ctr = 0;

        auto it1 = visited_list.find(currNode->getID());
        if (it1 != visited_list.end()) {
         //   std::cout << "recursive_dijkstra::currNode::currentNode::id=" << it1->first << std::endl;
            it1->second = true;
        }

        std::set<Edge*> adjacencyList;
        adjacencyList =  currNode->getAdjacencyList();
      //  std::cout << "recursive_dijkstra::currNode->getID(): " << currNode->getID() << std::endl;
        for (auto it = adjacencyList.begin(); it != adjacencyList.end(); ++it) {
      //      std::cout << "recursive_dijkstra::adj node id: " << (*it)->getNode()->getID() << std::endl;
            auto it2 = visited_list.find((*it)->getNode()->getID());
            if (it2 != visited_list.end()) {
         //       std::cout << "recursive_dijkstra::adjNode::currentNode::id=" << it1->first << std::endl;
                bool adj_visited = (*it2).second;
                if (!adj_visited) {
          //          std::cout << "adj visited check: " << adj_visited << std::endl;
                    recursive_dfs((*it)->getNode(), visited_list);
                }
            }
        }
        adjacencyList.clear();
   // std::cout << "exiting recursive_dijkstra " << std::endl;
}

// Complete this
double Graph::getMinDistance(const std::string &nid1,
                             const std::string &nid2) const {
    assert(nodes_.size() >= 2);  // Must have at least 2 nodes
    // To do
    Node* src = nodes_.at(nid1);
    Node* dest = nodes_.at(nid2);

    int source_dist =  std::distance(nodes_.begin(),nodes_.find(nid1));
    int destination_dist =  std::distance(nodes_.begin(),nodes_.find(nid2));

    if (source_dist == destination_dist) {
        return 0;
    }

    int max_nodes = getNoOfNodes();

    bool* visited = new bool[max_nodes];
    // set source node with infinity distance
    // except for the initial node and mark
    // them unvisited.
    for(int i = 0; i < max_nodes; i++)
    {
        visited[i] = false;
    }
    // Distance of source vertex from itself is always 0
    double min_distance = 0;
    std::set<Node*> neighbours = src->getNeighbors();
    for (auto it = neighbours.begin(); it != neighbours.end(); ++it) {
        double new_dist =  std::distance(nodes_.begin(),nodes_.find((*it)->getID()));
        std::set<Edge*> adjacencyList =  src->getAdjacencyList();

        for (auto it = adjacencyList.begin(); it != adjacencyList.end(); ++it) {
            double weight = 0;
            if ((*it)->getNode()->getID() == dest->getID()) {
               weight = (*it)->getWeight();
               new_dist = new_dist+weight;
            }
        }
        if (new_dist < source_dist) {
            min_distance = new_dist;
        }
    }
    return min_distance;
}


// Optional: complete this
double Graph::getLongestSimplePath() const {
    assert(nodes_.size() >= 1);  // Must have at least 1 node
    // To do
    getLongestSimplePathHelper(nodes_.begin()->first);
    return 0.0;
}

double Graph::getLongestSimplePathHelper(const std::string &nid1) const {
    Node* src = nodes_.at(nid1);
    int source_dist =  std::distance(nodes_.begin(),nodes_.find(nid1));
    int max_nodes = getNoOfNodes();

    bool* visited = new bool[max_nodes];
    // set source node with infinity distance
    // except for the initial node and mark
    // them unvisited.
    for(int i = 0; i < max_nodes; i++)
    {
        visited[i] = false;
    }
    // Distance of source vertex from itself is always 0
    double longestDistance = 0;
    std::set<Node*> neighbours = src->getNeighbors();
    for (auto it = neighbours.begin(); it != neighbours.end(); ++it) {
        int new_dist =  std::distance(nodes_.begin(),nodes_.find((*it)->getID()));
        // find weight of the edge that connects source to it(neighbor)
        // how to find the edge between the two nodes
        std::set<Edge*> adjacencyList =  src->getAdjacencyList();
        //Edge* edge = adjacencyList. .find(dest->getID());
        for (auto it = adjacencyList.begin(); it != adjacencyList.end(); ++it) {
            double weight = 0;
            if ((*it)->getNode()->getID() == src->getID()) {
                weight = (*it)->getWeight();
                new_dist = new_dist+weight;
            }
        }
        if (new_dist > source_dist) {
            longestDistance = new_dist;
        }
    }
    return longestDistance;
}


Graph::Graph(bool directed) : directed_(directed){};

Graph::~Graph() {
    for (Edge *edge : edges_) delete edge;
    edges_.clear();
    for (auto it : nodes_) delete it.second;
    nodes_.clear();
}

bool Graph::isDirected() const {
    return directed_;
}

bool Graph::addNode(const std::string &nid) {
  //  cout << "In Graph::addNode(): " << endl;
    Node *node = new Node(nid);
    bool inserted = nodes_.emplace(nid, node).second;
    if (!inserted) delete node;
   // cout << "Node added with id: " << node->getID() << endl;
    return inserted;
}

bool Graph::addEdge(const std::string &nid1, const std::string &nid2,
                    double weight) {
   // std::cout << "Start addEdge()" << std::endl;
    auto it1 = nodes_.find(nid1);
    auto it2 = nodes_.find(nid2);
    if (it1 == nodes_.end() || it2 == nodes_.end()) return false;

    Node *node1 = it1->second;
    Node *node2 = it2->second;

    Edge *edge1 = new Edge(node2, weight);
    node1->addEdge(edge1);
    edges_.insert(edge1);
    if (!directed_) {
        Edge *edge2 = new Edge(node1, weight);
        node2->addEdge(edge2);
        edges_.insert(edge2);
    }
  //  std::cout << "End addEdge()" << std::endl;
    return true;
}

std::string Graph::toString(const std::string &delimiter) const {
    std::vector<std::string> nids;
    nids.reserve(nodes_.size());
    for (auto it : nodes_) nids.push_back(it.first);
    std::sort(nids.begin(), nids.end());

    std::stringstream ss;
    // Iterate each node
    for (size_t i = 0; i < nids.size(); i++) {
        if (i > 0) ss << delimiter;
        ss << nids[i] << ": [";
        Node *node = nodes_.find(nids[i])->second;
        bool first = true;
        for (Node *neighbor : node->getNeighbors()) {
            if (first) {
                ss << neighbor->getID();
                first = false;
            } else {
                ss << ", " << neighbor->getID();
            }
        }
        ss << "]";
    }
    return ss.str();
}

void Graph::printGraph() const {
    std::cout << "Printing graph: " << std::endl;
    std::cout << toString() << std::endl;
}

size_t Graph::getNoOfNodes() const {
    return nodes_.size();
}
size_t Graph::getNoOfEdges() const {
    return edges_.size();
}

Node* Graph::findNode(std::string id) {
    return nodes_.at(id);
}

std::vector<std::string> Graph::getAllNodes() {
    std::vector<std::string> keys(nodes_.size());
    for (auto it = nodes_.begin(); it != nodes_.end(); ++it){
        keys.push_back(it->first);
    }
    return keys;
}
