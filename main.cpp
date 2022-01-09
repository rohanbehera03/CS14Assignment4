//
// Created by Rohan Behera on 12/10/20.
//
//
// g++ -std=c++11 graph.cpp main.cpp -o graph

#include <algorithm>
#include <cstdio>
#include <chrono>
#include <functional>
#include <iostream>
#include <random>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "graph.hpp"


// A pair of nodes such that node1 < node2
typedef struct NodePair {
    size_t index1;
    size_t index2;

    NodePair(size_t i, size_t j) {
        assert(i >= 0);
        assert(j >= 0);
        assert(i != j);
        if (i < j) {
            index1 = i;
            index2 = j;
        } else {
            index1 = j;
            index2 = i;
        }
    }
} NodePair;

// Make NodePair hashable
namespace std {
    template <>
    struct hash<NodePair> {
        inline size_t operator()(const NodePair& p) const {
            auto hash1 = std::hash<size_t>{}(p.index1);
            auto hash2 = std::hash<size_t>{}(p.index2);
            return hash1 ^ hash2;
        }
    };
}  // namespace std

// Make NodePair hashable
namespace std {
    template <>
    struct equal_to<NodePair> {
        inline bool operator()(const NodePair& a, const NodePair& b) const {
            return a.index1 == b.index1 && a.index2 == b.index2;
        }
    };
}
// Unique node names in sorted order
static std::vector<std::string> NODE_NAMES;
// Unique pairs of nodes as edges with the weights, no self-loop allowed
static std::unordered_map<NodePair, double> NODE_PAIRS;
// Random number using uniform distribution
static std::random_device RD;
static std::mt19937 GEN(RD());
static std::uniform_int_distribution<size_t> DIST;

// Generate n unique node names
static void generateNodeNames(size_t n);
static size_t getRandomIndex();
// Generate m unique pairs of nodes as edges
static void generateNodePairs(size_t m, double weight);

char alphabetNum[] = "abcdefghijklmnopqrstuvwxyz";
double tripleCliqueTime, isConnectedTime, getMinDistanceTime, longSimplePathTime;

int main() {
    //std::cout << "Start Graph Test" << std::endl;
    generateNodeNames(100);
    // std::cout << "End generateNodeNames" << std::endl;
    generateNodePairs(500, 1.0);
    //std::cout << "End generateNodePairs 1.0" << std::endl;
    generateNodePairs(500, 2.0);
    //std::cout << "End generateNodePairs 2.0" << std::endl;
    generateNodePairs(500, 3.0);
    //std::cout << "End generateNodePairs 3.0" << std::endl;
    Graph graph(false);
    //std::cout << "End Graph created" << std::endl;

    // Add nodes
    for (std::string name : NODE_NAMES) {
        graph.addNode(name);
    }
    //std::cout << "End addNodes" << std::endl;

    // Add edges
    for (auto it : NODE_PAIRS) {
        graph.addEdge(NODE_NAMES[it.first.index1], NODE_NAMES[it.first.index2],
                      it.second);
    }
    //std::cout << "End addEdges" << std::endl;

    std::cout << "No. of Nodes = " << graph.getNoOfNodes() << std::endl;
    std::cout << "No. of Edges = " << graph.getNoOfEdges() << std::endl;
    graph.dfs();
    graph.printGraph();
    std::cout << "End printGraph" << std::endl;
    std::cout << "End of Graph Test " << std::endl;

    auto start_time = std::chrono::high_resolution_clock::now();
    std::cout << "The shortest path distance between city 1 and city 2 is: " << graph.getMinDistance(NODE_NAMES.at(0), NODE_NAMES.at(1)) << std::endl;
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time);
    std::cout << "Time taken by getMinDistance(string city1, string city2) in nanoseconds: " << duration.count() << " ns" << std::endl << std::endl;
    getMinDistanceTime = duration.count();

    start_time = std::chrono::high_resolution_clock::now();
    std::cout << "1 means true, 0 means false. Testing hasTripletClique() " << graph.hasTripletClique() << std::endl;
    end_time = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time);
    std::cout << "Time taken by hasTripletClique() in nanoseconds: " << duration.count() << " ns" << std::endl << std::endl;
    tripleCliqueTime = duration.count();

    start_time = std::chrono::high_resolution_clock::now();
    std::cout << "The Graph is (1 means connected, 0 means not connected): " << graph.isConnected() << std::endl;
    end_time = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time);
    std::cout << "Time taken by isConnected() in nanoseconds: " << duration.count() << " ns" << std::endl;
    isConnectedTime = duration.count();

    start_time = std::chrono::high_resolution_clock::now();
    std::cout << "Length of longest simple path: " << graph.getLongestSimplePath() << std::endl;
    end_time = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time);
    std::cout << "Time taken by getLongestSimplePath() in nanoseconds: " << duration.count() << " ns" << std::endl;
    longSimplePathTime = duration.count();

    return 0;
}

static void generateNodeNames(size_t n) {
    assert(n > 0);
    assert(NODE_NAMES.empty());
    std::unordered_set<std::string> names;
    names.reserve(n);
    while (names.size() < n) {
        std::string name = "";  // To do: Generate a random name here
        for (int i = 0; i < 5; ++i)  {
            name += alphabetNum[rand() % (sizeof(alphabetNum) - 1)];
        }
        names.insert(name);     // Insert will fail for duplicate names
        name = "";
    }

    NODE_NAMES.reserve(n);
    for (std::string name : names) NODE_NAMES.push_back(name);
    std::sort(NODE_NAMES.begin(), NODE_NAMES.end());
    //  std::cout << "Ended generateNodeNames" << std::endl;
}

static size_t getRandomIndex() {
    assert(!NODE_NAMES.empty());
    if (DIST.a() != 0 || DIST.b() != NODE_NAMES.size() - 1) {
        decltype(DIST.param()) range(0, NODE_NAMES.size() - 1);
        DIST.param(range);
    }
    return DIST(GEN);
}

static void generateNodePairs(size_t m, double weight) {
    assert(m > 0);
    assert(weight > 0.0);
    assert(weight < Graph::INF);

    size_t original = NODE_PAIRS.size();
    NODE_PAIRS.reserve(original + m);
    while (NODE_PAIRS.size() - original < m) {
        // Randomly pick 2 distinct names
        size_t i = getRandomIndex();
        size_t j = getRandomIndex();
        while (i == j) j = getRandomIndex();
        // If a pair of nodes of these 2 names already exists, insert will fail
        if (i < j) {
            NODE_PAIRS.insert({NodePair(i, j), weight});
        } else {
            NODE_PAIRS.insert({NodePair(j, i), weight});
        }
    }

}
