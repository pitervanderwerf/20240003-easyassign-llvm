#pragma once

#include <iostream>
#include <map>
#include <vector>

// Forward declarations
struct Link;
struct Node;
struct Turn;

// Enum representing junction types
enum JunctionType : short {
    Undefined = 0,
    Equal = 1,
    Roundabout = 2,
    Priority = 3,
    Signalized = 4
};

// Struct representing a link in the network
struct Link {
    double bpr_alpha_forward;
    double bpr_alpha_backward;

    double cost_forward;
    double cost_backward;
    double dist_forward;
    double dist_backward;
    double time_forward;
    double time_backward;

    double speed_forward;
    double speed_backward;
    double capacity_forward;
    double capacity_backward;
    double flow_forward;
    double flow_backward;
    double flow_forward_optimal;
    double flow_backward_optimal;

    int id;
    int from_node;
    int to_node;

    bool direction_forward;
    bool direction_backward;
};

// Struct representing a node in the network
struct Node {
    int id;
    int junction_type; // Enum to represent the junction type
    std::map<int, Turn> turns;  // Map of turn_id to Turn structs
};

struct Turn {
    int turn_id;
    int from_node_id;  // The from-node of the turn
    int to_node_id;    // The to-node of the turn
    int approach_link_id;
    int outgoing_link_id;
    double turn_cost;
};

// Class representing the network
class Network {
public:
    Network();
    ~Network();

    std::map<int, Link>* fetchLinksFromDatabase();
    std::map<int, Node>* fetchJunctionsFromDatabase();
    std::map<int, double>* fetchNodeFractionsFromDatabase();
    std::map<int, int>* fetchPcToCentroidMap();


    void addLink(int id, int from_node, int to_node, double cost_forward, double cost_backward, bool direction_forward, bool direction_backward, double dist_forward, double dist_backward, double time_forward, double time_backward, double speed_forward, double speed_backward, double capacity_forward, double capacity_backward);
    void addNode(int id);

    void updateCosts();
    void updateCosts(int iteration);
    void reducePreviousLinkFlows(int iteration);

    void printNetwork() const;
    void printLinksToFile(const std::string& filename) const;

    std::map<int, Link> links_map;
    std::map<int, Node> nodes_map;
    std::map<int, double> node_fractions;
    std::map<int, int> pc_to_centroid_map;

private:
    std::vector<Node> nodes;  // List of nodes in the network
    std::vector<Link> links;  // List of links in the network
};