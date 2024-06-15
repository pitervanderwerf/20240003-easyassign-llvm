#pragma once

#include <map>
#include <mutex>
#include <shared_mutex>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/compressed_sparse_row_graph.hpp>
#include <boost/graph/visitors.hpp>

#include "Matrix.h"
#include "Network.h"
#include "boost/graph/dijkstra_shortest_paths.hpp"

struct EdgeProperties {
    double weight;
    int link_id;
    bool direction;
    bool turn;
};

struct LinkFlow {
    double flow_forward;
    double flow_backward;
};

struct Dimensions {
    int p;
    int m;
    int t;
    int u;
    std::vector<int> r;
    int i;
};

struct FoundAllTargets : public std::exception {
    const char* what() const noexcept override {
        return "All target vertices found";
    }
};

class Assignment {
public:
    Assignment(const Dimensions& dimensions);

    void buildGraphFromLinksMapWithTurns();
    void buildGraphFromLinksMapWithTurnsFull();
    void printNumberOfVertices();
    void printNumberOfEdges();
    void printEdgeById(int target_link_id);

    void execute();
    void executePc5(int number_of_pc5_nodes);
    void executePc6(int number_of_pc6_nodes);
    void assign(int start_source_vertex, int end_source_vertex);
    void assignPc5(int start_source_vertex, int end_source_vertex, int number_of_pc5_nodes);
    void assignPc6(int start_source_vertex, int end_source_vertex,int number_of_pc6_nodes);

    void printFlows();
    void printFlowsLinksMap();
    void flowsToLinks();
    void calculateTotalTraveltimeSpent();
    void flowsToLinksVector();
    void resetLinkFlows();
    void streamToDatabaseLinksMap();
    void printPaths();

    std::unordered_map<int, LinkFlow> link_flows;
    std::unordered_map<int, LinkFlow> link_flows_optimal;

    int counter = 0;
    int iteration = 0;

private:
    double va_factor = 1.0;

    Dimensions dimensions;

    Matrix odm;
    Matrix skm_cost;
    Matrix skm_dist;
    Matrix skm_time;

    Network network;
    std::map<int, Link>* links_map;
    std::map<int, Node>* nodes_map;
    std::map<int, double>* node_fractions;
    std::map<int, int>* pc_to_centroid_map;

    typedef boost::compressed_sparse_row_graph<boost::directedS, boost::no_property, EdgeProperties, int> Graph;
    typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
    typedef boost::graph_traits<Graph>::edge_descriptor Edge;
    Graph G;

    std::map<int, Edge> link_to_edge;
    std::map<Edge, int> edge_to_link;
    std::map<int, int> link_to_edge_index;
    std::map<int, int> edge_index_to_link;
    std::map<int, Vertex> node_to_vertex;
    std::map<Vertex, int> vertex_to_node;
    std::map<int, Edge> turn_to_edge;
    std::map<Edge, int> edge_to_turn;

    std::mutex counter_mutex;
    std::mutex predecessor_maps_mutex;
    std::mutex link_flows_mutex;
    std::mutex link_flows_optimal_mutex;
    std::mutex paths_file_mutex;
    std::mutex skim_list_mutex;

    std::string paths;

//// Custom visitor for early termination
//    class EarlyStopVisitor : public boost::default_dijkstra_visitor {
//    public:
//        explicit EarlyStopVisitor(const std::unordered_set<Vertex>& targets)
//                : targets(targets), target_size(targets.size()) {}
//
//        template <typename Vertex, typename Graph>
//        void examine_vertex(Vertex u, const Graph& g) {
//            if (targets.find(u) != targets.end()) {
//                found_targets.insert(u);
//                if (found_targets.size() == target_size) {
//                    throw FoundAllTargets();
//                }
//            }
//        }
//
//    private:
//        const std::unordered_set<Vertex>& targets;
//        const size_t target_size;
//        std::unordered_set<Vertex> found_targets;
//    };
// Custom visitor for early termination
//    class EarlyStopVisitor : public boost::default_dijkstra_visitor {
//    public:
//        EarlyStopVisitor(const std::vector<bool>& target_flags, size_t target_count)
//                : target_flags(target_flags), target_count(target_count), found_count(0) {}
//
//        template <typename Vertex, typename Graph>
//        void examine_vertex(Vertex u, const Graph& g) {
//            if (target_flags[u]) {
//                found_count++;
//                if (found_count == target_count) {
//                    throw FoundAllTargets();
//                }
//            }
//        }
//
//    private:
//        const std::vector<bool>& target_flags;
//        size_t target_count;
//        size_t found_count;
//    };
// Custom visitor for early termination with batch checking
    class EarlyStopVisitor : public boost::default_dijkstra_visitor {
    public:
        EarlyStopVisitor(const std::vector<bool>& target_flags, size_t target_count, size_t check_interval)
                : target_flags(target_flags), target_count(target_count), found_count(0), check_interval(check_interval), processed_count(0) {}

        template <typename Vertex, typename Graph>
        void examine_vertex(Vertex u, const Graph& g) {
            processed_count++;
            if (processed_count % check_interval == 0 || target_flags[u]) {
                if (target_flags[u]) {
                    found_count++;
                }
                if (found_count == target_count) {
                    throw FoundAllTargets();
                }
            }
        }

    private:
        const std::vector<bool>& target_flags;
        size_t target_count;
        size_t found_count;
        size_t check_interval;
        size_t processed_count;
    };
};
