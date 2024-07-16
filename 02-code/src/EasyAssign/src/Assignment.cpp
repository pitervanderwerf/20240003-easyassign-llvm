#include "Assignment.h"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <queue>
#include <sstream>
#include <thread>

#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/dijkstra_shortest_paths_no_color_map.hpp>

#include "Config.h"
#include "DatabaseConnection.h"
#include "Logger.h"
#include "Matrix.h"
#include "Stopwatch.h"

using namespace boost;

Assignment::Assignment(const Dimensions& dimensions)
        : dimensions(dimensions),
          odm(std::vector<int>{dimensions.p, dimensions.m, dimensions.t, dimensions.u, dimensions.r[0], dimensions.i}, "odm"),
          skm_cost(std::vector<int>{dimensions.p, dimensions.m, dimensions.t, dimensions.u, dimensions.r[0], dimensions.i}, "skm"),
          skm_dist(std::vector<int>{dimensions.p, dimensions.m, dimensions.t, dimensions.u, dimensions.r[1], dimensions.i}, "skm"),
          skm_time(std::vector<int>{dimensions.p, dimensions.m, dimensions.t, dimensions.u, dimensions.r[2], dimensions.i}, "skm"),
          links_map(network.fetchLinksFromDatabase()),
          nodes_map(network.fetchJunctionsFromDatabase())
{
    if (Config::LEVEL_OF_DETAIL != "C2C") {
        node_fractions = network.fetchNodeFractionsFromDatabase();
        pc_to_centroid_map = network.fetchPcToCentroidMap();
    }
}


// Graph operations
void Assignment::buildGraphFromLinksMapWithTurns() {
    std::vector<std::pair<Vertex, Vertex>> edge_pairs;
    std::vector<EdgeProperties> edge_properties;
    std::unordered_map<int, Vertex> local_node_to_vertex;

    for (const auto& pair : network.links_map) {
        const Link& link = pair.second;

        // Ensure unique CSR vertices for each node
        if (local_node_to_vertex.find(link.from_node) == local_node_to_vertex.end()) {
            local_node_to_vertex[link.from_node] = local_node_to_vertex.size();
        }
        if (local_node_to_vertex.find(link.to_node) == local_node_to_vertex.end()) {
            local_node_to_vertex[link.to_node] = local_node_to_vertex.size();
        }

        Vertex from_vertex = local_node_to_vertex[link.from_node];
        Vertex to_vertex = local_node_to_vertex[link.to_node];

        // Add edges based on direction and junction type
        if (link.direction_forward) {
            double cost = link.cost_forward;
            auto it = nodes_map->find(link.from_node);
            if (it != nodes_map->end()) {
                if (it->second.junction_type == 1) { // It's an equal priority junction
                    cost += Config::EQ_JUNCTION_COST;
                } else if (it->second.junction_type == -2) { // It's a centroid
                    cost += 10.0;
                }
            }

            edge_pairs.emplace_back(from_vertex, to_vertex);
            edge_properties.push_back({ cost, link.id, 0, 0 });
            link_to_edge_index[link.id] = edge_properties.size() - 1;
            edge_index_to_link[edge_properties.size() - 1] = link.id;
        }

        if (link.direction_backward) {
            double cost = link.cost_backward;
            auto it = nodes_map->find(link.to_node);
            if (it != nodes_map->end()) {
                if (it->second.junction_type == 1) { // It's an equal priority junction
                    cost += Config::EQ_JUNCTION_COST;
                } else if (it->second.junction_type == -2) { // It's a centroid
                    cost += 10.0;
                }
            }

            edge_pairs.emplace_back(to_vertex, from_vertex);
            edge_properties.push_back({ cost, link.id, 1, 0 });
            link_to_edge_index[link.id] = edge_properties.size() - 1;
            edge_index_to_link[edge_properties.size() - 1] = link.id;
        }
    }

    // Create the Graph
    G = Graph(boost::edges_are_unsorted_multi_pass, edge_pairs.begin(), edge_pairs.end(), edge_properties.begin(), local_node_to_vertex.size());

    // Update node_to_vertex and vertex_to_node mappings
    for (const auto& node_vertex_pair : local_node_to_vertex) {
        node_to_vertex[node_vertex_pair.first] = node_vertex_pair.second;
        vertex_to_node[node_vertex_pair.second] = node_vertex_pair.first;
    }

    // Print number of vertices and edges if needed
    printNumberOfVertices();
    printNumberOfEdges();
    Logger::log("");
}


void Assignment::printNumberOfVertices() {
    Logger::log("\t" + std::to_string(num_vertices(G)) + " vertices added to graph");
}

void Assignment::printNumberOfEdges() {
    Logger::log("\t" + std::to_string(num_edges(G)) + " edges added to graph");
}

// Function to find and print the edge with a specific link.id
void Assignment::printEdgeById(int target_link_id) {
    using namespace boost;

    // Check if the link.id exists in the map
    auto it = link_to_edge.find(target_link_id);
    if (it != link_to_edge.end()) {
        Edge targetEdge = it->second;
        Vertex sourceVertex = source(targetEdge, G);
        Vertex targetVertex = target(targetEdge, G);

        // Use the vertex_to_node map to get the actual nodes
        int source_node = vertex_to_node[sourceVertex];
        int target_node = vertex_to_node[targetVertex];

        std::cout << "Edge found for link.id: " << target_link_id << std::endl;
        std::cout << "Source node: " << source_node << std::endl;
        std::cout << "Target node: " << target_node << std::endl;
    }
    else {
        // Print a message if the edge with the specified link.id is not found
        std::cout << "Edge with link.id " << target_link_id << " not found." << std::endl;
    }
}

void Assignment::printFlows() {
    std::filesystem::create_directory("results");
    std::ofstream out("results/link_flows.txt");

    for (const auto& [link_id, flow] : link_flows) {
        out << link_id << ";" << flow.flow_forward << ";" << flow.flow_backward << "\n";
    }
}

void Assignment::printFlowsLinksMap() {
    std::filesystem::create_directory("results");
    std::ofstream out("results/link_flows_links_map.txt");

    for (const auto& [link_id, link] : network.links_map) {
        out << link_id << ";" << link.flow_forward << ";" << link.flow_backward << "\n";
    }

    std::ofstream out2("results/link_flows_links_map_optimal.txt");

    for (const auto& [link_id, link] : network.links_map) {
        out2 << link_id << ";" << link.flow_forward_optimal << ";" << link.flow_backward_optimal << "\n";
    }
}

void Assignment::resetLinkFlows() {
    this->link_flows.clear();
    this->link_flows_optimal.clear();
}

void Assignment::flowsToLinks() {
    for (const auto& [link_id, flow] : link_flows) {
        auto& link = network.links_map[link_id];
        link.flow_forward += flow.flow_forward;
        link.flow_backward += flow.flow_backward;
    }
    for (const auto& [link_id, flow] : link_flows_optimal) {
        auto& link = network.links_map[link_id];
        link.flow_forward_optimal = flow.flow_forward;
        link.flow_backward_optimal = flow.flow_backward;
    }
}

void Assignment::calculateTotalTraveltimeSpent() {
    double total_traveltime_spent = 0.0;
    double total_traveltime_spent_optimal = 0.0;

    for (auto& [link_id, link] : network.links_map) {
        double traveltime_forward = (link.flow_forward > 0.0 && link.cost_forward < 1000.0) ? link.cost_forward * link.flow_forward : 0.0;
        double traveltime_backward = (link.flow_backward > 0.0 && link.cost_backward < 1000.0) ? link.cost_backward * link.flow_backward : 0.0;
        double traveltime_forward_optimal = (link.flow_forward_optimal > 0.0 && link.cost_forward < 1000.0) ? link.cost_forward * link.flow_forward_optimal : 0.0;
        double traveltime_backward_optimal = (link.flow_backward_optimal > 0.0 && link.cost_backward < 1000.0) ? link.cost_backward * link.flow_backward_optimal : 0.0;

        total_traveltime_spent += traveltime_forward + traveltime_backward;
        total_traveltime_spent_optimal += traveltime_forward_optimal + traveltime_backward_optimal;

        // Reset the optimal flows for the next iteration
        link.flow_forward_optimal = 0.0;
        link.flow_backward_optimal = 0.0;
    }

    double duality_gap = total_traveltime_spent - total_traveltime_spent_optimal;
    double relative_gap = (total_traveltime_spent_optimal > 0) ? duality_gap / total_traveltime_spent_optimal : 0.0;

    Logger::log("\tLambda: " + std::to_string(va_factor));
    Logger::log("\tTotal traveltime spent        : " + std::to_string(total_traveltime_spent));
    Logger::log("\tTotal traveltime spent optimal: " + std::to_string(total_traveltime_spent_optimal));
    Logger::log("\tDuality gap: " + std::to_string(duality_gap));
    Logger::log("\tRelative gap: " + std::to_string(relative_gap));
}

void Assignment::printPaths() {
    std::ofstream paths_file;
    paths_file.open("paths.txt");
    paths_file << this->paths;
    paths_file.close();
}

void Assignment::streamToDatabaseLinksMap() {
    DatabaseConnection conn;
    if (!conn.checkAndConnect()) {
        Logger::log("Failed to connect to the database");
        return;
    }

    PGconn* connection = conn.getConnection();

    // Start a transaction
    PGresult* transaction_result = PQexec(connection, "BEGIN");
    if (PQresultStatus(transaction_result) != PGRES_COMMAND_OK) {
        Logger::log("Failed to start transaction: " + std::string(PQresultErrorMessage(transaction_result)));
        PQclear(transaction_result);
        return;
    }
    PQclear(transaction_result);

    // Clean the link_flows table
    std::string sql_truncate = "TRUNCATE link_flows";
    conn.executeQuery(sql_truncate.c_str());

    // Prepare the COPY FROM STDIN command
    std::string sql = "COPY link_flows (link_id, flow_forward, flow_backward) FROM STDIN";
    const char* copy_command = sql.c_str();
    PGresult* copy_result = PQexec(connection, copy_command);
    if (PQresultStatus(copy_result) != PGRES_COPY_IN) {
        Logger::log("Failed to prepare COPY FROM STDIN: " + std::string(PQresultErrorMessage(copy_result)));
        PQclear(copy_result);
        return;
    }
    PQclear(copy_result);

    // Generate the data in the required format and feed it to COPY
    std::ostringstream data_stream;
    for (const auto& entry : network.links_map) {
        int link_id = entry.second.id;
        double flow_forward = entry.second.flow_forward;
        double flow_backward = entry.second.flow_backward;

        data_stream << link_id << "\t" << flow_forward << "\t" << flow_backward << "\n";
    }

    // Feed the data to COPY via STDIN
    const std::string& data = data_stream.str();
    PQputCopyData(connection, data.c_str(), static_cast<int>(data.size()));
    PQputCopyEnd(connection, nullptr);

    // Commit the transaction
    transaction_result = PQexec(connection, "COMMIT");
    if (PQresultStatus(transaction_result) != PGRES_COMMAND_OK) {
        Logger::log("Failed to commit transaction: " + std::string(PQresultErrorMessage(transaction_result)));
    }
    PQclear(transaction_result);
}

// Assignment
void Assignment::assign(int start_source_vertex, int end_source_vertex) {
    std::vector<double> distances(num_vertices(G));
    std::vector<Vertex> predecessors(num_vertices(G));

    std::unordered_map<int, LinkFlow> local_link_flows; // Collect link_flows locally
    local_link_flows.reserve(network.links_map.size() / 32);
    std::unordered_map<int, LinkFlow> local_link_flows_optimal; // Collect link_flows locally
    local_link_flows_optimal.reserve(network.links_map.size() / 32);

    auto weight_map = get(&EdgeProperties::weight, G);

    std::map<std::pair<int, int>, double> local_skim_cost_list;
    std::string local_paths;

    for (int source_vertex_id = start_source_vertex; source_vertex_id <= end_source_vertex; ++source_vertex_id) {
        if (odm.row_sum_map[source_vertex_id - 1] > Config::SIGNIFICANT_SOURCE_PRODUCTION) {
            Vertex source_vertex = node_to_vertex[source_vertex_id];

            dijkstra_shortest_paths_no_color_map(
                    G,
                    source_vertex,
                    predecessor_map(&predecessors[0])
                            .distance_map(&distances[0])
                            .weight_map(weight_map)
            );

            if (Config::ASSIGN_FLOWS || Config::CREATE_SKIMS || Config::PRINT_PATHS) {
                for (int target_vertex_id = 1; target_vertex_id <= Config::NUMBER_OF_CENTROIDS; target_vertex_id++) {
                    if (source_vertex_id != target_vertex_id) {
//                    if (false) {
                        double total_od_flow = odm.at(source_vertex_id - 1, target_vertex_id - 1);
                        double od_flow = total_od_flow * va_factor;

                        if (total_od_flow >= Config::INSIGNIFICANT_OD_PAIRS_THRESHOLD) {
                            Vertex v = node_to_vertex[target_vertex_id];
                            double path_cost = 0;

                            while (v != source_vertex) {
                                Vertex prev = predecessors[v];
                                auto edge_pair = edge(prev, v, G);

                                if (edge_pair.second) {  // Check if the edge exists
                                    auto e = edge_pair.first;
                                    auto &edge_props = G[e];

                                    if (Config::ASSIGN_FLOWS) {
                                        auto &link_flow = local_link_flows[edge_props.link_id];
                                        auto &link_flow_optimal = local_link_flows_optimal[edge_props.link_id];

                                        if (edge_props.direction == 0) {
                                            link_flow.flow_forward += od_flow;
                                            link_flow_optimal.flow_forward += total_od_flow;
                                        } else if (edge_props.direction == 1) {
                                            link_flow.flow_backward += od_flow;
                                            link_flow_optimal.flow_backward += total_od_flow;
                                        }
                                    }

                                    if (Config::CREATE_SKIMS) {
                                        double cost = edge_props.weight;
                                        path_cost += cost;
                                    }
                                }

                                if (v == prev) {
                                    break;
                                }
                                v = prev;
                            }

                            if (Config::CREATE_SKIMS) {
                                local_skim_cost_list[std::make_pair(source_vertex_id, target_vertex_id)] = path_cost;
                            }
                        }
                    }
                }
            }
        }
        counter++;
        if (counter % 1000 == 0) {
            std::lock_guard<std::mutex> lock(counter_mutex);
            int progress = int(static_cast<double>(counter) / Config::NUMBER_OF_CENTROIDS * 100);
            Logger::log("\t" + std::to_string(counter) + " centroids done of " + std::to_string(Config::NUMBER_OF_CENTROIDS) + " \t /// Progress: " + std::to_string(progress) + "%");
        }
        if (counter == Config::NUMBER_OF_CENTROIDS) {
            std::lock_guard<std::mutex> lock(counter_mutex);
            Logger::log("\t" + std::to_string(counter) + " centroids done of " + std::to_string(Config::NUMBER_OF_CENTROIDS) + " \t /// Progress: " + std::to_string(100) + "%");
        }
    }

    // Flush local_link_flows to shared link_flows
    if (Config::ASSIGN_FLOWS) {
        std::lock_guard<std::mutex> lock_link_flows(link_flows_mutex);
        for (const auto& entry : local_link_flows) {
            link_flows[entry.first].flow_forward += entry.second.flow_forward;
            link_flows[entry.first].flow_backward += entry.second.flow_backward;
        }
        std::lock_guard<std::mutex> lock_link_flows_optimal(link_flows_optimal_mutex);
        for (const auto& entry : local_link_flows_optimal) {
            link_flows_optimal[entry.first].flow_forward += entry.second.flow_forward;
            link_flows_optimal[entry.first].flow_backward += entry.second.flow_backward;
        }
    }

    // Flush local_skim_list to shared skim-matrix
    if (Config::CREATE_SKIMS) {
        std::lock_guard<std::mutex> lock_skim_list(skim_list_mutex);

        auto updateSkimValues = [this](const auto& local_skim_list, auto& skim_matrix) {
            for (const auto& [key, value] : local_skim_list) {
                skim_matrix.at(key.first - 1, key.second - 1) = value;
                skim_matrix.incrementMatrixSum(value);
            }
        };
        updateSkimValues(local_skim_cost_list, skm_cost);
    }
}

// Execute
void Assignment::execute()
{
    Stopwatch stopwatch("ASSIGNMENT");
    stopwatch.start();

    for (int i = 1; i <= Config::NUMBER_OF_ITERATIONS; i++) {
        iteration = i;
        va_factor = (1.0 / iteration);

        buildGraphFromLinksMapWithTurns();

        if (iteration > 1) {
            network.reducePreviousLinkFlows(iteration);
        }

        if (iteration == 1) {
            odm.setOdMatrixFromBinary();
            odm.createSumRowMap();
        }

        std::vector<std::thread> threads;
        const int CHUNK_SIZE = 32;
        const int total_tasks = Config::NUMBER_OF_CENTROIDS;
        std::atomic<int> next_task(1); // Start from 1

        // Launch worker threads
        for (int i = 0; i < Config::MAX_THREADS; ++i) {
            threads.emplace_back([&, i]() {
                while (true) {
                    int task_id = next_task.fetch_add(CHUNK_SIZE); // Get the next chunk of tasks
                    if (task_id > total_tasks) {
                        break; // All tasks are completed
                    }

                    int start_source_vertex = task_id;
                    int end_source_vertex = std::min(task_id + CHUNK_SIZE - 1, Config::NUMBER_OF_CENTROIDS);

                    assign(start_source_vertex, end_source_vertex);
                }
            });
        }

        // Wait for all threads to finish
        for (std::thread& thread : threads) {
            thread.join();
        }

        Logger::log("");

        dimensions.i = iteration;
        skm_cost.setPmturi({ dimensions.p, dimensions.m, dimensions.t, dimensions.u, dimensions.r[0], iteration });
        skm_dist.setPmturi({ dimensions.p, dimensions.m, dimensions.t, dimensions.u, dimensions.r[1], iteration });
        skm_time.setPmturi({ dimensions.p, dimensions.m, dimensions.t, dimensions.u, dimensions.r[2], iteration });

        (Config::ASSIGN_FLOWS) ? flowsToLinks() : void();
        if (iteration == Config::NUMBER_OF_ITERATIONS) {
            (Config::ASSIGN_FLOWS) ? printFlowsLinksMap() : void();
            (Config::ASSIGN_FLOWS) ? streamToDatabaseLinksMap() : void();
            (Config::CREATE_SKIMS) ? skm_cost.writeMatrixToTextFile(("results/skm_cost_" + std::to_string(iteration) + ".txt").c_str()) : void();
        }

        calculateTotalTraveltimeSpent();
        Logger::log("");
        resetLinkFlows();
        network.updateCosts();

        counter = 0;
        stopwatch.checkpoint("ITERATION " + std::to_string(iteration));
    }

    stopwatch.finish();
}