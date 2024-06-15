#include "Network.h"

#include <cmath>
#include <fstream>
#include <iostream>

#include "Config.h"
#include "DatabaseConnection.h"
#include "Logger.h"
#include "Stopwatch.h"

Network::Network() {}
Network::~Network() {}

std::map<int, Link>* Network::fetchLinksFromDatabase() {
    Stopwatch stopwatch("FETCHING NETWORK FROM DATABASE");
    stopwatch.start();

        DatabaseConnection conn;

    if (!conn.checkAndConnect()) {
        std::cerr << "Database connection failed." << std::endl;
    }

    const char* sql;
    if (Config::LEVEL_OF_DETAIL == "C2C") {
        //sql = "SELECT link_id, from_node_id, to_node_id, direction_forward, direction_backward, COALESCE(speed_forward, 0.0)::double precision AS speed_forward, COALESCE(speed_backward, 0.0)::double precision AS speed_backward, ST_Length(geom) AS dist_forward, ST_Length(geom) AS dist_backward, COALESCE(traveltime_forward, 99999.0) AS traveltime_forward, COALESCE(traveltime_backward, 99999.0) AS traveltime_backward, COALESCE(capacity_forward, 0.0)::double precision AS capacity_forward, COALESCE(capacity_backward, 0.0)::double precision AS capacity_backward, COALESCE(lt_forward.bpr_alpha, 0.0) AS bpr_alpha_forward, COALESCE(lt_backward.bpr_alpha, 0.0) AS bpr_alpha_backward FROM links LEFT JOIN linktypes lt_forward ON linktype_forward = lt_forward.linktype_name LEFT JOIN linktypes lt_backward ON linktype_backward = lt_backward.linktype_name ORDER BY link_id ASC;";
        sql = "SELECT link_id, from_node_id, to_node_id, direction_forward, direction_backward, COALESCE(speed_forward, 0.0)::double precision AS speed_forward, COALESCE(speed_backward, 0.0)::double precision AS speed_backward, ST_Length(geom) AS dist_forward, ST_Length(geom) AS dist_backward, 1.0 AS traveltime_forward,  1.0 AS traveltime_backward, COALESCE(capacity_forward, 0.0)::double precision AS capacity_forward, COALESCE(capacity_backward, 0.0)::double precision AS capacity_backward, COALESCE(lt_forward.bpr_alpha, 0.0) AS bpr_alpha_forward, COALESCE(lt_backward.bpr_alpha, 0.0) AS bpr_alpha_backward FROM links LEFT JOIN linktypes lt_forward ON linktype_forward = lt_forward.linktype_name LEFT JOIN linktypes lt_backward ON linktype_backward = lt_backward.linktype_name ORDER BY link_id ASC;";
    }
    else if (Config::LEVEL_OF_DETAIL == "PC5") 
    {
        sql = "SELECT link_id, from_node_id, to_node_id, direction_forward, direction_backward, COALESCE(speed_forward, 0.0)::double precision AS speed_forward, COALESCE(speed_backward, 0.0)::double precision AS speed_backward, ST_Length(geom) AS dist_forward, ST_Length(geom) AS dist_backward, COALESCE(traveltime_forward, 99999.0) AS traveltime_forward, COALESCE(traveltime_backward, 99999.0) AS traveltime_backward, COALESCE(capacity_forward, 0.0)::double precision AS capacity_forward, COALESCE(capacity_backward, 0.0)::double precision AS capacity_backward, COALESCE(lt_forward.bpr_alpha, 0.0) AS bpr_alpha_forward, COALESCE(lt_backward.bpr_alpha, 0.0) AS bpr_alpha_backward FROM links_pc5 LEFT JOIN linktypes lt_forward ON linktype_forward = lt_forward.linktype_name LEFT JOIN linktypes lt_backward ON linktype_backward = lt_backward.linktype_name ORDER BY link_id ASC;";
    }
    else 
    {
        sql = "SELECT link_id, from_node_id, to_node_id, direction_forward, direction_backward, COALESCE(speed_forward, 0.0)::double precision AS speed_forward, COALESCE(speed_backward, 0.0)::double precision AS speed_backward, ST_Length(geom) AS dist_forward, ST_Length(geom) AS dist_backward, COALESCE(traveltime_forward, 99999.0) AS traveltime_forward, COALESCE(traveltime_backward, 99999.0) AS traveltime_backward, COALESCE(capacity_forward, 0.0)::double precision AS capacity_forward, COALESCE(capacity_backward, 0.0)::double precision AS capacity_backward, COALESCE(lt_forward.bpr_alpha, 0.0) AS bpr_alpha_forward, COALESCE(lt_backward.bpr_alpha, 0.0) AS bpr_alpha_backward FROM links_pc6 LEFT JOIN linktypes lt_forward ON linktype_forward = lt_forward.linktype_name LEFT JOIN linktypes lt_backward ON linktype_backward = lt_backward.linktype_name ORDER BY link_id ASC;";
    }
    
    PGresult* res = conn.executeQuery(sql);

    if (PQresultStatus(res) != PGRES_TUPLES_OK) {
        std::cerr << "Query execution failed: " << PQerrorMessage(conn.getConnection()) << std::endl;
        PQclear(res);
    }

    for (int row = 0; row < PQntuples(res); ++row) {
        Link link;
        
        double speed_forward = std::stod(PQgetvalue(res, row, 5)); // speed in km/h
        double speed_backward = std::stod(PQgetvalue(res, row, 6)); // speed in km/h
        double dist_forward = std::stod(PQgetvalue(res, row, 7)); // distance in meters
        double dist_backward = std::stod(PQgetvalue(res, row, 8)); // distance in meters

        double ff_traveltime_forward = dist_forward / (speed_forward / 3.6) / 60; // traveltime in minutes
        double ff_traveltime_backward = dist_backward / (speed_backward / 3.6) / 60; // traveltime in minutes

        link.id = std::stoi(PQgetvalue(res, row, 0));
        link.from_node = std::stoi(PQgetvalue(res, row, 1));
        link.to_node = std::stoi(PQgetvalue(res, row, 2));
        link.direction_forward = (PQgetvalue(res, row, 3)[0] == 't');
        link.direction_backward = (PQgetvalue(res, row, 4)[0] == 't');
        link.cost_forward = ff_traveltime_forward; // + Config::VALUE_OF_TIME * dist_forward; // generalized cost = traveltime + value_of_time * distance. This is in minutes so 18 monetary units per hour would be 18 / 60 = 0.3 in this case.
        link.cost_backward = ff_traveltime_backward; // + Config::VALUE_OF_TIME * dist_backward; // generalized cost = traveltime + value_of_time * distance
        link.dist_forward = dist_forward;
        link.dist_backward = dist_backward;
        link.time_forward = ff_traveltime_forward;
        link.time_backward = ff_traveltime_backward;
        link.speed_forward = speed_forward;
        link.speed_backward = speed_backward;
        link.capacity_forward = std::stod(PQgetvalue(res, row, 11));
        link.capacity_backward = std::stod(PQgetvalue(res, row, 12));
        link.flow_forward = 0;
        link.flow_backward = 0;
        link.flow_forward_optimal = 0;
        link.flow_backward_optimal = 0;
        link.bpr_alpha_forward = std::stod(PQgetvalue(res, row, 13));
        link.bpr_alpha_backward = std::stod(PQgetvalue(res, row, 14));

        links_map.emplace(link.id, link);
    }

    PQclear(res);

    Logger::log("\t" + std::to_string(links_map.size()) + " links read from database");

    stopwatch.finish();

    return &links_map;
}

std::map<int, Node>* Network::fetchJunctionsFromDatabase() {
    Stopwatch stopwatch("FETCHING NETWORK JUNCTIONS FROM DATABASE");
    stopwatch.start();

    DatabaseConnection conn;

    if (!conn.checkAndConnect()) {
        std::cerr << "Database connection failed." << std::endl;
    }
    
    const char* sql;
    if (Config::LEVEL_OF_DETAIL == "C2C") {
        sql = "SELECT node_id, junction_type FROM public.nodes WHERE junction_type != 0 ORDER BY node_id ASC;";
    }
    else if (Config::LEVEL_OF_DETAIL == "PC5")
    {
        sql = "SELECT node_id, junction_type FROM public.nodes_pc5 WHERE junction_type != 0 ORDER BY node_id ASC;";
    }
    else
    {
        sql = "SELECT node_id, junction_type FROM public.nodes_pc6 WHERE junction_type != 0 ORDER BY node_id ASC;";
    }

    PGresult* res = conn.executeQuery(sql);

    if (PQresultStatus(res) != PGRES_TUPLES_OK) {
        std::cerr << "Query execution failed: " << PQerrorMessage(conn.getConnection()) << std::endl;
        PQclear(res);
    }

    for (int row = 0; row < PQntuples(res); ++row) {
        Node node;

        node.id = std::stoi(PQgetvalue(res, row, 0));
        node.junction_type = std::stoi(PQgetvalue(res, row, 1));

        nodes_map.emplace(node.id, node);
    }

    PQclear(res);

    Logger::log("\t" + std::to_string(nodes_map.size()) + " junctions read from database");

    stopwatch.finish();

    return &nodes_map;
}

std::map<int, double>* Network::fetchNodeFractionsFromDatabase() {
    Stopwatch stopwatch("FETCHING NODE FRACTIONS FROM DATABASE");
    stopwatch.start();

    DatabaseConnection conn;

    if (!conn.checkAndConnect()) {
        std::cerr << "Database connection failed." << std::endl;
    }

    const char* sql;
    if (Config::LEVEL_OF_DETAIL == "PC5") {
        sql = "SELECT fid, node_fraction FROM public.centroids_pc5 ORDER BY fid ASC;";
    }
    else {
        sql = "SELECT fid, node_fraction FROM public.centroids_pc6 ORDER BY fid ASC;";
    }
    
    PGresult* res = conn.executeQuery(sql);

    if (PQresultStatus(res) != PGRES_TUPLES_OK) {
        std::cerr << "Query execution failed: " << PQerrorMessage(conn.getConnection()) << std::endl;
        PQclear(res);
    }

    for (int row = 0; row < PQntuples(res); ++row) {
        node_fractions.try_emplace(std::stoi(PQgetvalue(res, row, 0)), std::stod(PQgetvalue(res, row, 1)));
    }

    PQclear(res);

    Logger::log("\t" + std::to_string(node_fractions.size()) + " node fractions read from database");

    stopwatch.finish();

    return &node_fractions;
}

std::map<int, int>* Network::fetchPcToCentroidMap() {
    Stopwatch stopwatch("FETCHING PC_TO_CENTROID_MAP FROM DATABASE");
    stopwatch.start();

    DatabaseConnection conn;

    if (!conn.checkAndConnect()) {
        std::cerr << "Database connection failed." << std::endl;
    }

    const char* sql;
    if (Config::LEVEL_OF_DETAIL == "PC5") {
        sql = "SELECT fid, centroid_map FROM public.centroids_pc5 ORDER BY fid ASC;";
    }
    else {
        sql = "SELECT fid, centroid_map FROM public.centroids_pc6 ORDER BY fid ASC;";
    }

    PGresult* res = conn.executeQuery(sql);

    if (PQresultStatus(res) != PGRES_TUPLES_OK) {
        std::cerr << "Query execution failed: " << PQerrorMessage(conn.getConnection()) << std::endl;
        PQclear(res);
    }

    for (int row = 0; row < PQntuples(res); ++row) {
        pc_to_centroid_map.try_emplace(std::stoi(PQgetvalue(res, row, 0)), std::stoi(PQgetvalue(res, row, 1)));
    }

    PQclear(res);

    Logger::log("\t" + std::to_string(pc_to_centroid_map.size()) + " pc5-addresses read from database");

    stopwatch.finish();

    return &pc_to_centroid_map;
}

//void Network::addLink(int id, int from_node, int to_node, double cost_forward, double cost_backward, bool direction_forward, bool direction_backward, double dist_forward, double dist_backward, double time_forward, double time_backward, double speed_forward, double speed_backward, double capacity_forward, double capacity_backward) {
//    Link link = { id, from_node, to_node, direction_forward, direction_backward, cost_forward, cost_backward, dist_forward, dist_backward, time_forward, time_backward, speed_forward, speed_backward, capacity_forward, capacity_backward };
//    links.push_back(link);
//}

void Network::addNode(int id) {
    Node node = { id };
    nodes.push_back(node);
}

// Update costs
void Network::updateCosts() {
    Logger::log("\tUpdating linkcosts...");

    //double alpha = 0.87; // Delft default
    //double alpha = 0.5; // OtTraffic default
    double bpr_beta = 10.0;
    for (auto& [id, link] : links_map) {
        // Calculate new traveltime based on BPR-function
        double ff_traveltime_forward = link.dist_forward / (link.speed_forward / 3.6) / 60; // freeflow traveltime in minutes
        double ff_traveltime_backward = link.dist_backward / (link.speed_backward / 3.6) / 60; // freeflow traveltime in minutes

        double updated_time_forward = ff_traveltime_forward * (1 + link.bpr_alpha_forward * std::pow((link.flow_forward / link.capacity_forward), bpr_beta));
        double updated_time_backward = ff_traveltime_backward * (1 + link.bpr_alpha_backward * std::pow((link.flow_backward / link.capacity_backward), bpr_beta));

        // Update cost and traveltime
        // Update cost for forward direction
        auto itForward = nodes_map.find(link.from_node);
        link.cost_forward = (itForward != nodes_map.end() && itForward->second.junction_type == 1) ?
            (updated_time_forward + Config::EQ_JUNCTION_COST) :
            updated_time_forward; // + Config::VALUE_OF_TIME * link.dist_forward;
        // Update traveltime for forward direction
        link.time_forward = (itForward != nodes_map.end() && itForward->second.junction_type == 1) ?
            (updated_time_forward + Config::EQ_JUNCTION_COST) :
            updated_time_forward;

        // Update cost for backward direction
        auto itBackward = nodes_map.find(link.to_node);
        link.cost_backward = (itBackward != nodes_map.end() && itBackward->second.junction_type == 1) ?
            (updated_time_backward + Config::EQ_JUNCTION_COST) :
            updated_time_backward; // + Config::VALUE_OF_TIME * link.dist_backward;
        // Update traveltime for backward direction
        link.time_backward = (itBackward != nodes_map.end() && itBackward->second.junction_type == 1) ?
            (updated_time_backward + Config::EQ_JUNCTION_COST) :
            updated_time_backward;
        
        
        // Update traveltime and speed
        //link.time_forward = updated_time_forward;
        //link.time_backward = updated_time_backward;
        //link.speed_forward = (link.dist_forward / link.time_forward) * 3.6;
        //link.speed_backward = (link.dist_backward / link.time_backward) * 3.6;
    }
    Logger::log("\tUpdating linkcosts done");
}

void Network::updateCosts(int iteration) {
    Logger::log("\tUpdating linkcosts...");

    //double alpha = 0.87; // Delft default
    //double alpha = 0.5; // OtTraffic default
    double beta = 4.0;
    for (auto& [id, link] : links_map) {
        // Calculate new traveltime based on BPR-function
        double ff_traveltime_forward = link.dist_forward / (link.speed_forward / 3.6) / 60; // freeflow traveltime in minutes
        double ff_traveltime_backward = link.dist_backward / (link.speed_backward / 3.6) / 60; // freeflow traveltime in minutes

        double updated_time_forward = ff_traveltime_forward * (1 + link.bpr_alpha_forward * std::pow((link.flow_forward / link.capacity_forward), beta));
        double updated_time_backward = ff_traveltime_backward * (1 + link.bpr_alpha_backward * std::pow((link.flow_backward / link.capacity_backward), beta));

            // Update cost and traveltime
            // Update cost for forward direction
            auto itForward = nodes_map.find(link.from_node);
        link.cost_forward = (itForward != nodes_map.end() && itForward->second.junction_type == 1) ?
            (updated_time_forward + Config::EQ_JUNCTION_COST) :
            updated_time_forward; // + Config::VALUE_OF_TIME * link.dist_forward;
        // Update traveltime for forward direction
        link.time_forward = (itForward != nodes_map.end() && itForward->second.junction_type == 1) ?
            (updated_time_forward + Config::EQ_JUNCTION_COST) :
            updated_time_forward;

        // Update cost for backward direction
        auto itBackward = nodes_map.find(link.to_node);
        link.cost_backward = (itBackward != nodes_map.end() && itBackward->second.junction_type == 1) ?
            (updated_time_backward + Config::EQ_JUNCTION_COST) :
            updated_time_backward; // + Config::VALUE_OF_TIME * link.dist_backward;
        // Update traveltime for backward direction
        link.time_backward = (itBackward != nodes_map.end() && itBackward->second.junction_type == 1) ?
            (updated_time_backward + Config::EQ_JUNCTION_COST) :
            updated_time_backward;


        // Update traveltime and speed
        //link.time_forward = updated_time_forward;
        //link.time_backward = updated_time_backward;
        //link.speed_forward = (link.dist_forward / link.time_forward) * 3.6;
        //link.speed_backward = (link.dist_backward / link.time_backward) * 3.6;
    }
    Logger::log("\tUpdating linkcosts done");
}

void Network::reducePreviousLinkFlows(int iteration) {
    // Calculate the reduction factor 1 - 1/n
    double reduction_factor = 1.0 - 1.0 / iteration;

    // Iterate over the links_map and reduce flows
    for (auto& entry : links_map) {
        Link& link = entry.second;
        link.flow_forward *= reduction_factor;
        link.flow_backward *= reduction_factor;
    }
}

// Print information about the network
void Network::printNetwork() const {
    std::cout << "Nodes:" << std::endl;
    for (const auto& node : nodes) {
        std::cout << "Node " << node.id << std::endl;
    }

    std::cout << "\nLinks:" << std::endl;
    for (const auto& link : links) {
        std::cout << "Link " << link.id << " from Node " << link.from_node
            << " to Node " << link.to_node << " with cost " << link.cost_forward << std::endl;
    }
}

void Network::printLinksToFile(const std::string& filename) const {
    std::ofstream output_file(filename);
    if (!output_file.is_open()) {
        Logger::log("Error: Unable to open file " + filename + " for writing");
        return;
    }

    // Print header
    output_file << "link_id;" << "from_node;" << "to_node;" << "direction_forward;" << "direction_backward;" << "cost_forward;" << "cost_backward;" << "dist_forward;" << "dist_backward;" << "time_forward;" << "time_backward;" << "speed_forward;" << "speed_backward;" << "flow_forward;" << "flow_backward;" << "capacity_forward;" << "capacity_backward" << std::endl;

    // Print linkdata
    for (const auto& [link_id, link] : links_map) {
        output_file << link_id << ";"
            << link.from_node << ";"
            << link.to_node << ";"
            << link.direction_forward << ";"
            << link.direction_backward << ";"
            << link.bpr_alpha_forward << ";"
            << link.bpr_alpha_backward << ";"
            << link.cost_forward << ";"
            << link.cost_backward << ";"
            << link.dist_forward << ";"
            << link.dist_backward << ";"
            << link.time_forward << ";"
            << link.time_backward << ";"
            << link.speed_forward << ";"
            << link.speed_backward << ";"
            << link.flow_forward << ";"
            << link.flow_backward << ";"
            << link.capacity_forward << ";"
            << link.capacity_backward << std::endl;
    }

    Logger::log("Links printed to " + filename + " successfully");
}
