#include <iostream>
#include <vector>
#include <unordered_map>
#include <queue>
#include <cmath>
#include <algorithm>
#include <sqlite3.h>
#include <string>

// this script contains the functions to compute the map path
// it employs dijkstra algorithm to find the shortest path treating maps as nodes and wormholes as edges

struct Wormhole {
    std::string to_map;   
    double x, y;          
};

struct Node {
    std::string map_id;   
    double cost;         
    std::string parent;   

    bool operator>(const Node &other) const {
        return cost > other.cost; 
    }
};

double distance(double x1, double y1, double x2, double y2) {
    return std::sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
}

// query neighbors from the database given the from map id,
std::vector<Wormhole> get_neighbors(sqlite3* db, const std::string &from_map_id) {
    std::vector<Wormhole> neighbors;
    sqlite3_stmt *stmt;

    const char *sql =
        "SELECT to_map_id, from_wormhole_x, from_wormhole_y "
        "FROM wormholes WHERE from_map_id = ?;";

    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK) {
        std::cerr << "Failed to prepare statement: " << sqlite3_errmsg(db) << "\n";
        return neighbors;
    }

    sqlite3_bind_text(stmt, 1, from_map_id.c_str(), -1, SQLITE_TRANSIENT);

    while (sqlite3_step(stmt) == SQLITE_ROW) {
        std::string to_map = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 0));
        double x = sqlite3_column_double(stmt, 1);
        double y = sqlite3_column_double(stmt, 2);
        neighbors.push_back({to_map, x, y});
    }

    sqlite3_finalize(stmt);
    return neighbors;
}

/// dijkstra algorithm to compute the map path, it gets its neighbors from the database which acts like a adjacency list
std::vector<std::string> dijkstra(sqlite3* db,
                                  const std::string &start_map,
                                  const std::string &goal_map,
                                  double start_x, double start_y) {
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> pq;
    std::unordered_map<std::string, double> dist;   
    std::unordered_map<std::string, std::string> parent; 

    pq.push({start_map, 0.0, ""});
    dist[start_map] = 0.0;

    while (!pq.empty()) {
        Node current = pq.top();
        pq.pop();

        if (current.map_id == goal_map)
            break;

        auto neighbors = get_neighbors(db, current.map_id);
        if (neighbors.empty()) {
            return {}; 
        }

        for (const auto &wh : neighbors) {
            // compute cost:
            // for first step, distance from robot start position
            // otherwise, assume robot is at (0,0) in local map frame
            double wormhole_cost = current.parent.empty()
                ? distance(start_x, start_y, wh.x, wh.y)
                : distance(0, 0, wh.x, wh.y);

            double new_cost = current.cost + wormhole_cost;

            if (!dist.count(wh.to_map) || new_cost < dist[wh.to_map]) {
                dist[wh.to_map] = new_cost;
                parent[wh.to_map] = current.map_id;
                pq.push({wh.to_map, new_cost, current.map_id});
            }
        }
    }

    std::vector<std::string> path;
    std::string at = goal_map;

    while (true) {
        path.push_back(at);
        auto it = parent.find(at);
        if (it == parent.end()) break; 
        at = it->second;
    }

    std::reverse(path.begin(), path.end());
    return path;
}
