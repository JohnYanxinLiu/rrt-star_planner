#ifndef RRT_STAR_H
#define RRT_STAR_H

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <random>
#include <chrono>
#include <unordered_map>
#include <list>
#include <queue>
#include <float.h>

#include "arm_planner_util.h"

#define K_NEIGHBORS 4
#define K_SAMPLES 1000

#define GOAL_FOUND 1
#define GOAL_NOT_FOUND 0
#define GOAL_STATUS int

using namespace std;

typedef vector<double> data_t;
typedef string data_key_t;
typedef double cost_t;

typedef unordered_map<data_key_t, data_t> data_graph;
typedef unordered_map<data_key_t, cost_t> cost_graph;
typedef unordered_map<data_key_t, data_key_t> parent_graph;

data_key_t get_key(data_t d);

data_t init_data(int numofDOFs);

cost_t edge_cost(data_t q_start, data_t q_end);

bool near_goal(data_t q, data_t q_goal);


class Graph {
    // typedef data_t data_t;

    private:
        data_graph data_map;
        cost_graph cost_map;
        parent_graph parent_map;

        struct cmp {
            bool operator()(const std::pair<cost_t, data_t>& a, const std::pair<cost_t, data_t>& b) const;
        };

    public:
        Graph();
        
        void add_node(data_t d);

        double cost(data_t d);

        vector<data_t> k_nearest_neighbors(data_t q, size_t num_neighbors);

        data_t nearest_neighbor(data_t q);

        void add_edge(data_t q_start, data_t q_end);

        data_t steer(data_t q_start, data_t q_end);

        void rewire(data_t q_neighbor, data_t q_new, cost_t new_cost);

        bool has_parent(data_t q);

        void extract_path(data_t q_start, data_t q, double*** plan, int* planlength);

        void print_size();

        void print_graph();
};

void planner(
			double* map,
			int x_size,
			int y_size,
			double* start,
			double* goal,
            int numofDOFs,
            double*** plan,
            int* planlength);


#endif // RRT_STAR_H

