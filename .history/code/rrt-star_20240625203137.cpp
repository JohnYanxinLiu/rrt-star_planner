

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <random>
#include <chrono>
#include <unordered_map> 

typedef double* data;

typedef std::unordered_map<std::string, data> data_graph;
typedef std::unordered_map<std::string, double> cost_graph;
typedef std::unordered_map<std::string, std::vector<std::string>> neighbor_graph;

typedef struct graph {
    data_graph data;
    cost_graph cost;
    neighbor_graph neighbor;
} graph;

static void planner(
			double* map,
			int x_size,
			int y_size,
			data start,
			data goal,
            int numofDOFs,
            data** plan,
            int* planlength)
{



}