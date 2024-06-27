

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <random>
#include <chrono>
#include <unordered_map> 

using namespace std;

typedef vector<double> data;

typedef unordered_map<string, data> data_graph;
typedef unordered_map<string, double> cost_graph;
typedef unordered_map<string, vector<string>> neighbor_graph;

class Graph {
    typedef data data;

    private:
        data_graph data_map;
        cost_graph cost_map;
        neighbor_graph neighbor_map;
    public:
        Graph();
        
        void add_node(data d, int numofDOFs)
        {
            string key = "";
            for (int i = 0; i < numofDOFs; i++)
            {
                key += to_string(d[i]) + ",";
            }
            data_map[key] = d;
        }
};

Graph init_graph() {
    Graph g;
    return g;
}

data random_sample(int numofDOFs) {
    data q_rand = vector(numofDOFs);
    for (int i = 0; i < numofDOFs; i++) {
        q_rand[i] = (double)rand() / RAND_MAX;
    }
    return q_rand;
}

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
    Graph graph = init_graph();
    graph.add_node(start, numofDOFs);

    while (goal_not_found) {
        data q_rand = random_sample();
        data q_near = nearest_neighbor(q_rand, graph);
        data q_new = steer(q_rand, q_near, graph);
        if (collision_free(q_near, q_new, map, x_size, y_size)) {
            graph.add_node(q_new, numofDOFs);
            graph.add_edge(q_near, q_new, cost(q_near, q_new));
            graph.rewire(q_near, q_new);
        }        
    }

}