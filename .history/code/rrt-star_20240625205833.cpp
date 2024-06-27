

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <random>
#include <chrono>
#include <unordered_map>
#include <list>


#define K_NEIGHBORS 4
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
        
        void add_node(data d)
        {
            string key = "";
            for (int i = 0; i < d.size(); i++)
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
    data q_rand = data(numofDOFs);
    for (int i = 0; i < numofDOFs; i++) {
        //TODO: random sample
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
    graph.add_node(start);

    //TODO: write terminating condition
    while (goal_not_found()) {
        data q_rand = random_sample(numofDOFs);

        //Extend phase
        data q_nearest = nearest_neighbor(q_rand, graph);
        data q_new = steer(q_nearest, q_rand, graph);
        if (collision_free(q_nearest, q_new, map, x_size, y_size)) {
            graph.add_node(q_new);
            graph.add_edge(q_nearest, q_new);
        }
    }
}