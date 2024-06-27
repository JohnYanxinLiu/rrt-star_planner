

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

typedef vector<double> data_t;
typedef string key_t;

typedef unordered_map<key_t, data_t> data_graph;
typedef unordered_map<key_t, double> cost_graph;
typedef unordered_map<key_t, vector<key_t>> neighbor_graph;

key_t get_key(data_t d)
{
    key_t key = "";
    for (int i = 0; i < d.size(); i++)
    {
        key += to_string(d[i]) + ",";
    }
    return key;
}

data_t init_data(int numofDOFs)
{
    data_t data(numofDOFs);
    return data;
}

class Graph {
    typedef data_t data_t;

    private:
        data_graph data_map;
        cost_graph cost_map;
        neighbor_graph neighbor_map;
    public:
        Graph();
        
        void add_node(data_t d)
        {
            data_map[get_key(d)] = d;
        }

        data_t nearest_neighbor(data_t q_rand)
        {
            double min_dist = 1000000;
            data_t q_nearest;
            for (auto it = data_map.begin(); it != data_map.end(); it++)
            {
                double dist = 0;
                for (int i = 0; i < q_rand.size(); i++)
                {
                    dist += 0;// TODO: distance calculation
                    abort("not implemented")
                }
                dist = sqrt(dist);
                if (dist < min_dist)
                {
                    min_dist = dist;
                    q_nearest = it->second;
                }
            }
            return q_nearest;
        }

        data_t steer(data_t q_nearest, data_t q_rand)
        {
            data_t q_new = q_nearest;
            for (int i = 0; i < q_nearest.size(); i++)
            {
                q_new[i] = q_nearest[i] + 0; //TODO: steer
            }
            return q_new;
        }
};

Graph init_graph() {
    Graph g;
    return g;
}

data_t random_sample(int numofDOFs) {
    data_t q_rand = init_data(numofDOFs);
    for (int i = 0; i < numofDOFs; i++) {
        //TODO: random sample
    }
    return q_rand;
}


void extend(Graph *G, data_t q_nearest, data_t q_rand, double* map, int x_size, int y_size) {
    Graph graph = *G;
    data_t q_new = graph.steer(q_nearest, q_rand);
    if (collision_free(q_nearest, q_new, map, x_size, y_size)) {
        graph.add_node(q_new);
        graph.add_edge(q_nearest, q_new);
    }

    *G = graph;
    return;
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
        data q_nearest = graph.nearest_neighbor(q_rand);
        data q_new = graph.steer(q_nearest, q_rand);
        if (collision_free(q_nearest, q_new, map, x_size, y_size)) {
            graph.add_node(q_new);
            graph.add_edge(q_nearest, q_new);
        }
    }
}