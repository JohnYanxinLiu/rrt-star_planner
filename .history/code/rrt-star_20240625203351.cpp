

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <random>
#include <chrono>
#include <unordered_map> 

using namespace std;

typedef double* data;

typedef std::unordered_map<std::string, data> data_graph;
typedef std::unordered_map<std::string, double> cost_graph;
typedef std::unordered_map<std::string, std::vector<std::string>> neighbor_graph;

class graph {
    private:
        data_graph data;
        cost_graph cost;
        neighbor_graph neighbor;
    public:
        graph();
        void add_node(data d){
            std::string key = std::to_string(d[0]) + std::to_string(d[1]);
            data[key] = d;
        }
        void add_edge(data d1, data d2){
            std::string key1 = std::to_string(d1[0]) + std::to_string(d1[1]);
            std::string key2 = std::to_string(d2[0]) + std::to_string(d2[1]);
            neighbor[key1].push_back(key2);
            neighbor[key2].push_back(key1);
            cost[key1] = sqrt(pow(d1[0] - d2[0], 2) + pow(d1[1] - d2[1], 2));
            cost[key2] = sqrt(pow(d1[0] - d2[0], 2) + pow(d1[1] - d2[1], 2));
        }
        void remove_node(data d);
        void remove_edge(data d1, data d2);
        void get_neighbors(data d);
        void get_cost(data d1, data d2);
        void get_data(data d);  
};

graph init_graph() {
    graph g;
    return g;
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

    graph g = init_graph();



}