

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <random>
#include <chrono>
#include <unordered_map> 

using namespace std;

typedef double* data;

typedef unordered_map<string, data> data_graph;
typedef unordered_map<string, double> cost_graph;
typedef unordered_map<string, vector<string>> neighbor_graph;

class graph {
    typedef data_t data;
    private:
        data_graph data;
        cost_graph cost;
        neighbor_graph neighbor;
    public:
        graph();
        void add_node(data d){
            string key = to_string(d[0]) + "," + to_string(d[1]);
            data[key] = d;
        }
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