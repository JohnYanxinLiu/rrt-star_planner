

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
    typedef data data;

    private:
        data_graph data_map;
        cost_graph cost_map;
        neighbor_graph neighbor_map;
    public:
        graph();
        
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