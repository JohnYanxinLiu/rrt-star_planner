#include "rrt-star.h"

data_key_t get_key(data_t d)
{
    data_key_t key = "";
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

cost_t edge_cost(data_t q_start, data_t q_end)
{
    cost_t cost = 0;
    for (int i = 0; i < q_start.size(); i++){
        cost += abs(angularDisplacement(q_start[i], q_end[i]));
    }
    return cost;
}

bool near_goal(data_t q, data_t q_goal)
{
    return edge_cost(q, q_goal) < EPSILON;
}


bool Graph::cmp::operator()(const std::pair<cost_t, data_t>& a, const std::pair<cost_t, data_t>& b) const {
        // Compare based on the first element (double) of the pair
        // If a should come before b in the priority queue, return true
        return a.first < b.first;  // Example: smaller doubles have higher priority
};

Graph::Graph()
{
    return;
}

void Graph::add_node(data_t d)
{
    data_map[get_key(d)] = d;
}

double Graph::cost(data_t d) { return cost_map[get_key(d)]; }

vector<data_t> Graph::k_nearest_neighbors(data_t q, size_t num_neighbors)
{
    double min_dist = DBL_MAX;
    priority_queue<std::pair<double, data_t>, vector<std::pair<double, data_t>>, cmp> pq;
    data_t q_nearest;

    
    for (auto it = data_map.begin(); it != data_map.end(); it++)
    {
        pq.push({edge_cost(it->second, q), it->second});
    }

    vector<data_t> neighbors(num_neighbors);
    for (size_t i = 0; i < min(num_neighbors, pq.size()); i++)
    {
        neighbors[i] = pq.top().second;
        pq.pop();
    }

    return neighbors;
}

data_t Graph::nearest_neighbor(data_t q)
{
    return k_nearest_neighbors(q, 1).front();
}

void Graph::add_edge(data_t q_start, data_t q_end) { parent_map[get_key(q_end)] = get_key(q_start); }


data_t Graph::steer(data_t q_start, data_t q_end)
{
    data_t q_new = init_data(q_start.size()); 
    
    double dist = edge_cost(q_start, q_end); //TODO: Change this if the edge_cost is not the euclidean distance between two states.

    for (int i = 0; i < q_start.size(); i++)
    {
        double disp = angularDisplacement(q_start[i], q_end[i]);
        
        q_new[i] += (q_start[i] + EPSILON / dist * disp);
    }

    return q_new;
}


void Graph::rewire(data_t q_neighbor, data_t q_new, cost_t new_cost)
{
    parent_map[get_key(q_neighbor)] = get_key(q_new);
    cost_map[get_key(q_new)] = new_cost;
}


bool Graph::has_parent(data_t q) { return parent_map.find(get_key(q)) != parent_map.end(); }

void Graph::extract_path(data_t q_start, data_t q, double*** plan, int* planlength)
{
    list<data_t> pathList;
    
    for (q; q != q_start; q = data_map[parent_map[get_key(q)]])
    {
        pathList.push_front(q);
    }

    *planlength = pathList.size();
    *plan = (double**) malloc((*planlength)*sizeof(double*));

    for (int i = 0; !pathList.empty(); pathList.pop_front(), i++){
        
        (*plan)[i] = (double*) malloc(q_start.size()*sizeof(double));
        
        for(int j = 0; j < q_start.size(); j++){
            (*plan)[i][j] = pathList.front()[j];
        }

    }
}

int collision_free(data_t anglesInitial, data_t anglesFinal, 
			int numofDOFs, double* map, int x_size, int y_size) {
    int i;
	// std::vector<double> posInitial = manipulatorPosition(anglesInitial, x_size);
	// std::vector<double> posFinal = manipulatorPosition(anglesFinal, x_size);
	int numPoints = 2*(int)ceil(largestAngleDistance(anglesInitial, anglesFinal));

	// Create difference vector and current angles vector;
	data_t delta = init_data(numofDOFs);
	data_t currentAngles = anglesInitial;
	double totalDisp = 0;
	for (i = 0; i < numofDOFs; i++){
		delta[i] = angularDisplacement(anglesInitial[i], anglesFinal[i])/ ((double)numPoints);
	}

	
	// Interpolate and check
	for (int j = 0; j < numPoints; j++){
		//update currentt angle to new angle.
		for(i = 0; i < numofDOFs; i++){
			currentAngles[i] = currentAngles[i] + delta[i];
			if (currentAngles[i] > 2 * PI){
				currentAngles[i] = currentAngles[i] - 2 * PI;
			}
			if (currentAngles[i] < 0){
				currentAngles[i] = currentAngles[i] + 2 * PI;
			}
		}
		if(!IsValidArmConfiguration(currentAngles, map, x_size, y_size)){
			return 0; //If this point of the interpolation is not valid, return 0
		}
	}
	return 1;
}

Graph init_graph() {
    Graph g;
    return g;
}

data_t random_sample(int x_size, int y_size, int numofDOFs) {
    data_t q_rand = init_data(numofDOFs);
    
    while (1){
        for (int i = 0; i < numofDOFs; i++){
            q_rand[i] = (double) ((rand()%(359 + 1)) * (PI / 180));
        }
        vector<double> pos = manipulatorPosition(q_rand, x_size);
        
        if(pos[0] >= 0 && pos[0] < x_size && pos[1] >= 0 && pos[1] < y_size){
            return q_rand;
        }
    }
}


GOAL_STATUS extend(Graph *G, data_t q_rand, data_t q_goal, int numofDOFs, double* map, int x_size, int y_size) {
    Graph graph = *G;
    data_t q_nearest = graph.nearest_neighbor(q_rand);
    data_t q_new = graph.steer(q_nearest, q_rand);
    if (collision_free(q_nearest, q_new, numofDOFs, map, x_size, y_size)) {
        
        graph.add_node(q_new);
        graph.add_edge(q_nearest, q_new);
        
        vector<data_t> neighbors = graph.k_nearest_neighbors(q_new, K_NEIGHBORS);
        
        for (data_t q_neighbor : neighbors) 
        {
            //rewiring step
            double new_cost = graph.cost(q_neighbor) + edge_cost(q_neighbor, q_new);
            
            if (new_cost < graph.cost(q_new) && collision_free(q_neighbor, q_new, numofDOFs, map, x_size, y_size)) 
                graph.rewire(q_neighbor, q_new, new_cost);
        }

        if (near_goal(q_new, q_goal))
        {
            *G = graph;
            return GOAL_FOUND;    
        }
    }

    *G = graph;
    return GOAL_NOT_FOUND;
}


void planner(
        double* map,
        int x_size,
        int y_size,
        double* start,
        double* goal,
        int numofDOFs,
        double*** plan,
        int* planlength)
{
    
    Graph graph = init_graph();
    data_t q_start = init_data(numofDOFs);
    data_t q_goal = init_data(numofDOFs);
    for (int i = 0; i < numofDOFs; i++) {
        q_start[i] = start[i];
        q_goal[i] = goal[i];
    }
    graph.add_node(q_start);

    size_t num_samples = 0;
    while (num_samples < K_SAMPLES) {
        data_t q_rand = random_sample(numofDOFs, x_size, y_size);
        if (GOAL_FOUND == extend(&graph, q_rand, q_goal, numofDOFs, map, x_size, y_size))
        {
            graph.extract_path(q_start, q_goal, plan, planlength);
            return;
        }
    
        num_samples++;
    
    }

    return;
}