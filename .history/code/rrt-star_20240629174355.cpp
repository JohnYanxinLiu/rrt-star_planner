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
        return a.first > b.first;  // Example: smaller doubles have higher priority
};

Graph::Graph()
{
    return;
}

void Graph::add_node(data_t d, double cost)
{
    data_map[get_key(d)] = d;
    cost_map[get_key(d)] = cost;
}

double Graph::cost(data_t d) { return cost_map[get_key(d)]; }

list<data_t> Graph::get_neighbors(data_t q, double radius)
{
    list<data_t> neighbors;
    for (auto it = data_map.begin(); it != data_map.end(); it++)
    {
        if (edge_cost(it->second, q) < radius)
        {
            neighbors.push_back(it->second);
        }
    }
    return neighbors;
}


list<data_t> Graph::k_nearest_neighbors(data_t q, size_t num_neighbors)
{
    priority_queue<std::pair<cost_t, data_t>, vector<std::pair<cost_t, data_t>>, cmp> pq;
    data_t q_nearest;

    
    for (auto it = data_map.begin(); it != data_map.end(); it++)
    {
        pq.push({edge_cost(it->second, q), it->second});
    }

    list<data_t> neighbors;
    for (size_t i = 0; i < min(num_neighbors, pq.size()); i++)
    {
        neighbors.push_back(pq.top().second);
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

        if (q_new[i] > 2 * PI)
        {
            q_new[i] -= 2 * PI;
        }
        if (q_new[i] < 0)
        {
            q_new[i] += 2 * PI;
        }
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
    cout << "Extracting path!" << endl;
    print_graph();
    for (q; q != q_start; q = data_map[parent_map[get_key(q)]])
    {
        pathList.push_front(q);
    }
    pathList.push_front(q_start);
    cout << "Path extracted!" << endl;

    *planlength = pathList.size();
    *plan = (double**) malloc((*planlength)*sizeof(double*));

    cout << "Plan length: " << *planlength << endl;
    for (int i = 0; !pathList.empty(); pathList.pop_front(), i++){
        
        (*plan)[i] = (double*) malloc(q_start.size()*sizeof(double));
        
        for(int j = 0; j < q_start.size(); j++){
            (*plan)[i][j] = pathList.front()[j];
        }

    }
    cout << "Plan extracted!" << endl;
}

void Graph::print_size() 
{ 
    cout << "Data map (size):" << data_map.size() << endl;
    cout << "Cost map (size):" << cost_map.size() << endl;
    cout << "Parent map (size):" << parent_map.size() << endl; 
}

void Graph::print_graph()
{
    cout << "Data map:" << endl;
    for (auto it = data_map.begin(); it != data_map.end(); it++)
    {
        cout << "Key: " << it->first << " Value: ";
        for (int i = 0; i < it->second.size(); i++)
        {
            cout << it->second[i] << " ";
        }

        cout << "Parent: " << parent_map[it->first] << " Cost: " << cost_map[it->first] << endl;
        cout << endl;
    }
}

bool Graph::isin(data_t q) { return data_map.find(get_key(q)) != data_map.end(); }

int collision_free(data_t anglesInitial, data_t anglesFinal, 
			int numofDOFs, double* map, int x_size, int y_size) {
	// std::vector<double> posInitial = manipulatorPosition(anglesInitial, x_size);
	// std::vector<double> posFinal = manipulatorPosition(anglesFinal, x_size);
	int numPoints = 2*(int)ceil(largestAngleDistance(anglesInitial, anglesFinal));

	// Create difference vector and current angles vector;
	data_t delta = init_data(numofDOFs);
	for (int i = 0; i < numofDOFs; i++){
		delta[i] = angularDisplacement(anglesInitial[i], anglesFinal[i])/ ((double)numPoints);
	}

	data_t currentAngles = anglesInitial;
	// Interpolate and check
	for (int j = 0; j < numPoints; j++){
		//update currentt angle to new angle.
		for(int i = 0; i < numofDOFs; i++){
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
            q_rand[i] = (static_cast<double>(rand()) / RAND_MAX) * (2 * M_PI);
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
    
    // check if q_new is already in the graph
    if (graph.isin(q_new)) return GOAL_NOT_FOUND;

    // print q_new
    // cout << "q_new: ";
    // for (int i = 0; i < q_new.size(); i++)
    // {
    //     cout << q_new[i] << " ";
    // }
    // cout << endl;

    if (collision_free(q_nearest, q_new, numofDOFs, map, x_size, y_size)) {
        double q_new_cost = graph.cost(q_nearest) + edge_cost(q_nearest, q_new);
        list<data_t> neighbors = graph.get_neighbors(q_new, REWIRING_RADIUS_SCALER * EPSILON);


        for (data_t q_neighbor : neighbors) 
        {   
            //rewiring step
            double new_cost = graph.cost(q_neighbor) + edge_cost(q_neighbor, q_new);    
        
            if (new_cost < graph.cost(q_new) && collision_free(q_neighbor, q_new, numofDOFs, map, x_size, y_size)) 
                graph.rewire(q_neighbor, q_new, new_cost);
        }

        graph.add_node(q_new, q_new_cost);
        graph.add_edge(q_nearest, q_new);


        if (near_goal(q_new, q_goal))
        {
            graph.add_node(q_goal, q_new_cost + edge_cost(q_new, q_goal));
            graph.add_edge(q_new, q_goal);
            *G = graph;
            return GOAL_FOUND;    
        }
    } 
    // else {
    //     cout << "not collision free!" << endl;
    // }
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
    graph.add_node(q_start, 0);

    graph.print_size();


    // data_t q_test1 = q_start;
    // q_test1[0] += 0.1;
    // q_test1[1] += 0.1;

    // data_t q_test2 = q_start;
    // q_test2[0] += 0.2;
    // q_test2[1] += 0.1;

    // data_t q_test3 = q_start;
    // q_test3[0] += 0.2;

    // cout << endl;

    // graph.add_node(q_test1, edge_cost(q_start, q_test1));
    // graph.add_edge(q_start, q_test1);

    // graph.add_node(q_test2, edge_cost(q_test1, q_test2));
    // graph.add_edge(q_test1, q_test2);

    // graph.print_graph();

    // return;
    
    size_t num_samples = 0;
    cout << "Starting RRT*" << endl;
    while (num_samples < K_SAMPLES) {
        cout << "Sample: " << num_samples << endl;
        data_t q_rand = random_sample(numofDOFs, x_size, y_size);
        

        // graph.print_graph();
        if (GOAL_FOUND == extend(&graph, q_rand, q_goal, numofDOFs, map, x_size, y_size))
        {
            cout << "Path found!" << endl;
            graph.print_graph();
            graph.extract_path(q_start, q_goal, plan, planlength);
            // cout << "Path found!" << endl;
            return;
        }
    
        num_samples++;
    
    }

    graph.print_size();
    graph.print_graph();
    cout << "no path found!" << endl;
    
    data_t nearest = graph.nearest_neighbor(q_goal);

    cout << "Nearest neighbor to goal: ";
    for (int i = 0; i < nearest.size(); i++)
    {
        cout << nearest[i] << " ";
    }
    cout << endl;

    cout << "Goal: ";
    for (int i = 0; i < q_goal.size(); i++)
    {
        cout << q_goal[i] << " ";
    }
    cout << endl;

    cout << endl << endl;

    cout << "edge cost: " << edge_cost(nearest, q_goal) << endl;


    return;
}