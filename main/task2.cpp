#include <iostream>
#include <fstream>
#include "MAPFInstance.h"
#include "AStarPlanner.h"
#include <tuple>

using namespace std;

int main(int argc, char *argv[]) {
    MAPFInstance ins;
    string input_file = argv[1];
    string output_file = argv[2];
    if (ins.load_instance(input_file)) {
        ins.print_instance();
    } else {
        cout << "Fail to load the instance " << input_file << endl;
        exit(-1);
    }

    AStarPlanner a_star(ins);
    vector<Path> paths(ins.num_of_agents);

    // assign priority ordering to agents
    // By default, we use the index ordering of the agents where
    // the first always has the highest priority.
    list<int> priorities;
    for (int i = 0; i < ins.num_of_agents; i++) {
        priorities.push_back(i);
    }

    // plan paths
    // priorities.emplace(priorities.begin(), 2);
    list<Constraint> constraints;
    for (int i : priorities) {
        // Transform already planned paths into constraints
        paths[i] = a_star.find_path(i, constraints);

        if (!paths[i].empty()) {
            int t = 1;
            int last_node = paths[i][0];

            // modify all the previous constraints to consider this new agent
            list<Constraint> tmp = constraints;
            constraints.clear();
            for (auto c : tmp) {
                 constraints.push_back(make_tuple(i + 1, get<1>(c), get<2>(c), get<3>(c)));
            }

            for (int k = 1; k < paths[i].size(); k++) {
                int node = paths[i][k];
                // goal constraints
                if (node == ins.goal_locations[i + 1]) {
                    constraints.push_back(make_tuple(i + 1, node, -1, t - 1));
                }
                else if (node == ins.goal_locations[i]) {
                    for (int x = 1; x <= ins.map_size(); x++) {
                        constraints.push_back(make_tuple(i + 1, node, -1, t + x));
                    }
                }
                constraints.push_back(make_tuple(i + 1, node, -1, t));
                constraints.push_back(make_tuple(i + 1, last_node, node, t));
                constraints.push_back(make_tuple(i + 1, node, last_node, t));
                
                last_node = node;
                t++;
            }
        }
        if (i > 0) {
            if (paths[i].empty() || paths[i].size() >= (paths[i-1].size() + ins.map_size())) {
                cout << "Fail to find any solutions for agent " << i << endl;
                return 0;
            }
        }
        else {
            if (paths[i].empty()) {
                cout << "Fail to find any solutions for agent " << i << endl;
                return 0;
            }
        }
    }

    // print paths
    cout << "Paths:" << endl;
    int sum = 0;
    for (int i = 0; i < ins.num_of_agents; i++) {
        cout << "a" << i << ": " << paths[i] << endl;
        sum += (int)paths[i].size() - 1;
    }
    cout << "Sum of cost: " << sum << endl;

    // save paths
    ofstream myfile (output_file.c_str(), ios_base::out);
    if (myfile.is_open()) {
        for (int i = 0; i < ins.num_of_agents; i++) {
            myfile << paths[i] << endl;
        }
        myfile.close();
    } else {
        cout << "Fail to save the paths to " << output_file << endl;
        exit(-1);
    }
    return 0;
}